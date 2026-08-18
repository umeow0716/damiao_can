// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <damiao_can/canbus/can_socket.hpp>
#include <stdexcept>
#include <string>

namespace damiao_can::canbus {
namespace {

std::string errno_message(const std::string& action) {
    return action + ": " + std::strerror(errno);
}

}  // namespace

CANSocket::CANSocket(const std::string& interface, bool enable_fd)
    : socket_fd_(-1), interface_(interface), fd_enabled_(enable_fd) {
    initialize_socket(interface);
}

CANSocket::~CANSocket() { cleanup(); }

void CANSocket::initialize_socket(const std::string& interface) {
    if (interface.empty()) {
        throw std::invalid_argument("CAN interface name must not be empty");
    }
    if (interface.size() >= IFNAMSIZ) {
        throw std::invalid_argument("CAN interface name is too long: " + interface);
    }

    socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        throw CANSocketException(errno_message("failed to create SocketCAN socket"));
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        const std::string message = errno_message("failed to resolve CAN interface " + interface);
        cleanup();
        throw CANSocketException(message);
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (fd_enabled_) {
        int enable_canfd = 1;
        if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd,
                         sizeof(enable_canfd)) < 0) {
            const std::string message = errno_message("failed to enable CAN-FD frames");
            cleanup();
            throw CANSocketException(message);
        }
    }

    if (::bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        const std::string message =
            errno_message("failed to bind SocketCAN interface " + interface);
        cleanup();
        throw CANSocketException(message);
    }

    struct timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100;
    if (::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        const std::string message = errno_message("failed to set SocketCAN receive timeout");
        cleanup();
        throw CANSocketException(message);
    }
}

void CANSocket::cleanup() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

ssize_t CANSocket::read_raw_frame(void* buffer, size_t buffer_size) {
    if (!is_initialized()) {
        throw CANSocketException("socket is not initialized");
    }

    ssize_t result = -1;
    do {
        result = ::read(socket_fd_, buffer, buffer_size);
    } while (result < 0 && errno == EINTR);

    if (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        throw CANSocketException(errno_message("failed to read SocketCAN frame"));
    }
    return result;
}

ssize_t CANSocket::write_raw_frame(const void* buffer, size_t frame_size) {
    if (!is_initialized()) {
        throw CANSocketException("socket is not initialized");
    }

    ssize_t result = -1;
    do {
        result = ::write(socket_fd_, buffer, frame_size);
    } while (result < 0 && errno == EINTR);

    if (result < 0) {
        throw CANSocketException(errno_message("failed to write SocketCAN frame"));
    }
    if (static_cast<size_t>(result) != frame_size) {
        throw CANSocketException("short SocketCAN write: expected " + std::to_string(frame_size) +
                                 " bytes, wrote " + std::to_string(result));
    }
    return result;
}

bool CANSocket::write_can_frame(const can_frame& frame) {
    return write_raw_frame(&frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

bool CANSocket::write_canfd_frame(const canfd_frame& frame) {
    return write_raw_frame(&frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

bool CANSocket::read_can_frame(can_frame& frame) {
    const ssize_t bytes_read = read_raw_frame(&frame, sizeof(frame));
    if (bytes_read < 0) return false;
    if (bytes_read != static_cast<ssize_t>(sizeof(frame))) {
        throw CANSocketException(
            "unexpected classic CAN frame size: " + std::to_string(bytes_read) + " bytes");
    }
    return true;
}

bool CANSocket::read_canfd_frame(canfd_frame& frame) {
    const ssize_t bytes_read = read_raw_frame(&frame, sizeof(frame));
    if (bytes_read < 0) return false;
    if (bytes_read != static_cast<ssize_t>(sizeof(frame))) {
        throw CANSocketException("unexpected CAN-FD frame size: " + std::to_string(bytes_read) +
                                 " bytes");
    }
    return true;
}

bool CANSocket::is_data_available(int timeout_us) {
    if (!is_initialized()) {
        throw CANSocketException("socket is not initialized");
    }
    if (timeout_us < 0) {
        throw std::invalid_argument("timeout_us must be non-negative");
    }

    using clock = std::chrono::steady_clock;
    using microseconds = std::chrono::microseconds;
    const auto deadline = clock::now() + microseconds(timeout_us);

    while (true) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(socket_fd_, &read_fds);

        const auto now = clock::now();
        const auto remaining = now >= deadline
                                   ? microseconds(0)
                                   : std::chrono::duration_cast<microseconds>(deadline - now);
        struct timeval timeout{};
        timeout.tv_sec = static_cast<time_t>(remaining.count() / 1000000);
        timeout.tv_usec = static_cast<suseconds_t>(remaining.count() % 1000000);

        const int result = ::select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
        if (result > 0) return FD_ISSET(socket_fd_, &read_fds);
        if (result == 0) return false;
        if (errno == EINTR) {
            if (clock::now() >= deadline) return false;
            continue;
        }
        throw CANSocketException(errno_message("failed while waiting for SocketCAN data"));
    }
}

}  // namespace damiao_can::canbus
