#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdarg>
#include <cstring>
#include <damiao_can/can/socket/damiao_can.hpp>
#include <damiao_can/canbus/can_socket.hpp>
#include <iostream>
#include <stdexcept>

// Replace only CAN setup at the linker boundary. Reads, select, writes and close
// execute the production CANSocket implementation on a local datagram socket.
namespace {
int peer_fd = -1;
int socket_fd = -1;
size_t reads = 0;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
}  // namespace

extern "C" ssize_t __real_read(int fd, void* buffer, size_t length);
extern "C" ssize_t __wrap_read(int fd, void* buffer, size_t length) {
    if (fd == socket_fd) ++reads;
    return __real_read(fd, buffer, length);
}

extern "C" int __real_socket(int domain, int type, int protocol);
extern "C" int __wrap_socket(int domain, int type, int protocol) {
    if (domain != PF_CAN) return __real_socket(domain, type, protocol);
    int pair[2];
    if (::socketpair(AF_UNIX, SOCK_DGRAM, 0, pair) < 0) return -1;
    socket_fd = pair[0];
    peer_fd = pair[1];
    return socket_fd;
}

extern "C" int __real_ioctl(int fd, unsigned long request, ...);
extern "C" int __wrap_ioctl(int fd, unsigned long request, ...) {
    va_list args;
    va_start(args, request);
    void* argument = va_arg(args, void*);
    va_end(args);
    if (fd == socket_fd && request == SIOCGIFINDEX) {
        static_cast<ifreq*>(argument)->ifr_ifindex = 1;
        return 0;
    }
    return __real_ioctl(fd, request, argument);
}

extern "C" int __real_bind(int fd, const sockaddr* address, socklen_t length);
extern "C" int __wrap_bind(int fd, const sockaddr* address, socklen_t length) {
    if (fd == socket_fd && address->sa_family == AF_CAN) return 0;
    return __real_bind(fd, address, length);
}

extern "C" int __real_setsockopt(int fd, int level, int option, const void* value,
                                 socklen_t length);
extern "C" int __wrap_setsockopt(int fd, int level, int option, const void* value,
                                 socklen_t length) {
    if (fd == socket_fd && level == SOL_CAN_RAW) return 0;
    return __real_setsockopt(fd, level, option, value, length);
}

int main() {
    try {
        {
            damiao_can::canbus::CANSocket socket("offline", true);
            can_frame ack{};
            ack.can_id = 0x6F1;
            ack.can_dlc = 8;
            ack.__pad = 0xFF;  // Classic padding must never become FD flags.
            ack.data[0] = 1;
            ack.data[1] = 0x80;
            canfd_frame sample{};
            sample.can_id = 0x6F2;
            sample.len = 64;
            sample.flags = CANFD_BRS;
            sample.data[0] = 1;
            sample.data[1] = 0x90;
            sample.data[63] = 0x42;
            require(::write(peer_fd, &ack, CAN_MTU) == CAN_MTU, "enqueue classic ACK");
            require(::write(peer_fd, &sample, CANFD_MTU) == CANFD_MTU, "enqueue FD sample");
            require(::write(peer_fd, &ack, CAN_MTU) == CAN_MTU, "enqueue second ACK");
            canfd_frame frame{};
            require(socket.is_data_available(10000), "mixed input available");
            require(socket.read_canfd_frame(frame), "read classic on FD-enabled socket");
            require(frame.can_id == ack.can_id && frame.len == 8 && frame.flags == 0 &&
                        frame.data[1] == 0x80 && frame.data[63] == 0,
                    "classic frame normalized without stale payload or padding flags");
            require(socket.read_canfd_frame(frame), "read FD after classic");
            require(frame.can_id == sample.can_id && frame.len == 64 && frame.flags == CANFD_BRS &&
                        frame.data[63] == 0x42,
                    "FD64 preserved");
            require(socket.read_canfd_frame(frame) && frame.flags == 0 && frame.data[63] == 0,
                    "classic after FD clears stale bytes");
            require(!socket.is_data_available(0), "bounded input consumed");
        }
        char byte = 0;
        require(::read(socket_fd, &byte, 1) == -1 && errno == EBADF,
                "CANSocket destructor closes owned descriptor");
        ::close(peer_fd);
        {
            using namespace damiao_can::damiao_motor;
            damiao_can::can::socket::DamiaoCAN motor("offline", true);
            motor.init_motors({1}, {0x11}, std::vector<MotorType>{MotorType::DM4310}, {}, {0.15},
                              {true});
            const size_t before = reads;
            canfd_frame sent{};
            const auto receive_command = [&]() {
                require(::recv(peer_fd, &sent, sizeof(sent), MSG_DONTWAIT) == CANFD_MTU,
                        "legacy command transmitted without automatic reply");
            };
            motor.mit_control_one(0, MITParam{0, 0, 0, 0, 0.5});
            receive_command();
            require(sent.can_id == 1 && sent.len == 8, "MIT command format preserved");
            motor.set_control_mode_one(0, ControlMode::VEL);
            receive_command();
            motor.vel_control_one(0, VelParam{1.0});
            receive_command();
            require(sent.can_id == 0x201 && sent.len == 8, "velocity mode format preserved");
            float velocity = 0;
            std::memcpy(&velocity, sent.data, sizeof(velocity));
            require(velocity == -1.0f, "legacy reversed velocity command preserved");
            motor.set_control_mode_one(0, ControlMode::POS_VEL);
            receive_command();
            motor.posvel_control_one(0, PosVelParam{0.15, 20});
            receive_command();
            require(sent.can_id == 0x101, "position/velocity command format preserved");
            motor.set_control_mode_one(0, ControlMode::POS_FORCE);
            receive_command();
            motor.posforce_control_one(0, PosForceParam{0.15, 20, 0.5});
            receive_command();
            require(sent.can_id == 0x301, "position/force command format preserved");
            require(reads == before, "no-response control/mode paths never wait for feedback");

            // Telemetry/ACKs on this independently owned legacy socket are unrelated;
            // they must not satisfy the expected motor response or cause an exception.
            can_frame ack{};
            ack.can_id = 0x6F1;
            ack.can_dlc = 8;
            canfd_frame telemetry{};
            telemetry.can_id = 0x6F2;
            telemetry.len = 64;
            telemetry.flags = CANFD_BRS;
            canfd_frame feedback{};
            feedback.can_id = 0x11;
            feedback.len = 8;
            feedback.flags = CANFD_BRS;
            feedback.data[0] = 0xD1;  // J1 communication/safety fault.
            require(::write(peer_fd, &ack, CAN_MTU) == CAN_MTU, "legacy ACK traffic");
            require(::write(peer_fd, &telemetry, CANFD_MTU) == CANFD_MTU, "legacy sample traffic");
            require(::write(peer_fd, &feedback, CANFD_MTU) == CANFD_MTU,
                    "explicit legacy feedback");
            const auto received = motor.recv_all(10000);
            require(received.ok && received.expect == 1 && received.received == 1,
                    "legacy receive ignores sysid traffic and accepts motor feedback");
            require(motor.get_motor(0).get_fault() == 13,
                    "holder fault preserved from feedback header");
            feedback.len = 7;
            require(::write(peer_fd, &feedback, CANFD_MTU) == CANFD_MTU,
                    "inject truncated motor reply");
            auto invalid = motor.recv_all(3000);
            require(!invalid.ok && invalid.received == 0 && invalid.missing.size() == 1,
                    "short motor packet cannot satisfy explicit refresh");
            feedback.len = 8;
            feedback.can_id |= CAN_EFF_FLAG;
            require(::write(peer_fd, &feedback, CANFD_MTU) == CANFD_MTU,
                    "inject wrong motor ID format");
            invalid = motor.recv_all(3000);
            require(!invalid.ok && invalid.received == 0, "extended alias cannot satisfy refresh");
            require(motor.get_motor(0).get_fault() == 13,
                    "invalid replies never replace valid fault");
        }
        ::close(peer_fd);
        std::cout << "mixed classic/FD legacy transport and cleanup passed\n";
        return 0;
    } catch (const std::exception& error) {
        if (peer_fd >= 0) ::close(peer_fd);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
