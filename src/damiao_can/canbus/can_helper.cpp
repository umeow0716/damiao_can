// Copyright 2026 Enactic, Inc.
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

#include <damiao_can/canbus/can_helper.hpp>

#include <linux/can/netlink.h>
#include <linux/capability.h>
#include <linux/if_link.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace damiao_can::canbus {
namespace {

class ScopedFd {
public:
    explicit ScopedFd(int fd = -1) : fd_(fd) {}
    ~ScopedFd() {
        if (fd_ >= 0) ::close(fd_);
    }
    ScopedFd(const ScopedFd&) = delete;
    ScopedFd& operator=(const ScopedFd&) = delete;
    int get() const noexcept { return fd_; }

private:
    int fd_;
};

std::string errno_message(const std::string& prefix) {
    return prefix + ": " + std::strerror(errno);
}

bool has_cap_net_admin() noexcept {
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line)) {
        constexpr const char* prefix = "CapEff:\t";
        if (line.rfind(prefix, 0) != 0) continue;

        try {
            const unsigned long long mask = std::stoull(line.substr(std::strlen(prefix)), nullptr, 16);
            return (mask & (1ULL << CAP_NET_ADMIN)) != 0;
        } catch (...) {
            return false;
        }
    }
    return false;
}

bool has_controlling_terminal() noexcept {
    ScopedFd tty(::open("/dev/tty", O_RDWR | O_CLOEXEC));
    return tty.get() >= 0;
}

std::string find_ip_executable() {
    for (const char* candidate : {"/usr/sbin/ip", "/sbin/ip", "/usr/bin/ip", "/bin/ip"}) {
        if (::access(candidate, X_OK) == 0) return candidate;
    }
    return "ip";
}

int run_process(const std::vector<std::string>& args, bool quiet = false) {
    if (args.empty()) return 127;

    const pid_t pid = ::fork();
    if (pid < 0) throw CANHelperException(errno_message("fork failed"));

    if (pid == 0) {
        if (quiet) {
            const int null_fd = ::open("/dev/null", O_WRONLY | O_CLOEXEC);
            if (null_fd >= 0) {
                ::dup2(null_fd, STDERR_FILENO);
                ::close(null_fd);
            }
        }

        std::vector<char*> argv;
        argv.reserve(args.size() + 1);
        for (const auto& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
        argv.push_back(nullptr);

        ::execvp(argv[0], argv.data());
        _exit(errno == ENOENT ? 127 : 126);
    }

    int status = 0;
    while (::waitpid(pid, &status, 0) < 0) {
        if (errno == EINTR) continue;
        throw CANHelperException(errno_message("waitpid failed"));
    }

    if (WIFEXITED(status)) return WEXITSTATUS(status);
    if (WIFSIGNALED(status)) return 128 + WTERMSIG(status);
    return 125;
}

void run_ip_mutation(const std::vector<std::string>& ip_args) {
    std::vector<std::string> command;
    const std::string ip = find_ip_executable();
    const bool direct = has_cap_net_admin();
    const bool has_tty = has_controlling_terminal();

    if (direct) {
        command.push_back(ip);
    } else {
        command.push_back("sudo");
        if (!has_tty) command.push_back("-n");
        command.insert(command.end(), {"--", ip});
    }
    command.insert(command.end(), ip_args.begin(), ip_args.end());

    const int rc = run_process(command);
    if (rc == 127) {
        throw CANHelperException(direct ? "ip executable was not found"
                                        : "sudo executable was not found");
    }
    if (rc != 0) {
        if (!direct && !has_tty) {
            throw CANHelperException(
                "CAN configuration requires CAP_NET_ADMIN; sudo could not complete "
                "non-interactively because no controlling terminal is available (exit " +
                std::to_string(rc) + ")");
        }
        throw CANHelperException("failed to configure CAN interface (command exited with " +
                                 std::to_string(rc) + ")");
    }
}

std::string format_ratio(double value) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6) << value;
    std::string text = stream.str();
    while (!text.empty() && text.back() == '0') text.pop_back();
    if (!text.empty() && text.back() == '.') text.pop_back();
    return text;
}

void validate_config(const CANInterfaceConfig& config) {
    if (config.bitrate == 0) throw CANHelperException("CAN bitrate must be greater than zero");
    if (config.fd_enabled && config.dbitrate == 0) {
        throw CANHelperException("CAN-FD data bitrate must be greater than zero");
    }
    if (config.sample_point < 0.0 || config.sample_point >= 1.0) {
        throw CANHelperException("sample_point must be 0 (automatic) or in the range (0, 1)");
    }
    if (config.dsample_point < 0.0 || config.dsample_point >= 1.0) {
        throw CANHelperException("dsample_point must be 0 (automatic) or in the range (0, 1)");
    }
}

void parse_can_info_data(rtattr* data_attr, CANInterfaceStatus& status) {
    int remaining = RTA_PAYLOAD(data_attr);
    for (rtattr* attr = static_cast<rtattr*>(RTA_DATA(data_attr)); RTA_OK(attr, remaining);
         attr = RTA_NEXT(attr, remaining)) {
        switch (attr->rta_type) {
            case IFLA_CAN_BITTIMING:
                if (RTA_PAYLOAD(attr) >= sizeof(can_bittiming)) {
                    const auto* timing = static_cast<const can_bittiming*>(RTA_DATA(attr));
                    if (timing->bitrate != 0) status.bitrate = timing->bitrate;
                }
                break;
            case IFLA_CAN_DATA_BITTIMING:
                if (RTA_PAYLOAD(attr) >= sizeof(can_bittiming)) {
                    const auto* timing = static_cast<const can_bittiming*>(RTA_DATA(attr));
                    if (timing->bitrate != 0) status.dbitrate = timing->bitrate;
                }
                break;
            case IFLA_CAN_CTRLMODE:
                if (RTA_PAYLOAD(attr) >= sizeof(can_ctrlmode)) {
                    const auto* ctrlmode = static_cast<const can_ctrlmode*>(RTA_DATA(attr));
                    status.fd_enabled = (ctrlmode->flags & CAN_CTRLMODE_FD) != 0;
                }
                break;
            case IFLA_CAN_RESTART_MS:
                if (RTA_PAYLOAD(attr) >= sizeof(uint32_t)) {
                    status.restart_ms = *static_cast<const uint32_t*>(RTA_DATA(attr));
                }
                break;
            default:
                break;
        }
    }
}

void parse_link_info(rtattr* link_info, CANInterfaceStatus& status) {
    rtattr* info_data = nullptr;
    int remaining = RTA_PAYLOAD(link_info);
    for (rtattr* attr = static_cast<rtattr*>(RTA_DATA(link_info)); RTA_OK(attr, remaining);
         attr = RTA_NEXT(attr, remaining)) {
        if (attr->rta_type == IFLA_INFO_KIND && RTA_PAYLOAD(attr) > 0) {
            const auto* kind = static_cast<const char*>(RTA_DATA(attr));
            status.is_can = std::strcmp(kind, "can") == 0;
        } else if (attr->rta_type == IFLA_INFO_DATA) {
            info_data = attr;
        }
    }

    if (status.is_can && info_data != nullptr) parse_can_info_data(info_data, status);
}

CANInterfaceStatus query_status(const std::string& interface) {
    CANInterfaceStatus result;
    errno = 0;
    const unsigned int ifindex = ::if_nametoindex(interface.c_str());
    if (ifindex == 0) return result;

    result.exists = true;
    result.ifindex = static_cast<int>(ifindex);

    ScopedFd fd(::socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE));
    if (fd.get() < 0) throw CANHelperException(errno_message("failed to open rtnetlink socket"));

    sockaddr_nl local{};
    local.nl_family = AF_NETLINK;
    if (::bind(fd.get(), reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
        throw CANHelperException(errno_message("failed to bind rtnetlink socket"));
    }

    struct {
        nlmsghdr header;
        ifinfomsg info;
    } request{};
    request.header.nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
    request.header.nlmsg_type = RTM_GETLINK;
    request.header.nlmsg_flags = NLM_F_REQUEST;
    request.header.nlmsg_seq = 1;
    request.info.ifi_family = AF_UNSPEC;
    request.info.ifi_index = static_cast<int>(ifindex);

    sockaddr_nl kernel{};
    kernel.nl_family = AF_NETLINK;
    if (::sendto(fd.get(), &request, request.header.nlmsg_len, 0,
                 reinterpret_cast<sockaddr*>(&kernel), sizeof(kernel)) < 0) {
        throw CANHelperException(errno_message("failed to query CAN interface"));
    }

    alignas(nlmsghdr) char buffer[8192];
    while (true) {
        const ssize_t received = ::recv(fd.get(), buffer, sizeof(buffer), 0);
        if (received < 0) {
            if (errno == EINTR) continue;
            throw CANHelperException(errno_message("failed to receive CAN interface status"));
        }

        int remaining = static_cast<int>(received);
        for (nlmsghdr* header = reinterpret_cast<nlmsghdr*>(buffer); NLMSG_OK(header, remaining);
             header = NLMSG_NEXT(header, remaining)) {
            if (header->nlmsg_type == NLMSG_ERROR) {
                const auto* error = static_cast<const nlmsgerr*>(NLMSG_DATA(header));
                if (error->error == 0) continue;
                errno = -error->error;
                throw CANHelperException(errno_message("rtnetlink query failed"));
            }
            if (header->nlmsg_type == NLMSG_DONE) return result;
            if (header->nlmsg_type != RTM_NEWLINK) continue;

            const auto* info = static_cast<const ifinfomsg*>(NLMSG_DATA(header));
            if (info->ifi_index != static_cast<int>(ifindex)) continue;

            result.up = (info->ifi_flags & IFF_UP) != 0;
            result.running = (info->ifi_flags & IFF_RUNNING) != 0;

            int attr_len = IFLA_PAYLOAD(header);
            for (rtattr* attr = IFLA_RTA(const_cast<ifinfomsg*>(info)); RTA_OK(attr, attr_len);
                 attr = RTA_NEXT(attr, attr_len)) {
                if (attr->rta_type == IFLA_MTU && RTA_PAYLOAD(attr) >= sizeof(uint32_t)) {
                    result.mtu = *static_cast<const uint32_t*>(RTA_DATA(attr));
                } else if (attr->rta_type == IFLA_LINKINFO) {
                    parse_link_info(attr, result);
                }
            }
            return result;
        }
    }
}

}  // namespace

CANHelper::CANHelper(std::string interface) : interface_(std::move(interface)) {
    if (interface_.empty()) throw CANHelperException("CAN interface name must not be empty");
    if (interface_.size() >= IFNAMSIZ) throw CANHelperException("CAN interface name is too long");
}

bool CANHelper::exists() const noexcept { return ::if_nametoindex(interface_.c_str()) != 0; }

CANInterfaceStatus CANHelper::status() const { return query_status(interface_); }

bool CANHelper::can_configure_without_sudo() const noexcept { return has_cap_net_admin(); }

void CANHelper::set_up() const {
    const auto current = status();
    if (!current.exists) throw CANHelperException("CAN interface does not exist: " + interface_);
    if (!current.is_can) throw CANHelperException("network interface is not a CAN device: " + interface_);
    run_ip_mutation({"link", "set", "dev", interface_, "up"});
}

void CANHelper::set_down() const {
    const auto current = status();
    if (!current.exists) throw CANHelperException("CAN interface does not exist: " + interface_);
    if (!current.is_can) throw CANHelperException("network interface is not a CAN device: " + interface_);
    run_ip_mutation({"link", "set", "dev", interface_, "down"});
}

void CANHelper::configure(const CANInterfaceConfig& config) const {
    validate_config(config);
    const CANInterfaceStatus current = status();
    if (!current.exists) throw CANHelperException("CAN interface does not exist: " + interface_);
    if (!current.is_can) throw CANHelperException("network interface is not a CAN device: " + interface_);

    set_down();

    std::vector<std::string> args = {"link", "set", "dev", interface_, "type", "can", "bitrate",
                                     std::to_string(config.bitrate)};
    if (config.sample_point > 0.0) {
        args.insert(args.end(), {"sample-point", format_ratio(config.sample_point)});
    }
    args.insert(args.end(), {"restart-ms", std::to_string(config.restart_ms)});

    if (config.fd_enabled) {
        args.insert(args.end(), {"dbitrate", std::to_string(config.dbitrate), "fd", "on"});
        if (config.dsample_point > 0.0) {
            args.insert(args.end(), {"dsample-point", format_ratio(config.dsample_point)});
        }
        if (config.dsjw > 0) args.insert(args.end(), {"dsjw", std::to_string(config.dsjw)});
    } else {
        args.insert(args.end(), {"fd", "off"});
    }

    run_ip_mutation(args);
    if (config.bring_up) set_up();
}

}  // namespace damiao_can::canbus
