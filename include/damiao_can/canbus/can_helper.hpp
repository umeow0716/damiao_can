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

#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace damiao_can::canbus {

struct CANInterfaceStatus {
    bool exists{false};
    bool is_can{false};
    bool up{false};
    bool running{false};
    int ifindex{0};
    uint32_t mtu{0};
    std::optional<uint32_t> bitrate;
    std::optional<uint32_t> dbitrate;
    bool fd_enabled{false};
    std::optional<uint32_t> restart_ms;
};

struct CANInterfaceConfig {
    uint32_t bitrate{1000000};
    uint32_t dbitrate{5000000};
    bool fd_enabled{false};
    double sample_point{0.0};
    double dsample_point{0.0};
    uint32_t dsjw{0};
    std::optional<uint32_t> restart_ms;
    bool bring_up{true};
};

class CANHelperException : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class CANHelper {
public:
    explicit CANHelper(std::string interface);

    const std::string& interface() const noexcept { return interface_; }

    // Read-only probes never elevate privileges.
    bool exists() const noexcept;
    CANInterfaceStatus status() const;
    bool can_configure_without_sudo() const noexcept;

    // Mutations use the current process privilege when CAP_NET_ADMIN is
    // available. Otherwise they request elevation through interactive sudo.
    void set_up() const;
    void set_down() const;
    void set_bitrate(uint32_t bitrate, uint32_t dbitrate, bool fd_enabled) const;
    void configure(const CANInterfaceConfig& config) const;

private:
    std::string interface_;
};

}  // namespace damiao_can::canbus
