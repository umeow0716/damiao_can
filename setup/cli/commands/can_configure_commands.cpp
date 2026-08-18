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
#include <iostream>
#include <string>
#include <vector>

#include "cli.hpp"

namespace damiao_can::cli {

namespace {
double parse_ratio(const std::string& value, const char* name) {
    try {
        const double parsed = std::stod(value);
        if (parsed < 0.0 || parsed >= 1.0) {
            throw std::out_of_range("ratio");
        }
        return parsed;
    } catch (...) {
        throw canbus::CANHelperException(std::string(name) +
                                         " must be 0 (automatic) or in the range (0, 1)");
    }
}

uint32_t parse_uint(const std::string& value, const char* name) {
    try {
        std::size_t consumed = 0;
        const unsigned long parsed = std::stoul(value, &consumed, 10);
        if (consumed != value.size() || parsed > UINT32_MAX) {
            throw std::out_of_range("integer");
        }
        return static_cast<uint32_t>(parsed);
    } catch (...) {
        throw canbus::CANHelperException(std::string(name) + " must be a non-negative integer");
    }
}
}  // namespace

int run_can_configure(const std::vector<std::string>& interfaces, int bitrate, int dbitrate,
                      bool fd_mode, const std::string& sample_point,
                      const std::string& dsample_point, const std::string& dsjw, int restart_ms) {
    std::vector<std::string> target_interfaces = interfaces;
    if (target_interfaces.empty()) target_interfaces = {"can0", "can1", "can2", "can3"};

    std::cout << "=========================================================\n";
    std::cout << " CAN CONFIGURE\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << " Target    :";
    for (const auto& i : target_interfaces) std::cout << " " << i;
    std::cout << "\n";
    std::cout << " Mode      : " << (fd_mode ? "CAN-FD" : "Classic CAN") << "\n";
    std::cout << " Bitrate   : " << bitrate << " bps  (SP: " << sample_point << ")\n";
    if (fd_mode) {
        std::cout << " Data rate : " << dbitrate << " bps  (DSP: " << dsample_point
                  << ", DSJW: " << dsjw << ")\n";
    }
    std::cout << " Restart   : " << restart_ms << " ms\n";
    std::cout << "=========================================================\n\n";

    if (bitrate <= 0) throw canbus::CANHelperException("bitrate must be greater than zero");
    if (fd_mode && dbitrate <= 0) {
        throw canbus::CANHelperException("dbitrate must be greater than zero in CAN-FD mode");
    }
    if (restart_ms < 0) throw canbus::CANHelperException("restart-ms must not be negative");

    canbus::CANInterfaceConfig config;
    config.bitrate = static_cast<uint32_t>(bitrate);
    config.dbitrate = static_cast<uint32_t>(dbitrate);
    config.fd_enabled = fd_mode;
    config.sample_point = parse_ratio(sample_point, "sample-point");
    config.restart_ms = static_cast<uint32_t>(restart_ms);
    if (fd_mode) {
        config.dsample_point = parse_ratio(dsample_point, "dsample-point");
        config.dsjw = parse_uint(dsjw, "dsjw");
    }

    int failed = 0;
    for (const auto& iface : target_interfaces) {
        std::cout << ">>> [" << iface << "] Applying..." << std::endl;
        try {
            canbus::CANHelper helper(iface);
            const auto before = helper.status();
            if (!before.exists) {
                std::cerr << "✗ [" << iface << "] Interface does not exist." << std::endl;
                ++failed;
                continue;
            }
            if (!before.is_can) {
                std::cerr << "✗ [" << iface << "] Interface is not a CAN device." << std::endl;
                ++failed;
                continue;
            }

            helper.configure(config);
            const auto after = helper.status();
            std::cout << "✓ [" << iface << "] " << (after.up ? "UP" : "configured") << ", MTU "
                      << after.mtu << "." << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "✗ [" << iface << "] " << e.what() << std::endl;
            ++failed;
        }
    }

    const int total = static_cast<int>(target_interfaces.size());
    std::cout << "\n---------------------------------------------------------\n";
    if (failed == 0) {
        std::cout << "✓ All " << total << " interface(s) configured successfully.\n";
    } else {
        std::cerr << "✗ " << failed << "/" << total << " interface(s) failed.\n";
        std::cout << "  " << (total - failed) << "/" << total
                  << " interface(s) configured successfully.\n";
    }
    std::cout << "---------------------------------------------------------\n";
    return failed > 0 ? 1 : 0;
}

}  // namespace damiao_can::cli
