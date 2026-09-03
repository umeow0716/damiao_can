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

#pragma once

#include <cstdint>
#include <iosfwd>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "../../canbus/can_device_collection.hpp"
#include "../../canbus/can_socket.hpp"
#include "motor_component.hpp"
#include "motor_identity.hpp"

namespace damiao_can::can::socket {


struct MITExchangeSample {
    uint64_t tx_timestamp_ns = 0;
    uint64_t rx_timestamp_ns = 0;
    double command_tau = 0.0;
    double position = 0.0;
    double velocity = 0.0;
    double torque = 0.0;
    int t_mos = 0;
    int t_rotor = 0;
    bool valid = false;

    uint64_t round_trip_ns() const noexcept {
        return rx_timestamp_ns >= tx_timestamp_ns ? rx_timestamp_ns - tx_timestamp_ns : 0;
    }
};

struct PosVelExchangeSample {
    uint64_t tx_timestamp_ns = 0;
    uint64_t rx_timestamp_ns = 0;
    double command_position = 0.0;
    double command_velocity_limit = 0.0;
    double position = 0.0;
    double velocity = 0.0;
    double torque = 0.0;
    int t_mos = 0;
    int t_rotor = 0;
    bool valid = false;

    uint64_t round_trip_ns() const noexcept {
        return rx_timestamp_ns >= tx_timestamp_ns ? rx_timestamp_ns - tx_timestamp_ns : 0;
    }
};

struct DamiaoCANRecvResult {
    std::string can_interface;
    int expect = 0;
    int received = 0;
    bool ok = false;
    std::vector<uint32_t> missing;

    std::string to_string() const;
};

std::ostream& operator<<(std::ostream& os, const DamiaoCANRecvResult& result);

class MotorLimitResolutionError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class DamiaoCAN {
public:
    DamiaoCAN(const std::string& can_interface, bool enable_fd = false);
    ~DamiaoCAN() = default;

    std::string can_interface() const noexcept { return can_interface_; }
    bool can_fd_enabled() const noexcept { return enable_fd_; }

    void init_motors(const std::vector<uint32_t>& send_can_ids,
                     const std::vector<uint32_t>& recv_can_ids,
                     const std::vector<damiao_motor::MotorType>& motor_types,
                     const std::vector<damiao_motor::ControlMode>& control_modes = {});
    void init_motors(
        const std::vector<uint32_t>& send_can_ids,
        const std::vector<uint32_t>& recv_can_ids,
        const std::vector<std::optional<damiao_motor::MotorType>>& motor_types = {},
        const std::vector<damiao_motor::ControlMode>& control_modes = {});
    void init_motors_with_limits(
        const std::vector<damiao_motor::LimitParam>& limit_params,
        const std::vector<uint32_t>& send_can_ids, const std::vector<uint32_t>& recv_can_ids,
        const std::vector<damiao_motor::ControlMode>& control_modes = {});
    void set_motor_limits_one(int i, const damiao_motor::LimitParam& limit_param);

    std::vector<damiao_motor::Motor> get_motors() const;
    damiao_motor::Motor get_motor(int i) const;
    canbus::CANDeviceCollection& get_master_can_device_collection() {
        return *master_can_device_collection_;
    }

    void enable_all();
    void disable_all();
    void set_zero(int i);
    void set_zero_all();
    void refresh_one(int i);
    void refresh_all();
    void query_param_one(int i, int RID);
    void query_param_all(int RID);
    MotorIdentityResult probe_motor_identity(uint32_t send_can_id, int timeout_us = 5000);
    void set_callback_mode_all(damiao_motor::CallbackMode callback_mode);
    void set_control_mode_one(int i, damiao_motor::ControlMode mode);
    void set_control_mode_all(damiao_motor::ControlMode mode);
    void mit_control_one(int i, const damiao_motor::MITParam& mit_param);
    void mit_control_all(const std::vector<damiao_motor::MITParam>& mit_params);
    MITExchangeSample exchange_mit(int i, const damiao_motor::MITParam& mit_param,
                                   int timeout_us = 1000);
    PosVelExchangeSample exchange_posvel(int i, const damiao_motor::PosVelParam& posvel_param,
                                         int timeout_us = 500);
    void posvel_control_one(int i, const damiao_motor::PosVelParam& posvel_param);
    void posvel_control_all(const std::vector<damiao_motor::PosVelParam>& posvel_params);
    void vel_control_one(int i, const damiao_motor::VelParam& vel_param);
    void vel_control_all(const std::vector<damiao_motor::VelParam>& vel_params);
    void posforce_control_one(int i, const damiao_motor::PosForceParam& posforce_param);
    void posforce_control_all(const std::vector<damiao_motor::PosForceParam>& posforce_params);

    DamiaoCANRecvResult recv_all(int timeout_us = 500);
    int flush_rx();
    int expected_response_count() const;

private:
    std::string can_interface_;
    bool enable_fd_;
    std::unique_ptr<canbus::CANSocket> can_socket_;
    std::unique_ptr<MotorComponent> motor_collection_;
    std::unique_ptr<canbus::CANDeviceCollection> master_can_device_collection_;

    void register_motor_collection();
    std::optional<double> probe_param(uint32_t send_can_id, int rid, int timeout_us);
};

}  // namespace damiao_can::can::socket
