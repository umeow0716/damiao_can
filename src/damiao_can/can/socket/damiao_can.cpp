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

#include <linux/can.h>
#include <linux/can/raw.h>

#include <algorithm>
#include <chrono>
#include <damiao_can/can/socket/damiao_can.hpp>
#include <set>

namespace damiao_can::can::socket {

DamiaoCAN::DamiaoCAN(const std::string& can_interface, bool enable_fd)
    : can_interface_(can_interface), enable_fd_(enable_fd), can_helper_(can_interface) {
    can_socket_ = std::make_unique<canbus::CANSocket>(can_interface_, enable_fd_);
    master_can_device_collection_ = std::make_unique<canbus::CANDeviceCollection>(*can_socket_);
    motor_collection_ = std::make_unique<MotorComponent>(*can_socket_);
}

void DamiaoCAN::init_motors(const std::vector<damiao_motor::MotorType>& motor_types,
                            const std::vector<uint32_t>& send_can_ids,
                            const std::vector<uint32_t>& recv_can_ids,
                            const std::vector<damiao_motor::ControlMode>& control_modes) {
    if (motor_types.size() != send_can_ids.size() || motor_types.size() != recv_can_ids.size()) {
        throw std::invalid_argument(
            "Motor types, send CAN IDs, and receive CAN IDs vectors must have the same size, "
            "currently: " +
            std::to_string(motor_types.size()) + ", " + std::to_string(send_can_ids.size()) + ", " +
            std::to_string(recv_can_ids.size()));
    }

    motor_collection_->init_motor_devices(motor_types, send_can_ids, recv_can_ids, enable_fd_,
                                          control_modes);
    register_motor_collection();
}

void DamiaoCAN::register_motor_collection() {
    for (const auto& [id, device] : motor_collection_->get_device_collection().get_devices()) {
        master_can_device_collection_->add_device(device);
    }
}

std::vector<damiao_motor::Motor> DamiaoCAN::get_motors() const {
    return motor_collection_->get_motors();
}

damiao_motor::Motor DamiaoCAN::get_motor(int i) const { return motor_collection_->get_motor(i); }

void DamiaoCAN::enable_all() { motor_collection_->enable_all(); }
void DamiaoCAN::disable_all() { motor_collection_->disable_all(); }
void DamiaoCAN::set_zero(int i) { motor_collection_->set_zero(i); }
void DamiaoCAN::set_zero_all() { motor_collection_->set_zero_all(); }
void DamiaoCAN::refresh_one(int i) { motor_collection_->refresh_one(i); }
void DamiaoCAN::refresh_all() { motor_collection_->refresh_all(); }
void DamiaoCAN::query_param_one(int i, int RID) { motor_collection_->query_param_one(i, RID); }
void DamiaoCAN::query_param_all(int RID) { motor_collection_->query_param_all(RID); }
void DamiaoCAN::set_callback_mode_all(damiao_motor::CallbackMode callback_mode) {
    motor_collection_->set_callback_mode_all(callback_mode);
}
void DamiaoCAN::set_control_mode_one(int i, damiao_motor::ControlMode mode) {
    motor_collection_->set_control_mode_one(i, mode);
}
void DamiaoCAN::set_control_mode_all(damiao_motor::ControlMode mode) {
    motor_collection_->set_control_mode_all(mode);
}
void DamiaoCAN::mit_control_one(int i, const damiao_motor::MITParam& mit_param) {
    motor_collection_->mit_control_one(i, mit_param);
}
void DamiaoCAN::mit_control_all(const std::vector<damiao_motor::MITParam>& mit_params) {
    motor_collection_->mit_control_all(mit_params);
}
void DamiaoCAN::posvel_control_one(int i, const damiao_motor::PosVelParam& posvel_param) {
    motor_collection_->posvel_control_one(i, posvel_param);
}
void DamiaoCAN::posvel_control_all(const std::vector<damiao_motor::PosVelParam>& posvel_params) {
    motor_collection_->posvel_control_all(posvel_params);
}
void DamiaoCAN::vel_control_one(int i, const damiao_motor::VelParam& vel_param) {
    motor_collection_->vel_control_one(i, vel_param);
}
void DamiaoCAN::vel_control_all(const std::vector<damiao_motor::VelParam>& vel_params) {
    motor_collection_->vel_control_all(vel_params);
}
void DamiaoCAN::posforce_control_one(int i, const damiao_motor::PosForceParam& posforce_param) {
    motor_collection_->posforce_control_one(i, posforce_param);
}
void DamiaoCAN::posforce_control_all(
    const std::vector<damiao_motor::PosForceParam>& posforce_params) {
    motor_collection_->posforce_control_all(posforce_params);
}

int DamiaoCAN::recv_all(int timeout_us) {
    using clock = std::chrono::steady_clock;
    using microseconds = std::chrono::microseconds;

    if (timeout_us < 0) {
        throw std::invalid_argument("timeout_us must be non-negative");
    }

    const auto& devices = master_can_device_collection_->get_devices();
    if (devices.empty()) {
        return 0;
    }

    std::set<canid_t> responded_ids;
    const auto deadline = clock::now() + microseconds(timeout_us);

    auto remaining_timeout_us = [&]() -> int {
        const auto now = clock::now();
        if (now >= deadline) {
            return 0;
        }
        return static_cast<int>(std::chrono::duration_cast<microseconds>(deadline - now).count());
    };

    while (responded_ids.size() < devices.size()) {
        const int remaining = remaining_timeout_us();
        if (remaining <= 0 || !can_socket_->is_data_available(remaining)) {
            break;
        }

        canid_t response_id = 0;
        bool read_ok = false;

        if (enable_fd_) {
            canfd_frame frame;
            read_ok = can_socket_->read_canfd_frame(frame);
            if (read_ok) {
                response_id = frame.can_id & CAN_SFF_MASK;
                master_can_device_collection_->dispatch_frame_callback(frame);
            }
        } else {
            can_frame frame;
            read_ok = can_socket_->read_can_frame(frame);
            if (read_ok) {
                response_id = frame.can_id & CAN_SFF_MASK;
                master_can_device_collection_->dispatch_frame_callback(frame);
            }
        }

        if (read_ok && devices.find(response_id) != devices.end()) {
            responded_ids.insert(response_id);
        }
    }

    return static_cast<int>(responded_ids.size());
}

int DamiaoCAN::flush_rx() {
    int flushed = 0;

    if (enable_fd_) {
        canfd_frame frame;
        while (can_socket_->is_data_available(0) && can_socket_->read_canfd_frame(frame)) {
            flushed++;
        }
    } else {
        can_frame frame;
        while (can_socket_->is_data_available(0) && can_socket_->read_can_frame(frame)) {
            flushed++;
        }
    }

    return flushed;
}

int DamiaoCAN::expected_response_count() const {
    return static_cast<int>(master_can_device_collection_->get_devices().size());
}

}  // namespace damiao_can::can::socket
