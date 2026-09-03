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
#include <cmath>
#include <cstring>
#include <chrono>
#include <damiao_can/can/socket/damiao_can.hpp>
#include <iomanip>
#include <limits>
#include <optional>
#include <ostream>
#include <set>
#include <sstream>

namespace damiao_can::can::socket {

DamiaoCAN::DamiaoCAN(const std::string& can_interface, bool enable_fd)
    : can_interface_(can_interface), enable_fd_(enable_fd) {
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

void DamiaoCAN::init_motors_with_limits(
    const std::vector<damiao_motor::LimitParam>& limit_params,
    const std::vector<uint32_t>& send_can_ids, const std::vector<uint32_t>& recv_can_ids,
    const std::vector<damiao_motor::ControlMode>& control_modes) {
    if (limit_params.size() != send_can_ids.size() || limit_params.size() != recv_can_ids.size()) {
        throw std::invalid_argument(
            "Limit parameters, send CAN IDs, and receive CAN IDs vectors must have the same size");
    }
    motor_collection_->init_motor_devices_with_limits(limit_params, send_can_ids, recv_can_ids,
                                                       enable_fd_, control_modes);
    register_motor_collection();
}

void DamiaoCAN::set_motor_limits_one(int i, const damiao_motor::LimitParam& limit_param) {
    motor_collection_->set_limit_param_one(i, limit_param);
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

std::optional<double> DamiaoCAN::probe_param(uint32_t send_can_id, int rid, int timeout_us) {
    using clock = std::chrono::steady_clock;
    using microseconds = std::chrono::microseconds;

    if (send_can_id > CAN_SFF_MASK) {
        throw std::invalid_argument("send_can_id must fit in an 11-bit CAN ID");
    }
    if (rid < 0 || rid > 0xFF) {
        throw std::invalid_argument("rid must fit in one byte");
    }
    if (timeout_us < 0) {
        throw std::invalid_argument("timeout_us must be non-negative");
    }

    // Parameter reads do not require a MotorType.  Flush before the request so a stale
    // response for the same RID cannot be mistaken for the current probe.
    flush_rx();

    const auto packet =
        damiao_motor::CanPacketEncoder::create_query_param_command(send_can_id, rid);

    if (enable_fd_) {
        canfd_frame frame{};
        frame.can_id = packet.send_can_id;
        frame.len = static_cast<__u8>(packet.data.size());
        frame.flags = CANFD_BRS;
        std::copy(packet.data.begin(), packet.data.end(), frame.data);
        can_socket_->write_canfd_frame(frame);
    } else {
        can_frame frame{};
        frame.can_id = packet.send_can_id;
        frame.can_dlc = static_cast<__u8>(packet.data.size());
        std::copy(packet.data.begin(), packet.data.end(), frame.data);
        can_socket_->write_can_frame(frame);
    }

    const auto deadline = clock::now() + microseconds(timeout_us);
    while (clock::now() < deadline) {
        const auto now = clock::now();
        const int remaining = static_cast<int>(
            std::chrono::duration_cast<microseconds>(deadline - now).count());
        if (remaining <= 0 || !can_socket_->is_data_available(remaining)) {
            break;
        }

        std::vector<uint8_t> data;
        bool read_ok = false;
        if (enable_fd_) {
            canfd_frame frame{};
            read_ok = can_socket_->read_canfd_frame(frame);
            if (read_ok) {
                data.assign(frame.data, frame.data + frame.len);
            }
        } else {
            can_frame frame{};
            read_ok = can_socket_->read_can_frame(frame);
            if (read_ok) {
                data.assign(frame.data, frame.data + frame.can_dlc);
            }
        }

        if (!read_ok || data.size() < 8) {
            continue;
        }

        const uint32_t response_slave_id =
            static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8);
        if (response_slave_id != send_can_id || data[2] != 0x33 || data[3] != rid) {
            continue;
        }

        const auto parsed = damiao_motor::CanPacketDecoder::parse_motor_param_data(data);
        if (parsed.valid && parsed.rid == rid) {
            return parsed.value;
        }
    }

    return std::nullopt;
}

MotorIdentityResult DamiaoCAN::probe_motor_identity(uint32_t send_can_id, int timeout_us) {
    MotorIdentityRegisters registers;

    auto read_u32 = [&](damiao_motor::RID rid) -> std::optional<uint32_t> {
        const auto value = probe_param(send_can_id, static_cast<int>(rid), timeout_us);
        if (!value.has_value() || !std::isfinite(*value) || *value < 0.0 ||
            *value > static_cast<double>(std::numeric_limits<uint32_t>::max())) {
            return std::nullopt;
        }
        return static_cast<uint32_t>(std::llround(*value));
    };
    auto read_float = [&](damiao_motor::RID rid) -> std::optional<double> {
        const auto value = probe_param(send_can_id, static_cast<int>(rid), timeout_us);
        if (!value.has_value() || !std::isfinite(*value)) {
            return std::nullopt;
        }
        return value;
    };

    registers.hw_ver = read_u32(damiao_motor::RID::hw_ver);
    registers.sw_ver = read_u32(damiao_motor::RID::sw_ver);
    registers.sn = read_u32(damiao_motor::RID::SN);
    registers.npp = read_u32(damiao_motor::RID::NPP);
    registers.sub_ver = read_u32(damiao_motor::RID::sub_ver);
    registers.rs = read_float(damiao_motor::RID::Rs);
    registers.ls = read_float(damiao_motor::RID::LS);
    registers.flux = read_float(damiao_motor::RID::Flux);
    registers.gr = read_float(damiao_motor::RID::Gr);
    registers.pmax = read_float(damiao_motor::RID::PMAX);
    registers.vmax = read_float(damiao_motor::RID::VMAX);
    registers.tmax = read_float(damiao_motor::RID::TMAX);

    return classify_motor_identity(send_can_id, registers);
}
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

MITExchangeSample DamiaoCAN::exchange_mit(int i, const damiao_motor::MITParam& mit_param,
                                          int timeout_us) {
    using clock = std::chrono::steady_clock;
    using microseconds = std::chrono::microseconds;

    if (timeout_us < 0) {
        throw std::invalid_argument("timeout_us must be non-negative");
    }

    const auto motor = motor_collection_->get_motor(i);
    const canid_t target_recv_id = static_cast<canid_t>(motor.get_recv_can_id());

    MITExchangeSample sample;
    sample.command_tau = mit_param.tau;
    sample.position = std::numeric_limits<double>::quiet_NaN();
    sample.velocity = std::numeric_limits<double>::quiet_NaN();
    sample.torque = std::numeric_limits<double>::quiet_NaN();

    motor_collection_->mit_control_one(i, mit_param);
    sample.tx_timestamp_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now().time_since_epoch())
            .count());

    const auto deadline = clock::now() + microseconds(timeout_us);
    while (true) {
        const auto now = clock::now();
        if (now >= deadline) {
            break;
        }

        const int remaining = static_cast<int>(
            std::chrono::duration_cast<microseconds>(deadline - now).count());
        if (remaining <= 0 || !can_socket_->is_data_available(remaining)) {
            break;
        }

        canid_t response_id = 0;
        std::vector<uint8_t> data;
        bool read_ok = false;

        if (enable_fd_) {
            canfd_frame frame{};
            read_ok = can_socket_->read_canfd_frame(frame);
            if (read_ok) {
                response_id = frame.can_id & CAN_SFF_MASK;
                data.assign(frame.data, frame.data + frame.len);
                master_can_device_collection_->dispatch_frame_callback(frame);
            }
        } else {
            can_frame frame{};
            read_ok = can_socket_->read_can_frame(frame);
            if (read_ok) {
                response_id = frame.can_id & CAN_SFF_MASK;
                data.assign(frame.data, frame.data + frame.can_dlc);
                master_can_device_collection_->dispatch_frame_callback(frame);
            }
        }

        if (!read_ok || response_id != target_recv_id) {
            continue;
        }

        const auto state = damiao_motor::CanPacketDecoder::parse_motor_state_data(motor, data);
        sample.rx_timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now().time_since_epoch())
                .count());
        if (state.valid) {
            sample.position = state.position;
            sample.velocity = state.velocity;
            sample.torque = state.torque;
            sample.t_mos = state.t_mos;
            sample.t_rotor = state.t_rotor;
            sample.valid = true;
        }
        break;
    }

    return sample;
}

PosVelExchangeSample DamiaoCAN::exchange_posvel(
    int i, const damiao_motor::PosVelParam& posvel_param, int timeout_us) {
    using clock = std::chrono::steady_clock;
    using microseconds = std::chrono::microseconds;

    if (timeout_us < 0) {
        throw std::invalid_argument("timeout_us must be non-negative");
    }

    const auto motor = motor_collection_->get_motor(i);
    const canid_t target_recv_id = static_cast<canid_t>(motor.get_recv_can_id());

    PosVelExchangeSample sample;
    sample.command_position = posvel_param.q;
    sample.command_velocity_limit = posvel_param.dq;
    sample.position = std::numeric_limits<double>::quiet_NaN();
    sample.velocity = std::numeric_limits<double>::quiet_NaN();
    sample.torque = std::numeric_limits<double>::quiet_NaN();

    motor_collection_->posvel_control_one(i, posvel_param);
    sample.tx_timestamp_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now().time_since_epoch())
            .count());

    const auto deadline = clock::now() + microseconds(timeout_us);
    while (true) {
        const auto now = clock::now();
        if (now >= deadline) {
            break;
        }

        const int remaining = static_cast<int>(
            std::chrono::duration_cast<microseconds>(deadline - now).count());
        if (remaining <= 0 || !can_socket_->is_data_available(remaining)) {
            break;
        }

        canid_t response_id = 0;
        std::vector<uint8_t> data;
        bool read_ok = false;

        if (enable_fd_) {
            canfd_frame frame{};
            read_ok = can_socket_->read_canfd_frame(frame);
            if (read_ok) {
                response_id = frame.can_id & CAN_SFF_MASK;
                data.assign(frame.data, frame.data + frame.len);
                master_can_device_collection_->dispatch_frame_callback(frame);
            }
        } else {
            can_frame frame{};
            read_ok = can_socket_->read_can_frame(frame);
            if (read_ok) {
                response_id = frame.can_id & CAN_SFF_MASK;
                data.assign(frame.data, frame.data + frame.can_dlc);
                master_can_device_collection_->dispatch_frame_callback(frame);
            }
        }

        if (!read_ok || response_id != target_recv_id) {
            continue;
        }

        const auto state = damiao_motor::CanPacketDecoder::parse_motor_state_data(motor, data);
        sample.rx_timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now().time_since_epoch())
                .count());
        if (state.valid) {
            sample.position = state.position;
            sample.velocity = state.velocity;
            sample.torque = state.torque;
            sample.t_mos = state.t_mos;
            sample.t_rotor = state.t_rotor;
            sample.valid = true;
        }
        break;
    }

    return sample;
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

std::string DamiaoCANRecvResult::to_string() const {
    std::ostringstream os;
    os << "{can_interface: \"" << can_interface << "\", expected: " << expect
       << ", received: " << received << ", missing: [";

    for (std::size_t i = 0; i < missing.size(); ++i) {
        if (i != 0) {
            os << ", ";
        }

        os << "0x" << std::hex << std::nouppercase << std::setw(2) << std::setfill('0')
           << missing[i] << std::dec << std::setfill(' ');
    }

    os << "], ok: " << (ok ? "True" : "False") << "}";
    return os.str();
}

std::ostream& operator<<(std::ostream& os, const DamiaoCANRecvResult& result) {
    return os << result.to_string();
}

DamiaoCANRecvResult DamiaoCAN::recv_all(int timeout_us) {
    using clock = std::chrono::steady_clock;
    using microseconds = std::chrono::microseconds;

    if (timeout_us < 0) {
        throw std::invalid_argument("timeout_us must be non-negative");
    }

    DamiaoCANRecvResult result;
    result.can_interface = can_interface_;

    const auto& devices = master_can_device_collection_->get_devices();
    result.expect = static_cast<int>(devices.size());

    if (devices.empty()) {
        result.ok = true;
        return result;
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

    result.received = static_cast<int>(responded_ids.size());

    for (const auto& [recv_id, device] : devices) {
        if (responded_ids.find(recv_id) == responded_ids.end()) {
            result.missing.push_back(static_cast<uint32_t>(device->get_send_can_id()));
        }
    }

    std::sort(result.missing.begin(), result.missing.end());
    result.ok = result.missing.empty();
    return result;
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
