// Copyright 2026 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "../../damiao_motor/dm_motor_constants.hpp"

namespace damiao_can::can::socket {

enum class MotorIdentityConfidence : uint8_t { UNKNOWN = 0, PROBABLE = 1, EXACT = 2 };

struct MotorIdentityRegisters {
    std::optional<uint32_t> hw_ver;
    std::optional<uint32_t> sw_ver;
    std::optional<uint32_t> sn;
    std::optional<uint32_t> npp;
    std::optional<uint32_t> sub_ver;

    std::optional<double> rs;
    std::optional<double> ls;
    std::optional<double> flux;
    std::optional<double> gr;
    std::optional<double> pmax;
    std::optional<double> vmax;
    std::optional<double> tmax;
};

struct MotorIdentityResult {
    uint32_t send_can_id = 0;
    bool responded = false;

    // Protocol scaling is authoritative when all three registers are readable.
    std::optional<damiao_motor::LimitParam> protocol_limits;
    std::string protocol_family = "UNKNOWN";

    // Physical model identification is informational only. P/non-P variants may share
    // exactly the same protocol limits and therefore do not need to be distinguished.
    std::optional<damiao_motor::MotorType> motor_type;
    MotorIdentityConfidence confidence = MotorIdentityConfidence::UNKNOWN;
    std::string model_name = "UNKNOWN";
    std::string reason;
    std::string hw_version_ascii;
    std::string sw_version_ascii;
    std::string serial_ascii;
    MotorIdentityRegisters registers;
};

std::string decode_u32_ascii(uint32_t value);
std::string motor_type_name(damiao_motor::MotorType motor_type);
MotorIdentityResult classify_motor_identity(uint32_t send_can_id,
                                             const MotorIdentityRegisters& registers);

}  // namespace damiao_can::can::socket
