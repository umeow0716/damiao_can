// Copyright 2026 Enactic, Inc.
// Licensed under the Apache License, Version 2.0

#include <algorithm>
#include <cctype>
#include <cmath>
#include <damiao_can/can/socket/motor_identity.hpp>
#include <damiao_can/damiao_motor/dm_motor_constants.hpp>
#include <sstream>
#include <vector>

namespace damiao_can::can::socket {
namespace {

using damiao_motor::LimitParam;
using damiao_motor::MotorType;

bool nearly_equal(double a, double b, double abs_tol = 1e-4, double rel_tol = 1e-4) {
    return std::abs(a - b) <= std::max(abs_tol, rel_tol * std::max(std::abs(a), std::abs(b)));
}

bool valid_limit(double value) { return std::isfinite(value) && value > 0.0; }

std::optional<LimitParam> limits_from_registers(const MotorIdentityRegisters& r) {
    if (!r.pmax || !r.vmax || !r.tmax || !valid_limit(*r.pmax) || !valid_limit(*r.vmax) ||
        !valid_limit(*r.tmax)) {
        return std::nullopt;
    }
    return LimitParam{*r.pmax, *r.vmax, *r.tmax};
}

std::vector<MotorType> types_from_limits(const LimitParam& registers) {
    std::vector<MotorType> matches;
    for (std::size_t i = 0; i < damiao_motor::MOTOR_LIMIT_PARAMS.size(); ++i) {
        const LimitParam& built_in = damiao_motor::MOTOR_LIMIT_PARAMS[i];
        if (nearly_equal(registers.pMax, built_in.pMax) &&
            nearly_equal(registers.vMax, built_in.vMax) &&
            nearly_equal(registers.tMax, built_in.tMax)) {
            matches.push_back(static_cast<MotorType>(i));
        }
    }
    return matches;
}

std::vector<MotorType> types_from_partial_limits(const MotorIdentityRegisters& registers) {
    const bool has_pmax = registers.pmax && valid_limit(*registers.pmax);
    const bool has_vmax = registers.vmax && valid_limit(*registers.vmax);
    const bool has_tmax = registers.tmax && valid_limit(*registers.tmax);
    if (!has_pmax && !has_vmax && !has_tmax) {
        return {};
    }

    std::vector<MotorType> matches;
    for (std::size_t i = 0; i < damiao_motor::MOTOR_LIMIT_PARAMS.size(); ++i) {
        const LimitParam& built_in = damiao_motor::MOTOR_LIMIT_PARAMS[i];
        if (has_pmax && !nearly_equal(*registers.pmax, built_in.pMax)) continue;
        if (has_vmax && !nearly_equal(*registers.vmax, built_in.vMax)) continue;
        if (has_tmax && !nearly_equal(*registers.tmax, built_in.tMax)) continue;
        matches.push_back(static_cast<MotorType>(i));
    }
    return matches;
}

std::optional<MotorType> canonical_type_from_matches(const std::vector<MotorType>& matches) {
    if (matches.size() == 1) {
        return matches.front();
    }

    const auto contains = [&](MotorType type) {
        return std::find(matches.begin(), matches.end(), type) != matches.end();
    };

    // DM4340 and DM4340_48V use the same protocol limits.  For protocol-facing
    // APIs, DM4340 is the canonical family label; P/non-P variants share it too.
    if (matches.size() == 2 && contains(MotorType::DM4340) && contains(MotorType::DM4340_48V)) {
        return MotorType::DM4340;
    }

    return std::nullopt;
}

std::string protocol_family_name(const LimitParam& limits) {
    // Family labels are informational. The register values themselves remain authoritative.
    const auto matches = types_from_limits(limits);
    if (matches.empty()) {
        return "REGISTER_DEFINED";
    }

    const auto contains = [&](MotorType type) {
        return std::find(matches.begin(), matches.end(), type) != matches.end();
    };
    if (contains(MotorType::DM4310)) {
        return "DM4310/DM4310P family";
    }
    if (contains(MotorType::DM4340) || contains(MotorType::DM4340_48V)) {
        return "DM4340/DM4340P family";
    }
    if (contains(MotorType::DM8009)) {
        return "DM8009/DM8009P family";
    }

    if (matches.size() == 1) {
        return motor_type_name(matches.front());
    }

    std::ostringstream os;
    for (std::size_t i = 0; i < matches.size(); ++i) {
        if (i != 0) os << '/';
        os << motor_type_name(matches[i]);
    }
    return os.str();
}

bool has_any_register(const MotorIdentityRegisters& r) {
    return r.hw_ver || r.sw_ver || r.sn || r.npp || r.sub_ver || r.rs || r.ls || r.flux || r.gr ||
           r.pmax || r.vmax || r.tmax;
}

}  // namespace

std::string decode_u32_ascii(uint32_t value) {
    std::string result;
    result.reserve(4);
    for (int shift = 0; shift < 32; shift += 8) {
        const unsigned char ch = static_cast<unsigned char>((value >> shift) & 0xFFu);
        if (ch == 0) break;
        if (!std::isprint(ch)) return {};
        result.push_back(static_cast<char>(ch));
    }
    return result;
}

std::string motor_type_name(MotorType motor_type) {
    switch (motor_type) {
        case MotorType::DM3507:
            return "DM3507";
        case MotorType::DM4310:
            return "DM4310";
        case MotorType::DM4310_48V:
            return "DM4310_48V";
        case MotorType::DM4340:
            return "DM4340";
        case MotorType::DM4340_48V:
            return "DM4340_48V";
        case MotorType::DM6006:
            return "DM6006";
        case MotorType::DM8006:
            return "DM8006";
        case MotorType::DM8009:
            return "DM8009";
        case MotorType::DM10010L:
            return "DM10010L";
        case MotorType::DM10010:
            return "DM10010";
        case MotorType::DMH3510:
            return "DMH3510";
        case MotorType::DMH6215:
            return "DMH6215";
        case MotorType::DMG6220:
            return "DMG6220";
        case MotorType::COUNT:
        case MotorType::UNKNOWN:
            break;
    }
    return "UNKNOWN";
}

MotorIdentityResult classify_motor_identity(uint32_t send_can_id,
                                            const MotorIdentityRegisters& registers) {
    MotorIdentityResult result;
    result.send_can_id = send_can_id;
    result.registers = registers;
    result.responded = has_any_register(registers);

    if (registers.hw_ver) result.hw_version_ascii = decode_u32_ascii(*registers.hw_ver);
    if (registers.sw_ver) result.sw_version_ascii = decode_u32_ascii(*registers.sw_ver);
    if (registers.sn) result.serial_ascii = decode_u32_ascii(*registers.sn);

    if (!result.responded) {
        result.reason = "no readable identity registers";
        return result;
    }

    result.protocol_limits = limits_from_registers(registers);
    if (result.protocol_limits) {
        result.protocol_family = protocol_family_name(*result.protocol_limits);
        const auto matches = types_from_limits(*result.protocol_limits);
        const auto canonical_type = canonical_type_from_matches(matches);

        if (canonical_type) {
            result.motor_type = *canonical_type;
            result.model_name = motor_type_name(*canonical_type);
            result.confidence = MotorIdentityConfidence::PROBABLE;
            result.reason = matches.size() == 1
                                ? "PMAX/VMAX/TMAX read from motor registers; unique built-in "
                                  "protocol-limit match"
                                : "PMAX/VMAX/TMAX read from motor registers; matched a protocol "
                                  "family with shared limits";
        } else if (matches.size() > 1) {
            result.model_name = "UNKNOWN";
            result.reason =
                "PMAX/VMAX/TMAX are authoritative for protocol scaling; no unique "
                "protocol-family label is available because multiple models share these limits";
        } else {
            result.model_name = "UNKNOWN";
            result.reason =
                "PMAX/VMAX/TMAX are authoritative for protocol scaling; no built-in model label "
                "matches";
        }
        return result;
    }

    // Full register-defined scaling is unavailable.  Partial limits may still identify one
    // built-in protocol family, which allows AUTO initialization to use that family's table.
    const auto partial_matches = types_from_partial_limits(registers);
    const auto canonical_type = canonical_type_from_matches(partial_matches);
    if (canonical_type) {
        result.motor_type = *canonical_type;
        result.model_name = motor_type_name(*canonical_type);
        result.protocol_family = protocol_family_name(
            damiao_motor::MOTOR_LIMIT_PARAMS[static_cast<std::size_t>(*canonical_type)]);
        result.confidence = MotorIdentityConfidence::PROBABLE;
        result.reason =
            "PMAX/VMAX/TMAX are incomplete or invalid; remaining readable limit registers "
            "uniquely identify a built-in protocol family for fallback scaling";
    } else {
        result.reason =
            "PMAX/VMAX/TMAX are incomplete or invalid and no unique built-in protocol-family "
            "fallback can be identified";
    }

    return result;
}

}  // namespace damiao_can::can::socket
