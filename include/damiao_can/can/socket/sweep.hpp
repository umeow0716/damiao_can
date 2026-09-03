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

#include <cstddef>
#include <vector>

#include "damiao_can.hpp"

namespace damiao_can::can::socket {

// Configuration for a linearly swept MIT feed-forward torque excitation.
// Position/velocity gains are intentionally fixed at zero so the excitation
// identifies the mechanical plant rather than an outer position/velocity loop.
struct MITTorqueSweepConfig {
    double start_hz = 1.0;
    double stop_hz = 100.0;
    double amplitude_nm = 0.1;
    double bias_nm = 0.0;
    double sample_rate_hz = 1000.0;
    double duration_s = 10.0;
    int response_timeout_us = 1000;
};

struct MITTorqueSweepSample {
    double scheduled_time_s = 0.0;
    double command_time_s = 0.0;
    double frequency_hz = 0.0;
    double command_tau = 0.0;
    MITExchangeSample feedback;
};

struct MITTorqueSweepResult {
    std::vector<MITTorqueSweepSample> samples;
    std::size_t valid_samples = 0;
    std::size_t dropped_samples = 0;
    double elapsed_s = 0.0;

    bool ok() const noexcept { return !samples.empty() && dropped_samples == 0; }
    double valid_ratio() const noexcept {
        if (samples.empty()) {
            return 0.0;
        }
        return static_cast<double>(valid_samples) / static_cast<double>(samples.size());
    }
};

// POS_VEL command-tracking bandwidth measurement using a logarithmically
// spaced stepped-sine (sinestream) excitation. Each frequency is held for
// settling_cycles + measure_cycles. Only the measurement cycles should be
// used for gain/phase estimation.
struct PositionSweepConfig {
    double center_position_rad = 0.0;
    double start_hz = 1.0;
    double stop_hz = 100.0;
    double amplitude_rad = 0.05;
    double velocity_limit_rad_s = 10.0;
    int wait_us = 500;
    int points = 20;
    int settling_cycles = 2;
    int measure_cycles = 3;
};

struct PositionSweepSample {
    int frequency_index = 0;
    double scheduled_time_s = 0.0;
    double command_time_s = 0.0;
    double frequency_hz = 0.0;
    double phase_rad = 0.0;
    double command_amplitude_rad = 0.0;
    double command_position_rad = 0.0;
    bool measurement = false;
    PosVelExchangeSample feedback;
};

struct PositionSweepResult {
    std::vector<PositionSweepSample> samples;
    std::size_t valid_samples = 0;
    std::size_t dropped_samples = 0;
    double center_position_rad = 0.0;
    double elapsed_s = 0.0;

    bool ok() const noexcept { return !samples.empty() && dropped_samples == 0; }
    double valid_ratio() const noexcept {
        if (samples.empty()) {
            return 0.0;
        }
        return static_cast<double>(valid_samples) / static_cast<double>(samples.size());
    }
};

class SweepRunner {
public:
    explicit SweepRunner(DamiaoCAN& device) : device_(device) {}

    MITTorqueSweepResult run_mit_torque_chirp(int motor_index,
                                               const MITTorqueSweepConfig& config);
    PositionSweepResult run_position_sinestream(int motor_index,
                                                 const PositionSweepConfig& config);

    static void validate_mit_torque_config(const MITTorqueSweepConfig& config);
    static void validate_position_config(const PositionSweepConfig& config);

private:
    DamiaoCAN& device_;
};

}  // namespace damiao_can::can::socket
