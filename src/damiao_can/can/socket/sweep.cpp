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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <damiao_can/can/socket/sweep.hpp>
#include <limits>
#include <stdexcept>
#include <thread>

namespace damiao_can::can::socket {
namespace {

using SweepClock = std::chrono::steady_clock;
constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr std::size_t kMaxSweepSamples = 10'000'000;
constexpr double kPositionVelocityHeadroom = 0.8;
constexpr double kMinPositionSamplesPerCycle = 20.0;

void send_zero_torque_best_effort(DamiaoCAN& device, int motor_index) noexcept {
    try {
        device.mit_control_one(motor_index, damiao_motor::MITParam{0.0, 0.0, 0.0, 0.0, 0.0});
    } catch (...) {
        // The sweep's original exception/result is more useful than a cleanup failure.
    }
}

void send_center_position_best_effort(DamiaoCAN& device, int motor_index,
                                      double center_position_rad,
                                      double velocity_limit_rad_s) noexcept {
    try {
        device.posvel_control_one(
            motor_index,
            damiao_motor::PosVelParam{center_position_rad, velocity_limit_rad_s});
    } catch (...) {
        // Preserve the sweep's original exception/result if cleanup fails.
    }
}

}  // namespace

void SweepRunner::validate_mit_torque_config(const MITTorqueSweepConfig& config) {
    if (!std::isfinite(config.start_hz) || config.start_hz <= 0.0) {
        throw std::invalid_argument("start_hz must be finite and greater than zero");
    }
    if (!std::isfinite(config.stop_hz) || config.stop_hz <= 0.0) {
        throw std::invalid_argument("stop_hz must be finite and greater than zero");
    }
    if (!std::isfinite(config.amplitude_nm) || config.amplitude_nm <= 0.0) {
        throw std::invalid_argument("amplitude_nm must be finite and greater than zero");
    }
    if (!std::isfinite(config.bias_nm)) {
        throw std::invalid_argument("bias_nm must be finite");
    }
    if (!std::isfinite(config.sample_rate_hz) || config.sample_rate_hz <= 0.0) {
        throw std::invalid_argument("sample_rate_hz must be finite and greater than zero");
    }
    if (!std::isfinite(config.duration_s) || config.duration_s <= 0.0) {
        throw std::invalid_argument("duration_s must be finite and greater than zero");
    }
    if (config.response_timeout_us < 0) {
        throw std::invalid_argument("response_timeout_us must be non-negative");
    }

    const double max_frequency = std::max(config.start_hz, config.stop_hz);
    if (config.sample_rate_hz <= 2.0 * max_frequency) {
        throw std::invalid_argument(
            "sample_rate_hz must be greater than twice the highest sweep frequency");
    }

    const double requested_samples = std::floor(config.duration_s * config.sample_rate_hz) + 1.0;
    if (!std::isfinite(requested_samples) || requested_samples < 1.0 ||
        requested_samples > static_cast<double>(kMaxSweepSamples)) {
        throw std::invalid_argument("sweep sample count is outside the supported range");
    }
}


void SweepRunner::validate_position_config(const PositionSweepConfig& config) {
    if (!std::isfinite(config.center_position_rad)) {
        throw std::invalid_argument("center_position_rad must be finite");
    }
    if (!std::isfinite(config.start_hz) || config.start_hz <= 0.0) {
        throw std::invalid_argument("start_hz must be finite and greater than zero");
    }
    if (!std::isfinite(config.stop_hz) || config.stop_hz <= config.start_hz) {
        throw std::invalid_argument("stop_hz must be finite and greater than start_hz");
    }
    if (!std::isfinite(config.amplitude_rad) || config.amplitude_rad <= 0.0) {
        throw std::invalid_argument("amplitude_rad must be finite and greater than zero");
    }
    if (!std::isfinite(config.velocity_limit_rad_s) || config.velocity_limit_rad_s <= 0.0) {
        throw std::invalid_argument(
            "velocity_limit_rad_s must be finite and greater than zero");
    }
    if (config.wait_us <= 0) {
        throw std::invalid_argument("wait_us must be greater than zero");
    }
    if (config.points < 2) {
        throw std::invalid_argument("points must be at least 2");
    }
    if (config.settling_cycles < 0) {
        throw std::invalid_argument("settling_cycles must be non-negative");
    }
    if (config.measure_cycles <= 0) {
        throw std::invalid_argument("measure_cycles must be greater than zero");
    }

    const double nominal_rate_hz = 1'000'000.0 / static_cast<double>(config.wait_us);
    if (nominal_rate_hz < kMinPositionSamplesPerCycle * config.stop_hz) {
        throw std::invalid_argument(
            "wait_us is too long for stop_hz; use at least 20 command slots per highest-frequency cycle");
    }

    const double ratio = config.stop_hz / config.start_hz;
    double requested_samples = 0.0;
    for (int frequency_index = 0; frequency_index < config.points; ++frequency_index) {
        const double fraction = static_cast<double>(frequency_index) /
                                static_cast<double>(config.points - 1);
        const double frequency_hz = config.start_hz * std::pow(ratio, fraction);
        const double point_duration_s =
            static_cast<double>(config.settling_cycles + config.measure_cycles) / frequency_hz;
        requested_samples += std::floor(point_duration_s * nominal_rate_hz) + 1.0;
    }
    if (!std::isfinite(requested_samples) || requested_samples < 1.0 ||
        requested_samples > static_cast<double>(kMaxSweepSamples)) {
        throw std::invalid_argument("position sweep sample count is outside the supported range");
    }
}

PositionSweepResult SweepRunner::run_position_sinestream(
    int motor_index, const PositionSweepConfig& config) {
    validate_position_config(config);

    const auto motor = device_.get_motor(motor_index);
    const auto limits = motor.get_limit_param();
    if (std::abs(config.center_position_rad) + config.amplitude_rad > limits.pMax) {
        throw std::invalid_argument(
            "center_position_rad +/- amplitude_rad exceeds the configured motor position limit");
    }
    if (config.velocity_limit_rad_s > limits.vMax) {
        throw std::invalid_argument(
            "velocity_limit_rad_s exceeds the configured motor velocity limit");
    }

    const double nominal_rate_hz = 1'000'000.0 / static_cast<double>(config.wait_us);
    const double dt_s = 1.0 / nominal_rate_hz;
    const double frequency_ratio = config.stop_hz / config.start_hz;

    PositionSweepResult result;
    result.center_position_rad = config.center_position_rad;

    // Reserve a conservative estimate to reduce reallocations without computing
    // a huge vector up front.
    const double longest_point_s =
        static_cast<double>(config.settling_cycles + config.measure_cycles) / config.start_hz;
    const std::size_t reserve_per_point =
        static_cast<std::size_t>(std::floor(longest_point_s * nominal_rate_hz)) + 1;
    result.samples.reserve(std::min<std::size_t>(
        kMaxSweepSamples, reserve_per_point * static_cast<std::size_t>(config.points)));

    device_.flush_rx();

    const auto sweep_start = SweepClock::now();
    const uint64_t sweep_start_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(sweep_start.time_since_epoch())
            .count());

    try {
        for (int frequency_index = 0; frequency_index < config.points; ++frequency_index) {
            const double fraction = static_cast<double>(frequency_index) /
                                    static_cast<double>(config.points - 1);
            const double frequency_hz =
                config.start_hz * std::pow(frequency_ratio, fraction);

            // POS_VEL's v_des is a speed ceiling. Scale the displacement at high
            // frequencies so the requested sine does not itself require more than
            // 80% of that ceiling: v_peak = 2*pi*f*A.
            const double velocity_limited_amplitude =
                kPositionVelocityHeadroom * config.velocity_limit_rad_s /
                (kTwoPi * frequency_hz);
            const double command_amplitude_rad =
                std::min(config.amplitude_rad, velocity_limited_amplitude);

            const double total_cycles =
                static_cast<double>(config.settling_cycles + config.measure_cycles);
            const double point_duration_s = total_cycles / frequency_hz;
            const std::size_t sample_count =
                static_cast<std::size_t>(std::floor(point_duration_s / dt_s)) + 1;
            const double measure_start_s =
                static_cast<double>(config.settling_cycles) / frequency_hz;
            const auto point_start = SweepClock::now();

            for (std::size_t i = 0; i < sample_count; ++i) {
                const double local_s = static_cast<double>(i) * dt_s;
                const auto target = point_start +
                    std::chrono::duration_cast<SweepClock::duration>(
                        std::chrono::duration<double>(local_s));
                std::this_thread::sleep_until(target);

                const double phase_rad = kTwoPi * frequency_hz * local_s;
                const double command_position_rad =
                    config.center_position_rad + command_amplitude_rad * std::sin(phase_rad);
                const bool measurement = local_s + (0.5 * dt_s) >= measure_start_s;

                const double actual_s =
                    std::chrono::duration<double>(SweepClock::now() - sweep_start).count();
                const damiao_motor::PosVelParam command{command_position_rad,
                                                        config.velocity_limit_rad_s};
                PosVelExchangeSample feedback =
                    device_.exchange_posvel(motor_index, command, config.wait_us);

                PositionSweepSample sample;
                sample.frequency_index = frequency_index;
                sample.scheduled_time_s =
                    std::chrono::duration<double>(target - sweep_start).count();
                sample.command_time_s = feedback.tx_timestamp_ns >= sweep_start_ns
                                            ? static_cast<double>(feedback.tx_timestamp_ns -
                                                                  sweep_start_ns) /
                                                  1'000'000'000.0
                                            : actual_s;
                sample.frequency_hz = frequency_hz;
                sample.phase_rad = phase_rad;
                sample.command_amplitude_rad = command_amplitude_rad;
                sample.command_position_rad = command_position_rad;
                sample.measurement = measurement;
                sample.feedback = feedback;
                result.samples.push_back(sample);

                if (feedback.valid) {
                    ++result.valid_samples;
                } else {
                    ++result.dropped_samples;
                }
            }
        }
    } catch (...) {
        send_center_position_best_effort(device_, motor_index, config.center_position_rad,
                                         config.velocity_limit_rad_s);
        throw;
    }

    send_center_position_best_effort(device_, motor_index, config.center_position_rad,
                                     config.velocity_limit_rad_s);
    result.elapsed_s = std::chrono::duration<double>(SweepClock::now() - sweep_start).count();
    return result;
}

MITTorqueSweepResult SweepRunner::run_mit_torque_chirp(
    int motor_index, const MITTorqueSweepConfig& config) {
    validate_mit_torque_config(config);

    // Validate the motor index before changing the bus state.
    const auto motor = device_.get_motor(motor_index);
    const auto limits = motor.get_limit_param();
    if (std::abs(config.bias_nm) + config.amplitude_nm > limits.tMax) {
        throw std::invalid_argument(
            "bias_nm + amplitude_nm exceeds the configured motor torque limit");
    }

    const std::size_t sample_count =
        static_cast<std::size_t>(std::floor(config.duration_s * config.sample_rate_hz)) + 1;
    const double sweep_rate_hz_per_s = (config.stop_hz - config.start_hz) / config.duration_s;

    MITTorqueSweepResult result;
    result.samples.reserve(sample_count);

    device_.flush_rx();

    const auto start = SweepClock::now();
    const uint64_t start_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(start.time_since_epoch()).count());

    try {
        for (std::size_t i = 0; i < sample_count; ++i) {
            const double scheduled_s = static_cast<double>(i) / config.sample_rate_hz;
            const auto target = start + std::chrono::duration_cast<SweepClock::duration>(
                                            std::chrono::duration<double>(scheduled_s));
            std::this_thread::sleep_until(target);

            const double actual_s = std::chrono::duration<double>(SweepClock::now() - start).count();
            // Generate the discrete chirp from the nominal schedule. command_time_s records
            // the actual send timing so downstream identification can account for jitter.
            const double phase =
                kTwoPi * (config.start_hz * scheduled_s +
                          0.5 * sweep_rate_hz_per_s * scheduled_s * scheduled_s);
            const double frequency_hz =
                config.start_hz + sweep_rate_hz_per_s * scheduled_s;
            const double command_tau = config.bias_nm + config.amplitude_nm * std::sin(phase);

            damiao_motor::MITParam command{0.0, 0.0, 0.0, 0.0, command_tau};
            MITExchangeSample feedback =
                device_.exchange_mit(motor_index, command, config.response_timeout_us);

            MITTorqueSweepSample sample;
            sample.scheduled_time_s = scheduled_s;
            sample.command_time_s = feedback.tx_timestamp_ns >= start_ns
                                        ? static_cast<double>(feedback.tx_timestamp_ns - start_ns) /
                                              1'000'000'000.0
                                        : actual_s;
            sample.frequency_hz = frequency_hz;
            sample.command_tau = command_tau;
            sample.feedback = feedback;
            result.samples.push_back(sample);

            if (feedback.valid) {
                ++result.valid_samples;
            } else {
                ++result.dropped_samples;
            }
        }
    } catch (...) {
        send_zero_torque_best_effort(device_, motor_index);
        throw;
    }

    send_zero_torque_best_effort(device_, motor_index);
    result.elapsed_s = std::chrono::duration<double>(SweepClock::now() - start).count();
    return result;
}

}  // namespace damiao_can::can::socket
