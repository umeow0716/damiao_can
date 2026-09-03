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

void send_zero_torque_best_effort(DamiaoCAN& device, int motor_index) noexcept {
    try {
        device.mit_control_one(motor_index, damiao_motor::MITParam{0.0, 0.0, 0.0, 0.0, 0.0});
    } catch (...) {
        // The sweep's original exception/result is more useful than a cleanup failure.
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

MITTorqueSweepResult SweepRunner::run_mit_torque_chirp(
    int motor_index, const MITTorqueSweepConfig& config) {
    validate_mit_torque_config(config);

    // Validate the motor index before changing the bus state.
    const auto motor = device_.get_motor(motor_index);
    const auto limits = damiao_motor::Motor::get_limit_param(motor.get_motor_type());
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
