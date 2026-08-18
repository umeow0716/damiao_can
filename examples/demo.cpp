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

#include <atomic>
#include <chrono>
#include <csignal>
#include <damiao_can/can/socket/damiao_can.hpp>
#include <damiao_can/damiao_motor/dm_motor_constants.hpp>
#include <iostream>
#include <thread>

int main() {
    try {
        std::cout << "=== DamiaoCAN CAN Example ===" << std::endl;
        std::cout << "This example demonstrates the DamiaoCAN API functionality" << std::endl;

        // Initialize DamiaoCAN with CAN interface and enable CAN-FD
        std::cout << "Initializing DamiaoCAN CAN..." << std::endl;
        damiao_can::can::socket::DamiaoCAN damiao_can("can0",
                                                      true);  // Use CAN-FD on can0 interface

        // Initialize motors
        std::vector<damiao_can::damiao_motor::MotorType> motor_types = {
            damiao_can::damiao_motor::MotorType::DM4310,
            damiao_can::damiao_motor::MotorType::DM4310};
        std::vector<uint32_t> send_can_ids = {0x01, 0x02};
        std::vector<uint32_t> recv_can_ids = {0x11, 0x12};
        damiao_can.init_motors(motor_types, send_can_ids, recv_can_ids);

        // Set callback mode to ignore and enable all motors
        damiao_can.set_callback_mode_all(damiao_can::damiao_motor::CallbackMode::IGNORE);

        // Enable all motors
        std::cout << "\n=== Enabling Motors ===" << std::endl;
        damiao_can.enable_all();
        // Allow time (2ms) for the motors to respond for slow operations like enabling
        damiao_can.recv_all(2000);

        // Set device mode to param and query motor id
        std::cout << "\n=== Querying Motor Recv IDs ===" << std::endl;
        damiao_can.set_callback_mode_all(damiao_can::damiao_motor::CallbackMode::PARAM);
        damiao_can.query_param_all(static_cast<int>(damiao_can::damiao_motor::RID::MST_ID));
        // Allow time (2ms) for the motors to respond for slow operations like querying
        // parameter from register
        damiao_can.recv_all(2000);

        // Access motors through components
        for (const auto& motor : damiao_can.get_motors()) {
            std::cout << "Motor: " << motor.get_send_can_id() << " ID: "
                      << motor.get_param(static_cast<int>(damiao_can::damiao_motor::RID::MST_ID))
                      << std::endl;
        }

        // Set device mode to state and control motor
        std::cout << "\n=== Controlling Motors ===" << std::endl;
        damiao_can.set_callback_mode_all(damiao_can::damiao_motor::CallbackMode::STATE);

        // Control motors with position control
        damiao_can.mit_control_all({damiao_can::damiao_motor::MITParam{2, 1, 0, 0, 0},
                                    damiao_can::damiao_motor::MITParam{2, 1, 0, 0, 0}});
        damiao_can.recv_all(500);

        // Control motors with torque control
        damiao_can.mit_control_all({damiao_can::damiao_motor::MITParam{0, 0, 0, 0, 0.1},
                                    damiao_can::damiao_motor::MITParam{0, 0, 0, 0, 0.1}});
        damiao_can.recv_all(500);

        for (int i = 0; i < 10; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            damiao_can.refresh_all();
            damiao_can.recv_all(300);

            // Display motor states
            for (const auto& motor : damiao_can.get_motors()) {
                std::cout << "Motor: " << motor.get_send_can_id()
                          << " position: " << motor.get_position() << std::endl;
            }
        }

        damiao_can.disable_all();
        damiao_can.recv_all(1000);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
