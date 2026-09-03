# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Example: refresh multiple CAN buses in parallel with DamiaoCANGroup.

This example creates one DamiaoCAN instance per CAN interface through DamiaoCANGroup,
initializes motors on each bus, then uses flush_rx() + refresh_all() and calls
group.recv_all() to receive all buses in parallel. Each CAN interface keeps its
own receive worker thread.

Edit BUS_CONFIGS for your own CAN interfaces and motor IDs before running.
"""

import time

import damiao_can as dc


# Edit this table for your setup.
#
# send_ids are motor command CAN IDs.
# recv_ids are motor feedback CAN IDs.
# motor_types is omitted so each motor uses register-based AUTO initialization.
BUS_CONFIGS = {
    "can0": {
        "send_ids": [0x01],
        "recv_ids": [0x11],
        "control_modes": [dc.ControlMode.MIT],
    },
    "can1": {
        "send_ids": [0x02],
        "recv_ids": [0x12],
        "control_modes": [dc.ControlMode.MIT],
    },
}


def main() -> None:
    can_interfaces = list(BUS_CONFIGS)

    # Configure every CAN interface before DamiaoCANGroup creates its sockets.
    for can_interface in can_interfaces:
        helper = dc.CANHelper(can_interface)
        helper.set_down()
        helper.set_bitrate(1_000_000, 5_000_000, True)
        helper.set_up()

    # True means CAN-FD enabled. The group keeps one RX worker thread per interface.
    group = dc.DamiaoCANGroup(can_interfaces, True)

    for can_interface, config in BUS_CONFIGS.items():
        device = group.get_device(can_interface)

        device.init_motors(
            config["send_ids"],
            config["recv_ids"],
            control_modes=config["control_modes"],
        )

        device.set_callback_mode_all(dc.CallbackMode.STATE)

        print(
            f"{can_interface}: expected responses = "
            f"{device.expected_response_count()}"
        )

    try:
        group.enable_all()

        for step in range(30):
            # These two group helpers intentionally run sequentially across CAN
            # interfaces. Only recv_all() uses the per-interface RX worker threads.
            group.flush_rx()
            group.refresh_all()
            results = group.recv_all(timeout_us=500)

            print(f"\nstep {step}")
            print(results)

            # get() accepts an index or CAN interface name. If both are
            # supplied, can_id takes priority.
            first_result = results.get(index=0)
            same_result = results.get(
                index=999, can_id=first_result.can_interface)
            assert same_result.can_interface == first_result.can_interface

            for can_interface in can_interfaces:
                device = group.get_device(can_interface)

                for i, motor in enumerate(device.get_motors()):
                    print(
                        f"  {can_interface} motor[{i}] "
                        f"q={motor.get_position(): .6f} rad "
                        f"dq={motor.get_velocity(): .6f} rad/s "
                        f"tau={motor.get_torque(): .6f} Nm"
                    )

            time.sleep(0.1)

    finally:
        group.disable_all()


if __name__ == "__main__":
    main()
