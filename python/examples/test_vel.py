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
import damiao_can as dc
import time

# Configure SocketCAN before creating DamiaoCAN.
helper = dc.CANHelper("can0")
helper.set_down()
helper.set_bitrate(1_000_000, 5_000_000, True)
helper.set_up()

# Create the CAN socket only after the interface is ready.
device = dc.DamiaoCAN("can0", True)

# Initialize device motors. motor_types omitted => register-based AUTO.
send_ids = [0x0A]
recv_ids = [0x1A]
control_modes = [dc.ControlMode.VEL]
device.init_motors(send_ids, recv_ids, control_modes=control_modes)

# Enable motors
device.enable_all()
device.recv_all()

# VelParam(dq)
#   dq : target velocity (rad/s)
device.set_callback_mode_all(dc.CallbackMode.STATE)
device.vel_control_all([dc.VelParam(0.0)])
device.recv_all()

# Read motor position every 0.1s for 30 iterations
for _ in range(30):
    device.flush_rx()
    device.refresh_all()
    device.recv_all()
    for motor in device.get_motors():
        print(motor.get_position())
    time.sleep(0.1)

device.disable_all()
device.recv_all()
