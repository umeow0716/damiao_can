# Copyright 2026 Enactic, Inc.
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

# Create DamiaoCAN instance
device = dc.DamiaoCAN("can0", True)

# Initialize device motors
motor_types = [dc.MotorType.DM4310]
send_ids = [0x0A]
recv_ids = [0x1A]
control_modes = [dc.ControlMode.POS_FORCE]
device.init_motors(motor_types, send_ids, recv_ids, control_modes)

# Enable motors
device.enable_all()
device.recv_all()

# PosForceParam(q, dq, i)
#   q   : target position (rad)
#   dq  : max velocity (rad/s)
#   i   : current limit (A)
device.set_callback_mode_all(dc.CallbackMode.STATE)
device.posforce_control_all([dc.PosForceParam(0.0, 0.0, 0.0)])
device.recv_all()

# Read motor position every 0.1s for 30 iterations
for _ in range(30):
    device.refresh_all()
    device.recv_all()
    for motor in device.get_motors():
        print(motor.get_position())
    time.sleep(0.1)

device.disable_all()
device.recv_all()
