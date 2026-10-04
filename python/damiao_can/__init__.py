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
DamiaoCAN CAN Python bindings for motor control via SocketCAN.

This package provides Python bindings for the DamiaoCAN motor control system,
allowing you to control DAMIAO motors through SocketCAN.
"""

from .damiao_can import *

__version__ = "1.2.9"
__author__ = "Enactic, Inc."

# Direct export of C++ classes - no wrappers
__all__ = [
    # Passive System Identification telemetry v1
    "SysIdScanConfig", "SysIdScanRecord", "decode_sysid_scan_record",
    "SysIdSafetyInformation", "decode_sysid_safety", "decode_sysid_live",
    "SysIdOperation",
    "SysIdResult",
    "SysIdFrame",
    "SysIdAck",
    "SysIdInformation",
    "SysIdSample",
    "SysIdMeasurement",
    "SysIdReply",
    "SysIdStartResult",
    "SysIdDiagnostics",
    "SysIdStopResult",
    "SysIdProtocolError",
    "SysIdTimeoutError",
    "SysIdSessionError",
    "SystemIdentification",
    "decode_sysid_ack",
    "decode_sysid_information",
    "decode_sysid_sample",
    "encode_sysid_request",
    "sysid_command_sequence_distance",

    # Enums
    "MotorType",
    "MotorIdentityConfidence",
    "MotorVariable",
    "CallbackMode",
    "ControlMode",

    # Data structures
    "LimitParam",
    "MotorIdentityRegisters",
    "MotorIdentityResult",
    "ParamResult",
    "MotorStateResult",
    "CANPacket",
    "CanFrame",
    "CanFdFrame",
    "MITParam",
    "PosVelParam",
    "VelParam",
    "PosForceParam",

    # Helpers
    "CanPacketEncoder",
    "CanPacketDecoder",
    "CANHelper",
    "CANInterfaceStatus",
    "CANInterfaceConfig",

    # Main C++ classes
    "Motor",
    "CANSocket",
    "CANDevice",
    "MotorDeviceCan",
    "CANDeviceCollection",
    "DMDeviceCollection",
    "MotorComponent",
    "DamiaoCAN",
    "DamiaoCANGroup",
    "DamiaoCANRecvResult",
    "DamiaoCANGroupRecvResult",

    # Exceptions
    "MotorLimitResolutionError",
    "CANSocketException",
    "CANHelperException",
]
