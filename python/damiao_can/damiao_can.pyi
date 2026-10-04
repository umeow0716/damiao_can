"""
DamiaoCAN CAN Python bindings for motor control via SocketCAN
"""
from __future__ import annotations
import collections.abc
import enum
import typing
__all__: list[str] = ['ACC', 'MotorComponent', 'CANDevice', 'CANDeviceCollection', 'CANPacket', 'CANSocket', 'CANSocketException', 'COUNT', 'UNKNOWN', 'CTRL_MODE', 'CallbackMode', 'CanFdFrame', 'CanFrame', 'CanPacketDecoder', 'CanPacketEncoder', 'ControlMode', 'DEC', 'DM10010', 'DM10010L', 'DM3507', 'DM4310', 'DM4310_48V', 'DM4340', 'DM4340_48V', 'DM6006', 'DM8006', 'DM8009', 'DMDeviceCollection', 'DMG6220', 'DMH3510', 'DMH6215', 'Damp', 'Deta', 'ESC_ID', 'Flux', 'GREF', 'Gr', 'IGNORE', 'IQ_c1', 'I_BW', 'Inertia', 'KI_APR', 'KI_ASR', 'KP_APR', 'KP_ASR', 'KT_Value', 'LS', 'LimitParam', 'MAX_SPD',
                      'MIT', 'MITParam', 'MST_ID', 'Motor', 'MotorDeviceCan', 'MotorStateResult', 'MotorType', 'MotorIdentityConfidence', 'MotorIdentityRegisters', 'MotorIdentityResult', 'MotorVariable', 'NPP', 'OC_Value', 'OT_Value', 'OV_Value', 'DamiaoCAN', 'DamiaoCANGroup', 'DamiaoCANGroupRecvResult', 'DamiaoCANRecvResult', 'PARAM', 'PMAX', 'POS_FORCE', 'POS_VEL', 'ParamResult', 'PosForceParam', 'PosVelParam', 'VelParam', 'Rs', 'SN', 'STATE', 'TIMEOUT', 'TMAX', 'UV_Value', 'VEL', 'VL_c1', 'VMAX', 'V_BW', 'can_br', 'dir', 'hw_ver', 'k1', 'k2', 'm_off', 'p_m', 'sub_ver', 'sw_ver', 'u_off', 'v_off', 'xout']


class MotorComponent(DMDeviceCollection):
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    def __init__(self, can_socket: CANSocket) -> None:
        ...

    def init_motor_devices(self, motor_types: collections.abc.Sequence[MotorType], send_can_ids: collections.abc.Sequence[int], recv_can_ids: collections.abc.Sequence[int], use_fd: bool, control_modes: collections.abc.Sequence[ControlMode] = ..., offset: collections.abc.Sequence[float | None] | None = ..., reversed: collections.abc.Sequence[bool | None] | None = ...) -> None:
        ...


class CANDevice:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    def get_recv_can_id(self) -> int:
        ...

    def get_recv_can_mask(self) -> int:
        ...

    def get_send_can_id(self) -> int:
        ...

    def is_fd_enabled(self) -> bool:
        ...


class CANDeviceCollection:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    def __init__(self, can_socket: CANSocket) -> None:
        ...

    def add_device(self, device: CANDevice) -> None:
        ...

    @typing.overload
    def dispatch_frame_callback(self, frame: CanFrame) -> None:
        ...

    @typing.overload
    def dispatch_frame_callback(self, frame: CanFdFrame) -> None:
        ...

    def get_devices(self) -> collections.abc.Mapping[int, CANDevice]:
        ...

    def remove_device(self, device: CANDevice) -> None:
        ...


class CANPacket:
    send_can_id: int

    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    def __init__(self) -> None:
        ...

    @property
    def data(self) -> list[int]:
        ...

    @data.setter
    def data(self, arg: collections.abc.Sequence[int]) -> None:
        ...


class CANHelperException(RuntimeError):
    pass


class CANInterfaceStatus:
    exists: bool
    is_can: bool
    up: bool
    running: bool
    ifindex: int
    mtu: int
    bitrate: int | None
    dbitrate: int | None
    fd_enabled: bool
    restart_ms: int | None
    def __init__(self) -> None: ...


class CANInterfaceConfig:
    bitrate: int
    dbitrate: int
    fd_enabled: bool
    sample_point: float
    dsample_point: float
    dsjw: int
    restart_ms: int | None
    bring_up: bool
    def __init__(self) -> None: ...


class CANHelper:
    def __init__(self, interface: str) -> None: ...
    @property
    def interface(self) -> str: ...
    def exists(self) -> bool: ...
    def status(self) -> CANInterfaceStatus: ...
    def can_configure_without_sudo(self) -> bool: ...
    def set_up(self) -> None: ...
    def set_down(self) -> None: ...
    def set_bitrate(self, bitrate: int, dbitrate: int, fd: bool) -> None: ...
    def configure(self, config: CANInterfaceConfig) -> None: ...


class CANSocket:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    def __init__(self, interface: str, enable_fd: bool = False) -> None:
        ...

    def get_interface(self) -> str:
        ...

    def get_socket_fd(self) -> int:
        ...

    def is_canfd_enabled(self) -> bool:
        ...

    def is_initialized(self) -> bool:
        ...

    def read_can_frame(self, frame: CanFrame) -> bool:
        ...

    def read_canfd_frame(self, frame: CanFdFrame) -> bool:
        ...

    def read_raw_frame(self, buffer_size: int) -> bytes:
        ...

    def write_can_frame(self, frame: CanFrame) -> bool:
        ...

    def write_canfd_frame(self, frame: CanFdFrame) -> bool:
        ...

    def write_raw_frame(self, data: bytes) -> int:
        ...


class CANSocketException(RuntimeError):
    pass


class CallbackMode(enum.Enum):
    IGNORE = ...
    PARAM = ...
    STATE = ...


class CanFdFrame:
    can_id: int
    data: bytes
    flags: int
    len: int

    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    def __init__(self) -> None:
        ...


class CanFrame:
    can_dlc: int
    can_id: int
    data: bytes

    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    def __init__(self) -> None:
        ...


class CanPacketDecoder:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    @staticmethod
    def parse_motor_state_data(
        motor: Motor,
        data: collections.abc.Sequence[int],
    ) -> MotorStateResult:
        ...

    @staticmethod
    def parse_motor_param_data(
        data: collections.abc.Sequence[int],
    ) -> ParamResult:
        ...


class CanPacketEncoder:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    @staticmethod
    def create_refresh_command(motor: Motor) -> CANPacket:
        ...

    @staticmethod
    def create_enable_command(motor: Motor) -> CANPacket:
        ...

    @staticmethod
    def create_disable_command(motor: Motor) -> CANPacket:
        ...

    @staticmethod
    def create_set_zero_command(motor: Motor) -> CANPacket:
        ...

    @staticmethod
    def create_mit_control_command(motor: Motor, mit_param: MITParam) -> CANPacket:
        ...

    @staticmethod
    def create_posvel_control_command(motor: Motor, posvel_param: PosVelParam) -> CANPacket:
        ...

    @staticmethod
    def create_vel_control_command(motor: Motor, vel_param: VelParam) -> CANPacket:
        ...

    @staticmethod
    def create_posforce_control_command(motor: Motor, posforce_param: PosForceParam) -> CANPacket:
        ...

    @staticmethod
    def create_query_param_command(motor: Motor, rid: int) -> CANPacket:
        ...


class ControlMode(enum.Enum):
    MIT = ...
    POS_FORCE = ...
    POS_VEL = ...
    VEL = ...


class DMDeviceCollection:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    def __init__(self, can_socket: CANSocket) -> None:
        ...

    def disable_all(self) -> None:
        ...

    def enable_all(self) -> None:
        ...

    def get_device_collection(self) -> CANDeviceCollection:
        ...

    def get_motors(self) -> list[Motor]:
        ...

    def mit_control_all(self, mit_params: collections.abc.Sequence[MITParam]) -> None:
        ...

    def mit_control_one(self, index: int, mit_param: MITParam) -> None:
        ...

    def posforce_control_all(self, posforce_params: collections.abc.Sequence[PosForceParam]) -> None:
        ...

    def posforce_control_one(self, index: int, posforce_param: PosForceParam) -> None:
        ...

    def posvel_control_all(self, posvel_params: collections.abc.Sequence[PosVelParam]) -> None:
        ...

    def posvel_control_one(self, index: int, posvel_param: PosVelParam) -> None:
        ...

    def vel_control_one(self, index: int, vel_param: VelParam) -> None:
        ...

    def vel_control_all(self, vel_params: collections.abc.Sequence[VelParam]) -> None:
        ...

    def query_param_all(self, rid: int) -> None:
        ...

    def refresh_all(self) -> None:
        ...

    def set_callback_mode_all(self, callback_mode: CallbackMode) -> None:
        ...

    def set_control_mode_all(self, mode: ControlMode) -> None:
        ...

    def set_control_mode_one(self, index: int, mode: ControlMode) -> None:
        ...

    def set_zero_all(self) -> None:
        ...


class LimitParam:
    pMax: float
    tMax: float
    vMax: float

    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, pMax: float, vMax: float, tMax: float) -> None: ...


class MITParam:
    dq: float
    kd: float
    kp: float
    q: float
    tau: float

    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    @typing.overload
    def __init__(self) -> None:
        ...

    @typing.overload
    def __init__(self, kp: float, kd: float, q: float, dq: float, tau: float) -> None:
        ...


class Motor:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    @staticmethod
    def get_limit_param(motor_type: MotorType) -> LimitParam:
        ...

    @typing.overload
    def __init__(self, motor_type: MotorType, send_can_id: int, recv_can_id: int,
                 offset: float = 0.0, reversed: bool = False) -> None: ...

    @typing.overload
    def __init__(self, limits: LimitParam, send_can_id: int, recv_can_id: int,
                 motor_type: MotorType = ..., offset: float = 0.0, reversed: bool = False) -> None: ...

    def get_motor_type(self) -> MotorType:
        ...

    def get_offset(self) -> float:
        ...

    def is_reversed(self) -> bool:
        ...

    def get_limits(self) -> LimitParam:
        ...

    def set_limits(self, limits: LimitParam) -> None:
        ...

    def get_param(self, rid: int) -> float:
        ...

    def get_position(self) -> float:
        ...

    def get_recv_can_id(self) -> int:
        ...

    def get_send_can_id(self) -> int:
        ...

    def get_fault(self) -> int: ...

    def get_state_tmos(self) -> int:
        ...

    def get_state_trotor(self) -> int:
        ...

    def get_torque(self) -> float:
        ...

    def get_velocity(self) -> float:
        ...

    def is_enabled(self) -> bool:
        ...


class MotorDeviceCan(CANDevice):
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    def __init__(self, motor: Motor, recv_can_mask: int, use_fd: bool) -> None:
        ...

    @typing.overload
    def callback(self, frame: CanFrame) -> None:
        ...

    @typing.overload
    def callback(self, frame: CanFdFrame) -> None:
        ...

    def create_can_frame(self, send_can_id: int, data: collections.abc.Sequence[int]) -> CanFrame:
        ...

    def create_canfd_frame(self, send_can_id: int, data: collections.abc.Sequence[int]) -> CanFdFrame:
        ...

    def get_motor(self) -> Motor:
        ...

    def set_callback_mode(self, callback_mode: CallbackMode) -> None:
        ...


class MotorStateResult:
    fault: int
    position: float
    t_mos: int
    t_rotor: int
    torque: float
    valid: bool
    velocity: float

    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    def __init__(self) -> None:
        ...


class MotorType(enum.Enum):
    COUNT = ...
    UNKNOWN = ...
    DM10010 = ...
    DM10010L = ...
    DM3507 = ...
    DM4310 = ...
    DM4310_48V = ...
    DM4340 = ...
    DM4340_48V = ...
    DM6006 = ...
    DM8006 = ...
    DM8009 = ...
    DMG6220 = ...
    DMH3510 = ...
    DMH6215 = ...


class MotorIdentityConfidence(enum.Enum):
    UNKNOWN = ...
    PROBABLE = ...
    EXACT = ...


class MotorIdentityRegisters:
    hw_ver: int | None
    sw_ver: int | None
    sn: int | None
    npp: int | None
    sub_ver: int | None
    rs: float | None
    ls: float | None
    flux: float | None
    gr: float | None
    pmax: float | None
    vmax: float | None
    tmax: float | None
    def __init__(self) -> None: ...


class MotorIdentityResult:
    send_can_id: int
    responded: bool
    protocol_limits: LimitParam | None
    protocol_family: str
    motor_type: MotorType | None
    confidence: MotorIdentityConfidence
    model_name: str
    reason: str
    hw_version_ascii: str
    sw_version_ascii: str
    serial_ascii: str
    registers: MotorIdentityRegisters
    def __init__(self) -> None: ...


class MotorVariable(enum.Enum):
    ACC = ...
    COUNT = ...
    CTRL_MODE = ...
    DEC = ...
    Damp = ...
    Deta = ...
    ESC_ID = ...
    Flux = ...
    GREF = ...
    Gr = ...
    IQ_c1 = ...
    I_BW = ...
    Inertia = ...
    KI_APR = ...
    KI_ASR = ...
    KP_APR = ...
    KP_ASR = ...
    KT_Value = ...
    LS = ...
    MAX_SPD = ...
    MST_ID = ...
    NPP = ...
    OC_Value = ...
    OT_Value = ...
    OV_Value = ...
    PMAX = ...
    Rs = ...
    SN = ...
    TIMEOUT = ...
    TMAX = ...
    UV_Value = ...
    VL_c1 = ...
    VMAX = ...
    V_BW = ...
    can_br = ...
    dir = ...
    hw_ver = ...
    k1 = ...
    k2 = ...
    m_off = ...
    p_m = ...
    sub_ver = ...
    sw_ver = ...
    u_off = ...
    v_off = ...
    xout = ...


class DamiaoCAN:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """Create and return a new object."""

    def __init__(self, can_interface: str, enable_fd: bool = False) -> None:
        ...

    def disable_all(self) -> None:
        ...

    def enable_all(self) -> None:
        ...

    def expected_response_count(self) -> int:
        ...

    def flush_rx(self) -> int:
        ...

    def get_master_can_device_collection(self) -> CANDeviceCollection:
        ...

    def get_motor(self, index: int) -> Motor:
        ...

    def get_motors(self) -> list[Motor]:
        ...

    def init_motors(self, send_ids: collections.abc.Sequence[int], recv_ids: collections.abc.Sequence[int], motor_types: collections.abc.Sequence[MotorType | None] | None = ..., control_modes: collections.abc.Sequence[ControlMode] | None = ..., offset: collections.abc.Sequence[float | None] | None = ..., reversed: collections.abc.Sequence[bool | None] | None = ...) -> None:
        """Initialize motors; omitted/None motor types use register-based AUTO limits."""
        ...

    def init_motors_with_limits(self, limit_params: collections.abc.Sequence[LimitParam], send_can_ids: collections.abc.Sequence[int], recv_can_ids: collections.abc.Sequence[int], control_modes: collections.abc.Sequence[ControlMode] = ..., offset: collections.abc.Sequence[float | None] | None = ..., reversed: collections.abc.Sequence[bool | None] | None = ...) -> None:
        ...

    def set_motor_limits_one(self, index: int, limits: LimitParam) -> None:
        ...

    def mit_control_all(self, mit_params: collections.abc.Sequence[MITParam]) -> None:
        ...

    def mit_control_one(self, index: int, mit_param: MITParam) -> None:
        ...

    def posforce_control_all(self, posforce_params: collections.abc.Sequence[PosForceParam]) -> None:
        ...

    def posforce_control_one(self, index: int, posforce_param: PosForceParam) -> None:
        ...

    def posvel_control_all(self, posvel_params: collections.abc.Sequence[PosVelParam]) -> None:
        ...

    def posvel_control_one(self, index: int, posvel_param: PosVelParam) -> None:
        ...

    def query_param_all(self, rid: int) -> None:
        ...

    def query_param_one(self, index: int, rid: int) -> None:
        ...

    def probe_motor_identity(self, send_can_id: int, timeout_us: int = 100000) -> MotorIdentityResult:
        ...

    def recv_all(self, timeout_us: int = 500) -> DamiaoCANRecvResult:
        ...

    def refresh_all(self) -> None:
        ...

    def refresh_one(self, index: int) -> None:
        ...

    def set_callback_mode_all(self, callback_mode: CallbackMode) -> None:
        ...

    def set_control_mode_all(self, mode: ControlMode) -> None:
        ...

    def set_control_mode_one(self, index: int, mode: ControlMode) -> None:
        ...

    def set_zero(self, index: int) -> None:
        ...

    def set_zero_all(self) -> None:
        ...

    def vel_control_all(self, vel_params: collections.abc.Sequence[VelParam]) -> None:
        ...

    def vel_control_one(self, index: int, vel_param: VelParam) -> None:
        ...


class ParamResult:
    rid: int
    valid: bool
    value: float

    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    def __init__(self) -> None:
        ...


class PosForceParam:
    dq: float
    i: float
    q: float

    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    @typing.overload
    def __init__(self) -> None:
        ...

    @typing.overload
    def __init__(self, q: float, dq: float, i: float) -> None:
        ...


class PosVelParam:
    dq: float
    q: float

    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    @typing.overload
    def __init__(self) -> None:
        ...

    @typing.overload
    def __init__(self, q: float, dq: float) -> None:
        ...


class VelParam:
    dq: float

    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    @typing.overload
    def __init__(self) -> None:
        ...

    @typing.overload
    def __init__(self, dq: float) -> None:
        ...


class DamiaoCANRecvResult:
    can_interface: str
    expect: int
    received: int
    ok: bool
    missing: list[int]

    def __str__(self) -> str:
        ...

    def __repr__(self) -> str:
        ...


class DamiaoCANGroupRecvResult:
    @property
    def ok(self) -> bool:
        ...

    def size(self) -> int:
        ...

    def get(self, index: int | None = None, can_id: str | None = None) -> DamiaoCANRecvResult:
        ...

    def __str__(self) -> str:
        ...

    def __repr__(self) -> str:
        ...


class DamiaoCANGroup:
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """

    def __init__(self, can_interfaces: collections.abc.Sequence[str], enable_fd: bool = False) -> None:
        ...

    def size(self) -> int:
        ...

    @typing.overload
    def get_device(self, index: int) -> DamiaoCAN:
        ...

    @typing.overload
    def get_device(self, can_interface: str) -> DamiaoCAN:
        ...

    def enable_all(self) -> None:
        ...

    def disable_all(self) -> None:
        ...

    def set_zero_all(self) -> None:
        ...

    def flush_rx(self) -> None:
        ...

    def refresh_all(self) -> None:
        ...

    def recv_all(self, timeout_us: int = 500) -> DamiaoCANGroupRecvResult:
        ...


ACC: MotorVariable  # value = MotorVariable.ACC
COUNT: MotorVariable  # value = MotorVariable.COUNT
CTRL_MODE: MotorVariable  # value = MotorVariable.CTRL_MODE
DEC: MotorVariable  # value = MotorVariable.DEC
DM10010: MotorType  # value = MotorType.DM10010
DM10010L: MotorType  # value = MotorType.DM10010L
DM3507: MotorType  # value = MotorType.DM3507
DM4310: MotorType  # value = MotorType.DM4310
DM4310_48V: MotorType  # value = MotorType.DM4310_48V
DM4340: MotorType  # value = MotorType.DM4340
DM4340_48V: MotorType  # value = MotorType.DM4340_48V
DM6006: MotorType  # value = MotorType.DM6006
DM8006: MotorType  # value = MotorType.DM8006
DM8009: MotorType  # value = MotorType.DM8009
DMG6220: MotorType  # value = MotorType.DMG6220
DMH3510: MotorType  # value = MotorType.DMH3510
DMH6215: MotorType  # value = MotorType.DMH6215
Damp: MotorVariable  # value = MotorVariable.Damp
Deta: MotorVariable  # value = MotorVariable.Deta
ESC_ID: MotorVariable  # value = MotorVariable.ESC_ID
Flux: MotorVariable  # value = MotorVariable.Flux
GREF: MotorVariable  # value = MotorVariable.GREF
Gr: MotorVariable  # value = MotorVariable.Gr
IGNORE: CallbackMode  # value = CallbackMode.IGNORE
IQ_c1: MotorVariable  # value = MotorVariable.IQ_c1
I_BW: MotorVariable  # value = MotorVariable.I_BW
Inertia: MotorVariable  # value = MotorVariable.Inertia
KI_APR: MotorVariable  # value = MotorVariable.KI_APR
KI_ASR: MotorVariable  # value = MotorVariable.KI_ASR
KP_APR: MotorVariable  # value = MotorVariable.KP_APR
KP_ASR: MotorVariable  # value = MotorVariable.KP_ASR
KT_Value: MotorVariable  # value = MotorVariable.KT_Value
LS: MotorVariable  # value = MotorVariable.LS
MAX_SPD: MotorVariable  # value = MotorVariable.MAX_SPD
MIT: ControlMode  # value = ControlMode.MIT
MST_ID: MotorVariable  # value = MotorVariable.MST_ID
NPP: MotorVariable  # value = MotorVariable.NPP
OC_Value: MotorVariable  # value = MotorVariable.OC_Value
OT_Value: MotorVariable  # value = MotorVariable.OT_Value
OV_Value: MotorVariable  # value = MotorVariable.OV_Value
PARAM: CallbackMode  # value = CallbackMode.PARAM
PMAX: MotorVariable  # value = MotorVariable.PMAX
POS_FORCE: ControlMode  # value = ControlMode.POS_FORCE
POS_VEL: ControlMode  # value = ControlMode.POS_VEL
Rs: MotorVariable  # value = MotorVariable.Rs
SN: MotorVariable  # value = MotorVariable.SN
STATE: CallbackMode  # value = CallbackMode.STATE
TIMEOUT: MotorVariable  # value = MotorVariable.TIMEOUT
TMAX: MotorVariable  # value = MotorVariable.TMAX
UV_Value: MotorVariable  # value = MotorVariable.UV_Value
VEL: ControlMode  # value = ControlMode.VEL
VL_c1: MotorVariable  # value = MotorVariable.VL_c1
VMAX: MotorVariable  # value = MotorVariable.VMAX
V_BW: MotorVariable  # value = MotorVariable.V_BW
can_br: MotorVariable  # value = MotorVariable.can_br
dir: MotorVariable  # value = MotorVariable.dir
hw_ver: MotorVariable  # value = MotorVariable.hw_ver
k1: MotorVariable  # value = MotorVariable.k1
k2: MotorVariable  # value = MotorVariable.k2
m_off: MotorVariable  # value = MotorVariable.m_off
p_m: MotorVariable  # value = MotorVariable.p_m
sub_ver: MotorVariable  # value = MotorVariable.sub_ver
sw_ver: MotorVariable  # value = MotorVariable.sw_ver
u_off: MotorVariable  # value = MotorVariable.u_off
v_off: MotorVariable  # value = MotorVariable.v_off
xout: MotorVariable  # value = MotorVariable.xout


class MotorLimitResolutionError(RuntimeError):
    ...


# Passive System Identification telemetry v1; wire values are never host-adjusted.
class SysIdOperation(enum.Enum):
    START = ...
    STOP = ...
    HEARTBEAT = ...
    INFO = ...
    STATUS = ...
    SCAN_LOWER = ...
    SCAN_UPPER = ...
    SCAN_HOME = ...
    SCAN_SPEED = ...
    SCAN_ACCEL = ...
    SCAN_TORQUE = ...
    SCAN_TEMPERATURE = ...
    SCAN_REPEATS = ...
    SCAN_ERROR = ...
    SCAN_SECONDS = ...
    SCAN_START = ...
    SCAN_STATUS = ...
    SCAN_ABORT = ...

    DATA_ACK = 19
    DATA_REPLAY = 20
    SAFETY_STATUS = 21
    LIVE_START = 22
    LIVE_STOP = 23
    GUARD_LOWER = 24
    GUARD_UPPER = 25
    GUARD_SPEED = 26
    GUARD_TORQUE = 27
    GUARD_ARM = 28


class SysIdResult(enum.Enum):
    ACCEPTED = ...
    BAD_VERSION = ...
    BAD_ARGUMENT = ...
    BUSY = ...
    TRANSPORT_NOT_READY = ...
    UNSUPPORTED_OPERATION = ...
    MEASUREMENTS_NOT_READY = ...


class SysIdProtocolError(RuntimeError):
    ...


class SysIdTimeoutError(RuntimeError):
    ...


class SysIdSessionError(RuntimeError):
    ...


class SysIdFrame:
    def __init__(self) -> None: ...
    can_id: int
    fd: bool
    flags: int
    payload: bytes
    host_receive_time_ns: int


class SysIdAck:
    version: int
    operation: SysIdOperation
    result: SysIdResult
    request_sequence: int
    raw: SysIdFrame


class SysIdInformation:
    version: int
    message_type: int
    node: int
    active: bool
    request_sequence: int
    session_id: int
    control_tick: int
    sample_drop_count: int
    request_drop_count: int
    tx_retry_count: int
    next_sample_sequence: int
    output_torque_constant: float
    gear_ratio: float
    factory_velocity_previous_weight: float
    iq_filter_beta: float
    current_limit: float
    accepted_command_sequence: int
    applied_command_sequence: int
    sample_period_ticks: int
    stop_reason: int
    mode: int
    fault: int
    raw: SysIdFrame


class SysIdSample:
    version: int
    node: int
    flags: int
    sequence: int
    endpoint_tick: int
    applied_command_sequence: int
    sample_drop_count: int
    position: float
    velocity: float
    averaged_iq: float
    torque_estimate: float
    mit_feedforward_torque: float
    instantaneous_iq: float
    applied_command_tick: int
    temperature: float
    velocity_tick: int
    position_publication_tick: int
    interval_ticks: int
    mode: int
    fault: int
    raw: SysIdFrame
    @property
    def armed(self) -> bool: ...
    @property
    def current_saturated(self) -> bool: ...
    @property
    def voltage_saturated(self) -> bool: ...
    @property
    def warmup(self) -> bool: ...
    @property
    def command_unknown(self) -> bool: ...


class SysIdMeasurement:
    sample: SysIdSample
    session_id: int
    timeline_epoch: int
    unwrapped_sequence: int
    unwrapped_endpoint_tick: int
    sequence_gap: int
    position: float
    velocity: float
    averaged_iq: float
    torque_estimate: float
    mit_feedforward_torque: float
    instantaneous_iq: float
    mos_temperature: float


class SysIdSafetyInformation:
    node: int
    request_sequence: int
    control_tick: int
    command_lease_ticks: int
    live_period_ticks: int
    last_command_tick: int
    live_overwritten: int
    deadman_enabled: bool
    deadman_latched: bool
    live_active: bool
    guard_enabled: bool
    guard_reason: int
    guard_lower: float
    guard_upper: float
    guard_max_velocity: float
    guard_torque_limit: float
    raw: SysIdFrame


class SysIdReply:
    safety: SysIdSafetyInformation | None
    scan: SysIdScanRecord | None
    ack: SysIdAck | None
    information: SysIdInformation | None
    @property
    def accepted(self) -> bool: ...


class SysIdStartResult:
    reply: SysIdReply
    recovery_status: SysIdInformation | None
    session_id: int
    recovered: bool
    accepted: bool


class SysIdDiagnostics:
    host_scan_dropped: int
    host_sample_dropped: int
    host_reply_dropped: int
    host_other_dropped: int
    socket_dropped: int
    invalid_frames: int
    unassociated_samples: int
    session_discarded_samples: int
    sequence_gaps: int
    timeline_resets: int


class SysIdStopResult:
    reply: SysIdReply
    samples: list[SysIdMeasurement]
    final_status: SysIdInformation
    drain_limit_reached: bool


class SystemIdentification:
    def safety_status(self, timeout_us: int = 100000) -> SysIdReply: ...
    def configure_guard(self, lower: float, upper: float, max_velocity: float, torque_limit: float, timeout_us: int = 100000) -> None: ...
    def start_live(self, timeout_us: int = 100000) -> SysIdReply: ...
    def stop_live(self, timeout_us: int = 100000) -> SysIdReply: ...
    def read_latest(self, timeout_us: int = 0, max_frames: int = 256) -> SysIdMeasurement | None: ...
    def __init__(self, interface: str, node: int, offset: float = 0.0,
                 reversed: bool = False, queue_capacity: int = 2048, live_only: bool = False) -> None: ...

    def info(self, timeout_us: int = 100000) -> SysIdReply: ...
    def status(self, timeout_us: int = 100000) -> SysIdReply: ...

    def start(self, rate_hz: int = 500, timeout_us: int = 100000,
              reliable: bool = False) -> SysIdStartResult: ...

    def acknowledge_data(self, next_sequence: int,
                         timeout_us: int = 20000) -> SysIdReply: ...
    def replay_data(self, next_sequence: int,
                    timeout_us: int = 20000) -> SysIdReply: ...

    def heartbeat(self, timeout_us: int = 100000) -> SysIdReply: ...
    def stop(self, timeout_us: int = 100000, drain_timeout_us: int = 50000,
             max_drain_frames: int = 4096) -> SysIdStopResult: ...

    def poll(self, timeout_us: int = 0, max_frames: int = 256) -> int: ...
    def read_samples(self, timeout_us: int = 0,
                     max_frames: int = 256) -> list[SysIdMeasurement]: ...

    def take_replies(self) -> list[SysIdReply]: ...
    def take_other_frames(self) -> list[SysIdFrame]: ...
    def diagnostics(self) -> SysIdDiagnostics: ...
    @property
    def owned_session(self) -> int | None: ...
    def configure_scan(self, config: SysIdScanConfig,
                       timeout_us: int = 100000) -> None: ...

    def start_scan(self, timeout_us: int = 100000) -> SysIdReply: ...
    def scan_status(self, timeout_us: int = 100000) -> SysIdReply: ...
    def abort_scan(self, timeout_us: int = 100000) -> SysIdReply: ...
    def take_scan_records(self) -> list[SysIdScanRecord]: ...
    def close(self) -> None: ...
    def __enter__(self) -> SystemIdentification: ...
    def __exit__(self, exc_type: object, exc_value: object,
                 traceback: object) -> None: ...


def decode_sysid_safety(frame: SysIdFrame) -> SysIdSafetyInformation: ...
def decode_sysid_live(frame: SysIdFrame, offset: float = 0.0, reversed: bool = False) -> SysIdMeasurement: ...
def decode_sysid_ack(frame: SysIdFrame) -> SysIdAck: ...
def decode_sysid_information(frame: SysIdFrame) -> SysIdInformation: ...
def decode_sysid_sample(frame: SysIdFrame) -> SysIdSample: ...
def encode_sysid_request(node: int, operation: SysIdOperation,
                         argument: int, request_sequence: int) -> bytes: ...


def sysid_command_sequence_distance(previous: int, current: int) -> int: ...


class SysIdScanConfig:
    def __init__(self) -> None: ...
    lower: float
    upper: float
    home: float
    speed: float
    acceleration: float
    torque_limit: float
    temperature_limit: float
    repeats: float
    tracking_error: float
    max_seconds: float


class SysIdScanRecord:
    node: int
    phase: int
    reason: int
    sequence: int
    endpoint_tick: int
    capture_session: int
    scan_session: int
    completed_legs: int
    start_tick: int
    end_tick: int
    planned_position: float
    command_velocity: float
    raw_torque: float
    position: float
    home: float
    target: float
    owned: bool
    active: bool
    raw: SysIdFrame


def decode_sysid_scan_record(frame: SysIdFrame) -> SysIdScanRecord: ...
