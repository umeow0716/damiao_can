# Damiao CAN

Python-first SocketCAN library for controlling DaMiao motors on Linux. The Python package uses the C++ core in this repository for low-level CAN communication.

## Requirements

Linux with SocketCAN, Python 3.10+, and a CAN/CAN-FD adapter. Installing from source also requires a C++17 compiler and CMake.

## Install

```bash
pip install "git+https://github.com/umeow0716/damiao_can.git@main#subdirectory=python"
```

## Basic Python Usage

```python
import damiao_can as dc

# Configure SocketCAN before DamiaoCAN creates its socket.
helper = dc.CANHelper("can0")
helper.set_down()
helper.set_bitrate(1_000_000, 5_000_000, True)
helper.set_up()

# True enables CAN-FD socket support.
device = dc.DamiaoCAN("can0", True)

device.init_motors(
    [dc.MotorType.DM4310],
    [0x01],  # send_id
    [0x11],  # recv_id
    [dc.ControlMode.MIT],
)

device.enable_all()

device.flush_rx()
device.refresh_all()
result = device.recv_all()

print(result)
# {can_interface: "can0", expected: 1, received: 1, missing: [], ok: True}

device.disable_all()
```

For multiple CAN interfaces, configure every interface before creating the group:

```python
interfaces = ["can0", "can1"]

for interface in interfaces:
    helper = dc.CANHelper(interface)
    helper.set_down()
    helper.set_bitrate(1_000_000, 5_000_000, True)
    helper.set_up()

group = dc.DamiaoCANGroup(interfaces, True)

group.flush_rx()
group.refresh_all()
results = group.recv_all()

print(results)
print(results.get(can_id="can0"))
```

## Python CLI

The package also provides a small command-line interface:

```bash
python -m damiao_can --help
```

Use the command-specific help for available options and examples:

```bash
python -m damiao_can set-zero --help
python -m damiao_can set-baudrate --help
python -m damiao_can drop-test --help
```

## CAN Interface Helper

`CANHelper` is independent from `DamiaoCAN`. Use it before creating a device or group when the SocketCAN interface needs to be configured.

```python
import damiao_can as dc

helper = dc.CANHelper("can0")
print(helper.status())

helper.set_down()
helper.set_bitrate(1_000_000, 5_000_000, True)
helper.set_up()

device = dc.DamiaoCAN("can0", True)
```

Do not bring an interface down after `DamiaoCAN` or `DamiaoCANGroup` has created sockets for it. Read-only status checks do not require root privileges. Configuration uses the process's existing network capability when available and otherwise requests authorization through `sudo`.

## License

Licensed under the Apache License 2.0. See `LICENSE.txt`.

This repository contains modifications to code originally copyright 2025 Enactic, Inc.
