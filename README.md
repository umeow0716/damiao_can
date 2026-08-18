# Damiao CAN

> [!NOTE]
> This repository is a DaMiao CAN motor-control library derived from an Enactic CAN implementation.
>
> The intended usage of this fork is to install and use the Python package directly from this Git repository.
>
> No system package is required. A local/Git Python install builds the extension against the C++ source tree included in this repository, keeping the Python bindings and C++ core in sync.

The repository provides a SocketCAN C++ core and Python bindings focused on direct DaMiao motor control.

## Features

- Python package installation from the `python/` subdirectory through Git URLs.
- Python type stubs (`.pyi`) and `py.typed` for IDE completion and static analysis.
- Simplified high-level `DamiaoCAN` API with direct motor access.
- A single `recv_all()` receive API, plus `flush_rx()` and `refresh_all()` for explicit receive-flow composition.
- Pure velocity control support through `ControlMode.VEL`, `VelParam`, and `vel_control_one/all`.
- Additional Python examples for low-level motor-control usage.

---

## Requirements

This package is intended for Linux systems with SocketCAN support.

You need:

- Linux with SocketCAN support
- A working CAN or CAN-FD adapter
- Python 3.10+
- A C++17-capable compiler
- CMake / Ninja or an equivalent build backend

On Ubuntu, a typical setup is:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  ninja-build \
  python3-dev \
  python3-venv \
  can-utils
```

`can-utils` is optional but useful for debugging with tools such as `candump`, `cansend`, and `ip link`.

---

## Install

### Install with pip

```bash
pip install "git+https://github.com/umeow0716/damiao_can.git@main#subdirectory=python"
```

### Install with uv

```bash
uv add "git+https://github.com/umeow0716/damiao_can.git@main#subdirectory=python"
```

### Local editable install

```bash
git clone https://github.com/umeow0716/damiao_can.git
cd damiao_can/python
pip install -e .
```

Or with `uv`:

```bash
git clone https://github.com/umeow0716/damiao_can.git
cd damiao_can/python
uv pip install -e .
```

The Python package builds a native extension module. When installed from this repository, the build uses the C++ source tree included in this repo.

---

## Setup SocketCAN

### Classic CAN

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### CAN-FD

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
```

Check the interface:

```bash
ip -details link show can0
```

Monitor frames:

```bash
candump can0
```

---

## CAN interface helper

`CANHelper` can inspect a SocketCAN interface without root privileges. Read-only probes do not invoke `sudo`.

```python
import damiao_can as dc

helper = dc.CANHelper("can0")
status = helper.status()

print(status.exists, status.is_can, status.up, status.mtu)
print(status.bitrate, status.dbitrate, status.fd_enabled)
```

Configuration is explicit. If the process already has `CAP_NET_ADMIN`, it is used directly. Otherwise the helper starts `sudo` as a child process and lets `sudo` request authorization from the controlling terminal. Passwords are never passed through the Python/C++ API.

```python
config = dc.CANInterfaceConfig()
config.bitrate = 1_000_000
config.dbitrate = 5_000_000
config.fd_enabled = True
config.sample_point = 0.75
config.dsample_point = 0.75
config.dsjw = 2
config.restart_ms = 100

helper.configure(config)
```

A successfully constructed `DamiaoCAN` also exposes the helper as `device.can_helper`. Use a standalone `CANHelper` when the interface may not exist yet, because `DamiaoCAN` still opens its SocketCAN socket during construction.

---

## Basic Python Usage

```python
import damiao_can as dc

device = dc.DamiaoCAN("can0", True)

device.init_motors(
    [dc.MotorType.DM4310],
    [0x01],  # send CAN ID
    [0x11],  # receive CAN ID
    [dc.ControlMode.MIT],
)

device.enable_all()

device.flush_rx()
device.refresh_all()
result = device.recv_all()
print(result)
# {can_interface: "can0", expected: 1, received: 1, missing: [], ok: True}
# Properties: can_interface, expect, received, missing, ok.

# The `missing` property contains motor send IDs. The result formatter displays
# those IDs as hexadecimal values such as 0x01.

# Multiple CAN interfaces use the same explicit receive flow. flush_rx() and
# refresh_all() iterate over each CAN device; recv_all() uses one receive
# worker thread per CAN interface.
group = dc.DamiaoCANGroup(["can0", "can1"], True)
group.flush_rx()
group.refresh_all()
results = group.recv_all()
print(results)

# Lookup by index:
print(results.get(index=0))

# Lookup by CAN interface. can_id takes priority when both arguments are set:
print(results.get(index=999, can_id="can1"))

device.disable_all()
```

---

## Development

Clone the repository:

```bash
git clone https://github.com/umeow0716/damiao_can.git
cd damiao_can
```

Install the Python package locally:

```bash
cd python
pip install -e .
```

For C++ development, see:

```text
dev/README.md
```

---

## Project origin

This codebase is derived from an Enactic CAN implementation and retains the original Apache-2.0 license and copyright notices. Git history records the detailed provenance.

---

## License

Licensed under the Apache License 2.0. See `LICENSE.txt` for details.

Copyright 2025 Enactic, Inc.
