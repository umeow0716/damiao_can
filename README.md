# Damiao CAN

Python-first SocketCAN library for controlling DaMiao motors on Linux. The Python package uses the C++ core in this repository for low-level CAN communication.

## Requirements

Linux with SocketCAN, Python 3.10+, and a CAN/CAN-FD adapter. Installing from source also requires a C++17 compiler and CMake.

## Install

```bash
pip install "git+https://github.com/umeow0716/damiao_can.git@main#subdirectory=python"
```

If you do not want `CANHelper` to request root authorization through `sudo` when configuring a CAN interface, grant your login session the `CAP_NET_ADMIN` ambient capability. On Ubuntu/Debian with `pam_cap`:

```bash
sudo apt install libpam-cap
sudo sed -i \
  -e "/^[[:space:]]*\\^cap_net_admin[[:space:]]\\+${USER}[[:space:]]*$/d" \
  -e "1i ^cap_net_admin ${USER}" \
  /etc/security/capability.conf
sudo pam-auth-update --enable capability
sudo reboot
```

Log out and log back in after changing the PAM capability configuration. `CAP_NET_ADMIN` allows network-interface administration, so only grant it to users that should be allowed to configure network devices.

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

For multiple CAN interfaces, configure every interface before creating the group, then initialize the motors on each interface through its `DamiaoCAN` device:

```python
bus_configs = {
    "can0": {
        "motor_types": [dc.MotorType.DM4310],
        "send_ids": [0x01],
        "recv_ids": [0x11],
        "control_modes": [dc.ControlMode.MIT],
    },
    "can1": {
        "motor_types": [dc.MotorType.DM4310],
        "send_ids": [0x02],
        "recv_ids": [0x12],
        "control_modes": [dc.ControlMode.MIT],
    },
}

interfaces = list(bus_configs)

for interface in interfaces:
    helper = dc.CANHelper(interface)
    helper.set_down()
    helper.set_bitrate(1_000_000, 5_000_000, True)
    helper.set_up()

group = dc.DamiaoCANGroup(interfaces, True)

for interface, config in bus_configs.items():
    device = group.get_device(interface)
    device.init_motors(
        config["motor_types"],
        config["send_ids"],
        config["recv_ids"],
        config["control_modes"],
    )

group.enable_all()
group.flush_rx()
group.refresh_all()
results = group.recv_all()

print(results)
print(results.get(can_id="can0"))

group.disable_all()
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
python -m damiao_can sweep position --help
python -m damiao_can sweep torque --help
```

Measure POS_VEL position-command tracking and estimate the first -3 dB cutoff:

```bash
python -m damiao_can sweep position -i can0 --id 1
```

The default position experiment uses 20 logarithmically spaced points from 1 to 100 Hz,
up to +/-0.05 rad excursion, a 10 rad/s POS_VEL velocity ceiling, and `--wait-us 500`.
The excitation amplitude is reduced automatically at high frequency so the commanded sine
does not itself demand the full velocity ceiling. Raw timestamped samples and a gain/phase
CSV are written separately.

Run a drop test:

```bash
python -m damiao_can drop-test -i can0 --from 1 --to 8 --wait-us 500
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
