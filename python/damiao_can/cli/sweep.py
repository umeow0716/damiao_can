# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""MIT torque chirp sweep command."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Any

from .common import (
    DEFAULT_BITRATE,
    DEFAULT_DBITRATE,
    RECV_ID_OFFSET,
    configure_interface,
    parse_int,
    parse_rate,
)

DEFAULT_AMPLITUDE_NM = 0.1
DEFAULT_START_HZ = 1.0
DEFAULT_STOP_HZ = 100.0
DEFAULT_SAMPLE_RATE_HZ = 1000.0
DEFAULT_DURATION_S = 10.0
DEFAULT_RESPONSE_TIMEOUT_US = 1000
DEFAULT_OUTPUT = "sweep.csv"

_MOTOR_TYPES = (
    "DM3507",
    "DM4310",
    "DM4310_48V",
    "DM4340",
    "DM4340_48V",
    "DM6006",
    "DM8006",
    "DM8009",
    "DM10010L",
    "DM10010",
    "DMH3510",
    "DMH6215",
    "DMG6220",
)


def _positive_float(value: str) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid number {value!r}") from exc
    if result <= 0.0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return result


def _nonnegative_int(value: str) -> int:
    try:
        result = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid integer {value!r}") from exc
    if result < 0:
        raise argparse.ArgumentTypeError("value must be non-negative")
    return result


def _add_interface_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "-i",
        "--interface",
        required=True,
        metavar="IFACE",
        help="SocketCAN interface to use (required, e.g. can0)",
    )
    parser.add_argument(
        "--no-fd",
        dest="fd",
        action="store_false",
        default=True,
        help="disable CAN-FD on the host interface (default: CAN-FD enabled)",
    )
    parser.add_argument(
        "--bitrate",
        type=parse_rate,
        default=DEFAULT_BITRATE,
        metavar="RATE",
        help="host nominal CAN bitrate (default: 1M)",
    )
    parser.add_argument(
        "--dbitrate",
        type=parse_rate,
        default=None,
        metavar="RATE",
        help="host CAN-FD data bitrate (default: 5M; ignored when --no-fd is used)",
    )


def _confirm(args: argparse.Namespace) -> bool:
    if args.yes:
        return True
    if not sys.stdin.isatty():
        raise ValueError("sweep requires an interactive confirmation; pass --yes to run non-interactively")

    print(
        "WARNING: this command enables the motor and injects a torque chirp.\n"
        f"  motor ID: 0x{args.motor_id:x}\n"
        f"  frequency: {args.start_hz:g} -> {args.stop_hz:g} Hz\n"
        f"  torque: bias {args.bias_nm:g} Nm, amplitude {args.amplitude_nm:g} Nm\n"
        f"  duration: {args.duration_s:g} s at {args.sample_rate_hz:g} samples/s\n"
        "Make sure the mechanism has clearance and an emergency stop is available."
    )
    answer = input("Type 'yes' to continue: ").strip().lower()
    return answer == "yes"


def _make_sweep_device(api: Any, args: argparse.Namespace) -> Any:
    if args.motor_id < 0 or args.motor_id + RECV_ID_OFFSET > 0x7FF:
        raise ValueError("--id must fit an 11-bit CAN ID with recv_id = id + 0x10")

    configure_interface(api, args)
    device = api.DamiaoCAN(args.interface, args.fd)
    motor_type = getattr(api.MotorType, args.motor_type)
    device.init_motors(
        [motor_type],
        [args.motor_id],
        [args.motor_id + RECV_ID_OFFSET],
        [api.ControlMode.MIT],
    )
    device.set_callback_mode_all(api.CallbackMode.IGNORE)
    return device


def _write_raw_csv(path: Path, result: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "scheduled_time_s",
                "command_time_s",
                "frequency_hz",
                "command_tau_nm",
                "tx_timestamp_ns",
                "rx_timestamp_ns",
                "round_trip_ns",
                "position_rad",
                "velocity_rad_s",
                "measured_torque_nm",
                "t_mos_c",
                "t_rotor_c",
                "valid",
            ]
        )
        for sample in result.samples:
            feedback = sample.feedback
            writer.writerow(
                [
                    sample.scheduled_time_s,
                    sample.command_time_s,
                    sample.frequency_hz,
                    sample.command_tau,
                    feedback.tx_timestamp_ns,
                    feedback.rx_timestamp_ns,
                    feedback.round_trip_ns,
                    feedback.position,
                    feedback.velocity,
                    feedback.torque,
                    feedback.t_mos,
                    feedback.t_rotor,
                    int(feedback.valid),
                ]
            )


def _cmd_sweep(args: argparse.Namespace) -> int:
    if args.sample_rate_hz <= 2.0 * max(args.start_hz, args.stop_hz):
        raise ValueError("--sample-rate-hz must be greater than twice the highest sweep frequency")
    if abs(args.bias_nm) + args.amplitude_nm <= 0.0:
        raise ValueError("torque excitation must be non-zero")
    if not _confirm(args):
        print("Sweep cancelled.")
        return 2

    from .common import load_api

    api = load_api()
    device = _make_sweep_device(api, args)

    config = api.MITTorqueSweepConfig()
    config.start_hz = args.start_hz
    config.stop_hz = args.stop_hz
    config.amplitude_nm = args.amplitude_nm
    config.bias_nm = args.bias_nm
    config.sample_rate_hz = args.sample_rate_hz
    config.duration_s = args.duration_s
    config.response_timeout_us = args.response_timeout_us

    output = Path(args.output)
    enabled = False
    try:
        device.flush_rx()
        device.enable_all()
        enabled = True
        result = api.run_mit_torque_chirp(device, 0, config)
    finally:
        if enabled:
            try:
                device.disable_all()
            except RuntimeError:
                pass

    _write_raw_csv(output, result)
    print(
        f"sweep complete: samples={len(result.samples)} valid={result.valid_samples} "
        f"dropped={result.dropped_samples} valid_ratio={result.valid_ratio:.3f} "
        f"elapsed={result.elapsed_s:.3f}s"
    )
    print(f"raw CSV: {output}")
    return 0 if result.ok else 2


def register(subparsers: Any) -> None:
    parser = subparsers.add_parser(
        "sweep",
        help="run a timestamped MIT torque chirp and save raw samples to CSV",
        description=(
            "Run a single-motor MIT torque chirp in the native C++ acquisition loop.\n"
            "The command enables the motor for the sweep and disables it afterward.\n"
            "No FFT/Bode processing is performed; output is timestamped raw CSV."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Default sweep:
  # 0.1 Nm, 1 -> 100 Hz, 1000 samples/s, 10 s, output sweep.csv
  python -m damiao_can sweep -i can0 --id 0x01

Override example:
  python -m damiao_can sweep -i can0 --id 0x01 --nm 0.05 \
      --start-hz 2 --end-hz 80 --rate-hz 1000 --duration 8 \
      --output motor01.csv

Non-interactive example:
  python -m damiao_can sweep -i can0 --id 0x01 --yes
""",
    )
    _add_interface_options(parser)
    parser.add_argument(
        "--id",
        dest="motor_id",
        required=True,
        type=parse_int,
        metavar="ID",
        help="motor send CAN ID; receive ID is ID + 0x10",
    )
    parser.add_argument(
        "--motor-type",
        choices=_MOTOR_TYPES,
        default="DM4310",
        help="motor type used for protocol limits (default: DM4310)",
    )
    parser.add_argument(
        "--start-hz",
        type=_positive_float,
        default=DEFAULT_START_HZ,
        help=f"chirp start frequency in Hz (default: {DEFAULT_START_HZ:g})",
    )
    parser.add_argument(
        "--stop-hz",
        "--end-hz",
        type=_positive_float,
        default=DEFAULT_STOP_HZ,
        help=f"chirp stop frequency in Hz (default: {DEFAULT_STOP_HZ:g})",
    )
    parser.add_argument(
        "--amplitude-nm",
        "--nm",
        type=_positive_float,
        default=DEFAULT_AMPLITUDE_NM,
        metavar="NM",
        help=f"sine torque amplitude in Nm (default: {DEFAULT_AMPLITUDE_NM:g})",
    )
    parser.add_argument(
        "--bias-nm",
        type=float,
        default=0.0,
        metavar="NM",
        help="constant torque bias in Nm (default: 0)",
    )
    parser.add_argument(
        "--sample-rate-hz",
        "--rate-hz",
        type=_positive_float,
        default=DEFAULT_SAMPLE_RATE_HZ,
        metavar="HZ",
        help=f"native acquisition rate in samples/s (default: {DEFAULT_SAMPLE_RATE_HZ:g})",
    )
    parser.add_argument(
        "--duration-s",
        "--duration",
        type=_positive_float,
        default=DEFAULT_DURATION_S,
        metavar="SEC",
        help=f"chirp duration in seconds (default: {DEFAULT_DURATION_S:g})",
    )
    parser.add_argument(
        "--response-timeout-us",
        type=_nonnegative_int,
        default=DEFAULT_RESPONSE_TIMEOUT_US,
        metavar="US",
        help=f"per-sample motor response timeout in us (default: {DEFAULT_RESPONSE_TIMEOUT_US})",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=DEFAULT_OUTPUT,
        metavar="CSV",
        help=f"raw CSV output path (default: {DEFAULT_OUTPUT})",
    )
    parser.add_argument(
        "-y",
        "--yes",
        action="store_true",
        help="skip the interactive motion-safety confirmation",
    )
    parser.set_defaults(func=_cmd_sweep)
