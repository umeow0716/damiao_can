# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Frequency-response sweep commands for DAMIAO motors."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

from .common import (
    DEFAULT_BITRATE,
    DEFAULT_DBITRATE,
    RECV_ID_OFFSET,
    configure_interface,
    parse_int,
    parse_rate,
)

# Position/POS_VEL tracking sweep defaults. The displacement is a maximum:
# the native runner reduces it at high frequency so the requested sine remains
# below 80% of the POS_VEL velocity ceiling.
DEFAULT_POSITION_AMPLITUDE_RAD = 0.05
DEFAULT_POSITION_START_HZ = 1.0
DEFAULT_POSITION_STOP_HZ = 100.0
DEFAULT_POSITION_VELOCITY_LIMIT_RAD_S = 10.0
DEFAULT_POSITION_WAIT_US = 500
DEFAULT_POSITION_POINTS = 20
DEFAULT_POSITION_SETTLING_CYCLES = 2
DEFAULT_POSITION_MEASURE_CYCLES = 3
DEFAULT_POSITION_OUTPUT = "sweep_position.csv"

# Existing MIT torque chirp defaults retained as the plant-identification mode.
DEFAULT_TORQUE_AMPLITUDE_NM = 0.1
DEFAULT_TORQUE_START_HZ = 1.0
DEFAULT_TORQUE_STOP_HZ = 100.0
DEFAULT_TORQUE_SAMPLE_RATE_HZ = 1000.0
DEFAULT_TORQUE_DURATION_S = 10.0
DEFAULT_TORQUE_RESPONSE_TIMEOUT_US = 1000
DEFAULT_TORQUE_OUTPUT = "sweep_torque.csv"

_MOTOR_TYPE_ALIASES = {
    "DM4310P": "DM4310",
    "DM4340P": "DM4340",
    "DM8009P": "DM8009",
}

_MOTOR_TYPES = (
    "auto",
    "DM3507",
    "DM4310",
    "DM4310P",
    "DM4310_48V",
    "DM4340",
    "DM4340P",
    "DM4340_48V",
    "DM6006",
    "DM8006",
    "DM8009",
    "DM8009P",
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
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and greater than zero")
    return result


def _positive_int(value: str) -> int:
    try:
        result = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid integer {value!r}") from exc
    if result <= 0:
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
        help="host CAN-FD data bitrate (default: 5M; ignored with --no-fd)",
    )


def _add_single_motor_options(parser: argparse.ArgumentParser) -> None:
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
        default="auto",
        help="fallback built-in protocol limits; default auto reads PMAX/VMAX/TMAX from registers",
    )
    parser.add_argument(
        "-y",
        "--yes",
        action="store_true",
        help="skip the interactive motion-safety confirmation",
    )


def _validate_motor_id(motor_id: int) -> None:
    if motor_id < 0 or motor_id + RECV_ID_OFFSET > 0x7FF:
        raise ValueError("--id must fit an 11-bit CAN ID with recv_id = id + 0x10")


def _make_sweep_device(
    api: Any, args: argparse.Namespace, control_mode: Any, callback_mode: Any
) -> Any:
    _validate_motor_id(args.motor_id)
    configure_interface(api, args)
    device = api.DamiaoCAN(args.interface, args.fd)
    if args.motor_type == "auto":
        identity = device.probe_motor_identity(args.motor_id, 5_000)
        if not identity.responded:
            raise ValueError(
                "motor auto-detection got no register response; pass --motor-type to override"
            )
        if identity.protocol_limits is None:
            raise ValueError(
                f"motor register limits are unavailable: {identity.reason}; "
                "pass --motor-type to use built-in fallback limits"
            )
        limits = identity.protocol_limits
        print(
            f"auto limits {identity.protocol_family}: "
            f"PMAX={limits.pMax:g} VMAX={limits.vMax:g} TMAX={limits.tMax:g} "
            f"(sw={identity.sw_version_ascii or '-'})"
        )
        device.init_motors_with_limits(
            [limits],
            [args.motor_id],
            [args.motor_id + RECV_ID_OFFSET],
            [control_mode],
        )
    else:
        canonical_motor_type = _MOTOR_TYPE_ALIASES.get(args.motor_type, args.motor_type)
        motor_type = getattr(api.MotorType, canonical_motor_type)
        device.init_motors(
            [args.motor_id],
            [args.motor_id + RECV_ID_OFFSET],
            [motor_type],
            [control_mode],
        )
    device.set_callback_mode_all(callback_mode)
    return device


def _position_expected_duration_s(args: argparse.Namespace) -> float:
    if args.points <= 1:
        return 0.0
    ratio = args.stop_hz / args.start_hz
    cycles = args.settling_cycles + args.measure_cycles
    total = 0.0
    for index in range(args.points):
        fraction = index / (args.points - 1)
        frequency = args.start_hz * ratio**fraction
        total += cycles / frequency
    return total


def _confirm_position(args: argparse.Namespace, center_rad: float) -> bool:
    if args.yes:
        return True
    if not sys.stdin.isatty():
        raise ValueError(
            "position sweep requires interactive confirmation; pass --yes to run non-interactively"
        )

    duration = _position_expected_duration_s(args)
    print(
        "WARNING: this command enables the motor and injects POS_VEL position sinusoids.\n"
        f"  motor ID: 0x{args.motor_id:x}\n"
        f"  center: {center_rad:.6g} rad\n"
        f"  frequency: {args.start_hz:g} -> {args.stop_hz:g} Hz ({args.points} log-spaced points)\n"
        f"  max excursion: +/-{args.amplitude_rad:g} rad (reduced automatically at high frequency)\n"
        f"  POS_VEL velocity ceiling: {args.velocity_limit_rad_s:g} rad/s\n"
        f"  wait/update slot: {args.wait_us} us\n"
        f"  cycles/point: {args.settling_cycles} settle + {args.measure_cycles} measure\n"
        f"  nominal experiment time: about {duration:.1f} s\n"
        "Make sure the mechanism has clearance and an emergency stop is available."
    )
    return input("Type 'yes' to continue: ").strip().lower() == "yes"


def _read_center_position(device: Any, wait_us: int) -> float:
    # A fresh state frame avoids starting a position sweep around a stale/default
    # host-side Motor object. Retry a few times because the same CLI wait policy is
    # intentionally used here as in drop-test.
    for _ in range(3):
        device.flush_rx()
        device.refresh_one(0)
        result = device.recv_all(wait_us)
        if result.ok:
            return float(device.get_motor(0).get_position())
    raise RuntimeError(
        f"motor did not return a fresh state within 3 attempts at --wait-us {wait_us}"
    )


def _write_position_raw_csv(path: Path, result: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "frequency_index",
                "scheduled_time_s",
                "command_time_s",
                "frequency_hz",
                "phase_rad",
                "command_amplitude_rad",
                "command_position_rad",
                "command_velocity_limit_rad_s",
                "measurement",
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
                    sample.frequency_index,
                    sample.scheduled_time_s,
                    sample.command_time_s,
                    sample.frequency_hz,
                    sample.phase_rad,
                    sample.command_amplitude_rad,
                    sample.command_position_rad,
                    feedback.command_velocity_limit,
                    int(sample.measurement),
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


def _solve_3x3(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    augmented = [row[:] + [value] for row, value in zip(matrix, rhs)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise ValueError("frequency point does not contain enough independent samples")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        for item in range(column, 4):
            augmented[column][item] /= scale
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            for item in range(column, 4):
                augmented[row][item] -= factor * augmented[column][item]
    return [augmented[row][3] for row in range(3)]


def _fit_sine(samples: Iterable[Any]) -> tuple[float, float, int]:
    # y = c + a*sin(phi) + b*cos(phi)
    n = 0
    s_sin = s_cos = s_sin2 = s_cos2 = s_sincos = 0.0
    s_y = s_ysin = s_ycos = 0.0
    for sample in samples:
        if not sample.measurement or not sample.feedback.valid:
            continue
        y = float(sample.feedback.position)
        if not math.isfinite(y):
            continue
        sine = math.sin(float(sample.phase_rad))
        cosine = math.cos(float(sample.phase_rad))
        n += 1
        s_sin += sine
        s_cos += cosine
        s_sin2 += sine * sine
        s_cos2 += cosine * cosine
        s_sincos += sine * cosine
        s_y += y
        s_ysin += y * sine
        s_ycos += y * cosine

    if n < 6:
        raise ValueError("too few valid measurement samples")

    _, a, b = _solve_3x3(
        [
            [float(n), s_sin, s_cos],
            [s_sin, s_sin2, s_sincos],
            [s_cos, s_sincos, s_cos2],
        ],
        [s_y, s_ysin, s_ycos],
    )
    return math.hypot(a, b), math.degrees(math.atan2(b, a)), n


def _unwrap_phase(previous: float | None, current: float) -> float:
    if previous is None:
        return current
    while current - previous > 180.0:
        current -= 360.0
    while current - previous < -180.0:
        current += 360.0
    return current


def _estimate_position_response(result: Any) -> list[dict[str, float | int]]:
    grouped: dict[int, list[Any]] = defaultdict(list)
    for sample in result.samples:
        grouped[int(sample.frequency_index)].append(sample)

    rows: list[dict[str, float | int]] = []
    previous_phase: float | None = None
    for index in sorted(grouped):
        samples = grouped[index]
        if not samples:
            continue
        command_amplitude = float(samples[0].command_amplitude_rad)
        if command_amplitude <= 0.0:
            continue
        try:
            response_amplitude, phase_deg, valid_samples = _fit_sine(samples)
        except ValueError:
            continue
        gain = response_amplitude / command_amplitude
        if gain <= 0.0 or not math.isfinite(gain):
            continue
        phase_deg = _unwrap_phase(previous_phase, phase_deg)
        previous_phase = phase_deg
        rows.append(
            {
                "frequency_hz": float(samples[0].frequency_hz),
                "command_amplitude_rad": command_amplitude,
                "response_amplitude_rad": response_amplitude,
                "gain": gain,
                "gain_db": 20.0 * math.log10(gain),
                "phase_deg": phase_deg,
                "valid_measure_samples": valid_samples,
            }
        )

    if rows:
        baseline_db = float(rows[0]["gain_db"])
        for row in rows:
            row["normalized_gain_db"] = float(row["gain_db"]) - baseline_db
    return rows


def _find_cutoff_hz(rows: list[dict[str, float | int]]) -> float | None:
    for previous, current in zip(rows, rows[1:]):
        y0 = float(previous["normalized_gain_db"])
        y1 = float(current["normalized_gain_db"])
        if y0 > -3.0 and y1 <= -3.0:
            f0 = float(previous["frequency_hz"])
            f1 = float(current["frequency_hz"])
            if y1 == y0:
                return f1
            fraction = (-3.0 - y0) / (y1 - y0)
            return math.exp(math.log(f0) + fraction * (math.log(f1) - math.log(f0)))
    return None


def _summary_path(raw_path: Path) -> Path:
    return raw_path.with_name(f"{raw_path.stem}_bode{raw_path.suffix or '.csv'}")


def _write_position_summary_csv(path: Path, rows: list[dict[str, float | int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "frequency_hz",
        "command_amplitude_rad",
        "response_amplitude_rad",
        "gain",
        "gain_db",
        "normalized_gain_db",
        "phase_deg",
        "valid_measure_samples",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _cmd_position(args: argparse.Namespace) -> int:
    if args.stop_hz <= args.start_hz:
        raise ValueError("--stop-hz must be greater than --start-hz")
    nominal_rate_hz = 1_000_000.0 / args.wait_us
    if nominal_rate_hz < 20.0 * args.stop_hz:
        raise ValueError(
            "--wait-us is too long for --stop-hz; require at least 20 command slots per top-frequency cycle"
        )

    from .common import load_api

    api = load_api()
    device = _make_sweep_device(api, args, api.ControlMode.POS_VEL, api.CallbackMode.STATE)
    center_position = _read_center_position(device, args.wait_us)

    if not _confirm_position(args, center_position):
        print("Sweep cancelled.")
        return 2

    config = api.PositionSweepConfig()
    config.center_position_rad = center_position
    config.start_hz = args.start_hz
    config.stop_hz = args.stop_hz
    config.amplitude_rad = args.amplitude_rad
    config.velocity_limit_rad_s = args.velocity_limit_rad_s
    config.wait_us = args.wait_us
    config.points = args.points
    config.settling_cycles = args.settling_cycles
    config.measure_cycles = args.measure_cycles

    output = Path(args.output)
    enabled = False
    try:
        device.flush_rx()
        device.enable_all()
        enabled = True
        result = api.run_position_sinestream(device, 0, config)
    finally:
        if enabled:
            try:
                device.disable_all()
            except RuntimeError:
                pass

    _write_position_raw_csv(output, result)
    rows = _estimate_position_response(result)
    bode_output = _summary_path(output)
    _write_position_summary_csv(bode_output, rows)
    cutoff_hz = _find_cutoff_hz(rows)

    print(
        f"position sweep complete: samples={len(result.samples)} valid={result.valid_samples} "
        f"dropped={result.dropped_samples} valid_ratio={result.valid_ratio:.3f} "
        f"elapsed={result.elapsed_s:.3f}s"
    )
    print(f"raw CSV: {output}")
    print(f"frequency response CSV: {bode_output}")
    if cutoff_hz is None:
        print(
            f"-3 dB cutoff not observed between {args.start_hz:g} and {args.stop_hz:g} Hz"
        )
    else:
        print(f"estimated POS_VEL command-tracking -3 dB cutoff: {cutoff_hz:.3g} Hz")
    return 0 if result.ok and rows else 2


def _write_torque_raw_csv(path: Path, result: Any) -> None:
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


def _confirm_torque(args: argparse.Namespace) -> bool:
    if args.yes:
        return True
    if not sys.stdin.isatty():
        raise ValueError(
            "torque sweep requires interactive confirmation; pass --yes to run non-interactively"
        )
    print(
        "WARNING: this command enables the motor and injects a MIT torque chirp.\n"
        f"  motor ID: 0x{args.motor_id:x}\n"
        f"  frequency: {args.start_hz:g} -> {args.stop_hz:g} Hz\n"
        f"  torque: bias {args.bias_nm:g} Nm, amplitude {args.amplitude_nm:g} Nm\n"
        f"  duration: {args.duration_s:g} s at {args.sample_rate_hz:g} samples/s\n"
        "Make sure the mechanism has clearance and an emergency stop is available."
    )
    return input("Type 'yes' to continue: ").strip().lower() == "yes"


def _cmd_torque(args: argparse.Namespace) -> int:
    if args.sample_rate_hz <= 2.0 * max(args.start_hz, args.stop_hz):
        raise ValueError("--sample-rate-hz must exceed twice the highest sweep frequency")
    if abs(args.bias_nm) + args.amplitude_nm <= 0.0:
        raise ValueError("torque excitation must be non-zero")
    if not _confirm_torque(args):
        print("Sweep cancelled.")
        return 2

    from .common import load_api

    api = load_api()
    device = _make_sweep_device(api, args, api.ControlMode.MIT, api.CallbackMode.IGNORE)

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

    _write_torque_raw_csv(output, result)
    print(
        f"torque sweep complete: samples={len(result.samples)} valid={result.valid_samples} "
        f"dropped={result.dropped_samples} valid_ratio={result.valid_ratio:.3f} "
        f"elapsed={result.elapsed_s:.3f}s"
    )
    print(f"raw CSV: {output}")
    return 0 if result.ok else 2


def _register_position(subparsers: Any) -> None:
    parser = subparsers.add_parser(
        "position",
        help="measure POS_VEL position-command tracking frequency response",
        description=(
            "Run a logarithmically spaced POS_VEL sinestream around the motor's current position.\n"
            "Each frequency has settling and measurement cycles. The high-frequency position\n"
            "excursion is automatically reduced to stay below the configured velocity ceiling.\n"
            "The command saves raw samples and a gain/phase CSV, and estimates the first -3 dB cutoff."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Default position sweep:
  # 1 -> 100 Hz, max +/-0.05 rad, v_des=10 rad/s, wait-us=500
  python -m damiao_can sweep position -i can0 --id 0x01

Lower-motion example:
  python -m damiao_can sweep position -i can0 --id 0x01 \\
      --amplitude-rad 0.03 --stop-hz 50 --output joint1.csv
""",
    )
    _add_single_motor_options(parser)
    parser.add_argument(
        "--start-hz",
        type=_positive_float,
        default=DEFAULT_POSITION_START_HZ,
        help=f"first test frequency in Hz (default: {DEFAULT_POSITION_START_HZ:g})",
    )
    parser.add_argument(
        "--stop-hz",
        "--end-hz",
        type=_positive_float,
        default=DEFAULT_POSITION_STOP_HZ,
        help=f"last test frequency in Hz (default: {DEFAULT_POSITION_STOP_HZ:g})",
    )
    parser.add_argument(
        "--amplitude-rad",
        type=_positive_float,
        default=DEFAULT_POSITION_AMPLITUDE_RAD,
        metavar="RAD",
        help=(
            "maximum sinusoidal position excursion; reduced at high frequency to respect v_des "
            f"(default: {DEFAULT_POSITION_AMPLITUDE_RAD:g} rad)"
        ),
    )
    parser.add_argument(
        "--velocity-limit-rad-s",
        type=_positive_float,
        default=DEFAULT_POSITION_VELOCITY_LIMIT_RAD_S,
        metavar="RAD_S",
        help=(
            "POS_VEL v_des maximum absolute speed "
            f"(default: {DEFAULT_POSITION_VELOCITY_LIMIT_RAD_S:g} rad/s)"
        ),
    )
    parser.add_argument(
        "--wait-us",
        type=_positive_int,
        default=DEFAULT_POSITION_WAIT_US,
        metavar="US",
        help=(
            "per-command update slot and motor-response wait in microseconds "
            f"(default: {DEFAULT_POSITION_WAIT_US})"
        ),
    )
    parser.add_argument(
        "--points",
        type=_positive_int,
        default=DEFAULT_POSITION_POINTS,
        metavar="N",
        help=f"log-spaced frequency points (default: {DEFAULT_POSITION_POINTS})",
    )
    parser.add_argument(
        "--settling-cycles",
        type=_nonnegative_int,
        default=DEFAULT_POSITION_SETTLING_CYCLES,
        metavar="N",
        help=f"discarded cycles before measurement (default: {DEFAULT_POSITION_SETTLING_CYCLES})",
    )
    parser.add_argument(
        "--measure-cycles",
        type=_positive_int,
        default=DEFAULT_POSITION_MEASURE_CYCLES,
        metavar="N",
        help=f"cycles used for gain/phase fit (default: {DEFAULT_POSITION_MEASURE_CYCLES})",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=DEFAULT_POSITION_OUTPUT,
        metavar="CSV",
        help=f"raw CSV output path (default: {DEFAULT_POSITION_OUTPUT})",
    )
    parser.set_defaults(func=_cmd_position)


def _register_torque(subparsers: Any) -> None:
    parser = subparsers.add_parser(
        "torque",
        help="run the existing MIT feed-forward torque chirp",
        description=(
            "Run a single-motor MIT torque chirp in the native C++ acquisition loop.\n"
            "This mode is for plant/mechanical identification; it is not the default\n"
            "measurement for POS_VEL command-tracking bandwidth."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    _add_single_motor_options(parser)
    parser.add_argument(
        "--start-hz",
        type=_positive_float,
        default=DEFAULT_TORQUE_START_HZ,
    )
    parser.add_argument(
        "--stop-hz",
        "--end-hz",
        type=_positive_float,
        default=DEFAULT_TORQUE_STOP_HZ,
    )
    parser.add_argument(
        "--amplitude-nm",
        "--nm",
        type=_positive_float,
        default=DEFAULT_TORQUE_AMPLITUDE_NM,
        metavar="NM",
    )
    parser.add_argument("--bias-nm", type=float, default=0.0, metavar="NM")
    parser.add_argument(
        "--sample-rate-hz",
        "--rate-hz",
        type=_positive_float,
        default=DEFAULT_TORQUE_SAMPLE_RATE_HZ,
        metavar="HZ",
    )
    parser.add_argument(
        "--duration-s",
        "--duration",
        type=_positive_float,
        default=DEFAULT_TORQUE_DURATION_S,
        metavar="SEC",
    )
    parser.add_argument(
        "--response-timeout-us",
        type=_nonnegative_int,
        default=DEFAULT_TORQUE_RESPONSE_TIMEOUT_US,
        metavar="US",
    )
    parser.add_argument(
        "-o", "--output", default=DEFAULT_TORQUE_OUTPUT, metavar="CSV"
    )
    parser.set_defaults(func=_cmd_torque)


def register(subparsers: Any) -> None:
    parser = subparsers.add_parser(
        "sweep",
        help="measure motor frequency response (position tracking or torque plant excitation)",
        description=(
            "Frequency-response measurements. Use 'sweep position' for POS_VEL closed-loop\n"
            "command tracking and -3 dB cutoff; use 'sweep torque' for MIT plant excitation."
        ),
    )
    modes = parser.add_subparsers(dest="sweep_mode", metavar="MODE", required=True)
    _register_position(modes)
    _register_torque(modes)
