# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Mechanical identification workflow for DAMIAO arm motors.

The CLI follows docs/motor_identification_workflow.md:

1. VEL steady-state friction characterization.
2. MIT torque-only breakaway ramps.
3. Short MIT torque perturbations around VEL operating points.
4. MIT torque stepped-sine FRF acquisition.

Experiment orchestration intentionally lives in Python.  The C++ layer remains a
low-level SocketCAN/control library and provides the synchronous exchange_mit()
primitive used where exact transmit/receive timestamps matter.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

from .common import (
    DEFAULT_BITRATE,
    DEFAULT_DBITRATE,
    RECV_ID_OFFSET,
    configure_interface,
    parse_int,
    parse_rate,
)

DEFAULT_RESPONSE_TIMEOUT_US = 1_000
DEFAULT_PARAMETER_TIMEOUT_US = 100_000
DEFAULT_SAMPLE_RATE_HZ = 200.0
DEFAULT_STAGE1_SETTLE_S = 1.0
DEFAULT_STAGE1_MEASURE_S = 1.0
DEFAULT_STAGE2_RAMP_RATE_NM_S = 0.2
DEFAULT_STAGE2_MOTION_THRESHOLD_RAD_S = 0.02
DEFAULT_STAGE2_CONSECUTIVE_SAMPLES = 5
DEFAULT_STAGE2_ZERO_SETTLE_S = 0.5
DEFAULT_STAGE3_SETTLE_S = 1.0
DEFAULT_STAGE3_BASELINE_S = 0.5
DEFAULT_STAGE3_PULSE_S = 0.15
DEFAULT_STAGE3_RECOVERY_S = 0.5
DEFAULT_STAGE3_STEPS = 6
DEFAULT_STAGE3_TRACKING_RATIO_MIN = 0.85
DEFAULT_STAGE3_SAFETY_MARGIN = 0.8
DEFAULT_STAGE4_START_HZ = 1.0
DEFAULT_STAGE4_STOP_HZ = 50.0
DEFAULT_STAGE4_POINTS = 16
DEFAULT_STAGE4_SETTLING_CYCLES = 2
DEFAULT_STAGE4_MEASURE_CYCLES = 4
DEFAULT_STAGE4_SAMPLE_RATE_HZ = 1_000.0
DEFAULT_T_MOS_LIMIT_C = 70
DEFAULT_T_ROTOR_LIMIT_C = 70
DEFAULT_MAX_CONSECUTIVE_RESPONSE_MISSES = 3

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


class ExperimentAbort(RuntimeError):
    """Raised when a runtime safety boundary is crossed."""


@dataclass(frozen=True)
class _SafetyLimits:
    origin_position_rad: float
    max_position_delta_rad: float
    max_velocity_rad_s: float
    max_t_mos_c: int
    max_t_rotor_c: int
    max_consecutive_response_misses: int


@dataclass
class _ResponseGuard:
    max_consecutive_misses: int
    misses: int = 0

    def observe(self, valid: bool, context: str) -> None:
        if valid:
            self.misses = 0
            return
        self.misses += 1
        if self.misses >= self.max_consecutive_misses:
            raise ExperimentAbort(
                f"{context}: {self.misses} consecutive motor responses were missing"
            )


@dataclass(frozen=True)
class _StateSample:
    time_s: float
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    t_mos_c: int
    t_rotor_c: int
    valid: bool


@dataclass(frozen=True)
class _FitResult:
    amplitude: float
    phase_rad: float
    offset: float
    residual_rms: float


def _positive_float(value: str) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid number {value!r}") from exc
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and greater than zero")
    return result


def _nonnegative_float(value: str) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid number {value!r}") from exc
    if not math.isfinite(result) or result < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and non-negative")
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


def _finite_float(value: str) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid number {value!r}") from exc
    if not math.isfinite(result):
        raise argparse.ArgumentTypeError("value must be finite")
    return result


def _float_list(value: str) -> list[float]:
    items: list[float] = []
    for token in value.split(","):
        token = token.strip()
        if not token:
            continue
        items.append(_finite_float(token))
    if not items:
        raise argparse.ArgumentTypeError("provide at least one comma-separated value")
    return items


def _validate_motor_id(motor_id: int) -> None:
    if motor_id < 0 or motor_id + RECV_ID_OFFSET > 0x7FF:
        raise ValueError("--id must fit an 11-bit CAN ID with recv_id = id + 0x10")


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
        help=(
            "fallback built-in protocol limits; default auto reads PMAX/VMAX/TMAX "
            "from the device"
        ),
    )
    parser.add_argument(
        "--parameter-timeout-us",
        type=_positive_int,
        default=DEFAULT_PARAMETER_TIMEOUT_US,
        metavar="US",
        help="maximum wait per register read during auto detection (default: 100000)",
    )
    parser.add_argument(
        "--response-timeout-us",
        type=_nonnegative_int,
        default=DEFAULT_RESPONSE_TIMEOUT_US,
        metavar="US",
        help="maximum wait for each runtime motor response (default: 1000)",
    )
    parser.add_argument(
        "-y",
        "--yes",
        action="store_true",
        help="skip the interactive motion-safety confirmation",
    )


def _add_safety_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--max-position-delta-rad",
        required=True,
        type=_positive_float,
        metavar="RAD",
        help="abort if position moves this far from the experiment start position",
    )
    parser.add_argument(
        "--max-velocity-rad-s",
        required=True,
        type=_positive_float,
        metavar="RAD_S",
        help="abort if absolute measured joint velocity exceeds this value",
    )
    parser.add_argument(
        "--max-t-mos-c",
        type=_positive_int,
        default=DEFAULT_T_MOS_LIMIT_C,
        metavar="C",
        help=f"abort at this MOS temperature (default: {DEFAULT_T_MOS_LIMIT_C} C)",
    )
    parser.add_argument(
        "--max-t-rotor-c",
        type=_positive_int,
        default=DEFAULT_T_ROTOR_LIMIT_C,
        metavar="C",
        help=f"abort at this rotor temperature (default: {DEFAULT_T_ROTOR_LIMIT_C} C)",
    )
    parser.add_argument(
        "--max-consecutive-response-misses",
        type=_positive_int,
        default=DEFAULT_MAX_CONSECUTIVE_RESPONSE_MISSES,
        metavar="N",
        help=(
            "abort after this many consecutive missing runtime responses "
            f"(default: {DEFAULT_MAX_CONSECUTIVE_RESPONSE_MISSES})"
        ),
    )


def _add_output_dir(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "-o",
        "--output-dir",
        default="identification",
        metavar="DIR",
        help="directory for CSV/JSON outputs (default: identification)",
    )


def _make_device(api: Any, args: argparse.Namespace, control_mode: Any, callback_mode: Any) -> Any:
    _validate_motor_id(args.motor_id)
    configure_interface(api, args)
    device = api.DamiaoCAN(args.interface, args.fd)
    recv_id = args.motor_id + RECV_ID_OFFSET

    if args.motor_type == "auto":
        identity = device.probe_motor_identity(args.motor_id, args.parameter_timeout_us)
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
            f"protocol limits {identity.protocol_family}: "
            f"PMAX={limits.pMax:g} VMAX={limits.vMax:g} TMAX={limits.tMax:g} "
            f"(sw={identity.sw_version_ascii or '-'})"
        )
        device.init_motors_with_limits(
            [limits],
            [args.motor_id],
            [recv_id],
            [control_mode],
        )
    else:
        canonical = _MOTOR_TYPE_ALIASES.get(args.motor_type, args.motor_type)
        motor_type = getattr(api.MotorType, canonical)
        device.init_motors(
            [args.motor_id],
            [recv_id],
            [motor_type],
            [control_mode],
        )

    device.set_callback_mode_all(callback_mode)
    return device


def _fresh_state(device: Any, timeout_us: int, attempts: int = 3) -> _StateSample:
    for _ in range(attempts):
        device.flush_rx()
        device.refresh_one(0)
        result = device.recv_all(timeout_us)
        if result.ok:
            motor = device.get_motor(0)
            return _StateSample(
                time_s=0.0,
                position_rad=float(motor.get_position()),
                velocity_rad_s=float(motor.get_velocity()),
                torque_nm=float(motor.get_torque()),
                t_mos_c=int(motor.get_state_tmos()),
                t_rotor_c=int(motor.get_state_trotor()),
                valid=True,
            )
    raise RuntimeError(
        f"motor did not return a fresh state within {attempts} attempts "
        f"at --response-timeout-us {timeout_us}"
    )


def _safety_from_args(args: argparse.Namespace, origin_position_rad: float) -> _SafetyLimits:
    return _SafetyLimits(
        origin_position_rad=origin_position_rad,
        max_position_delta_rad=args.max_position_delta_rad,
        max_velocity_rad_s=args.max_velocity_rad_s,
        max_t_mos_c=args.max_t_mos_c,
        max_t_rotor_c=args.max_t_rotor_c,
        max_consecutive_response_misses=args.max_consecutive_response_misses,
    )


def _check_safety(
    safety: _SafetyLimits,
    *,
    position_rad: float,
    velocity_rad_s: float,
    t_mos_c: int,
    t_rotor_c: int,
) -> None:
    position_delta = abs(position_rad - safety.origin_position_rad)
    if position_delta > safety.max_position_delta_rad:
        raise ExperimentAbort(
            f"position safety limit exceeded: delta={position_delta:.6g} rad > "
            f"{safety.max_position_delta_rad:.6g} rad"
        )
    if abs(velocity_rad_s) > safety.max_velocity_rad_s:
        raise ExperimentAbort(
            f"velocity safety limit exceeded: |dq|={abs(velocity_rad_s):.6g} rad/s > "
            f"{safety.max_velocity_rad_s:.6g} rad/s"
        )
    if t_mos_c >= safety.max_t_mos_c:
        raise ExperimentAbort(
            f"MOS temperature safety limit reached: {t_mos_c} C >= {safety.max_t_mos_c} C"
        )
    if t_rotor_c >= safety.max_t_rotor_c:
        raise ExperimentAbort(
            f"rotor temperature safety limit reached: {t_rotor_c} C >= "
            f"{safety.max_t_rotor_c} C"
        )


def _confirm(args: argparse.Namespace, title: str, lines: Sequence[str]) -> bool:
    if args.yes:
        return True
    if not sys.stdin.isatty():
        raise ValueError(
            f"{title} requires interactive confirmation; "
            "pass --yes to run non-interactively"
        )

    print(f"WARNING: {title} commands motor motion/torque.")
    print(f"  motor ID: 0x{args.motor_id:x}")
    for line in lines:
        print(f"  {line}")
    print(
        "  hard aborts: "
        f"position delta {args.max_position_delta_rad:g} rad, "
        f"|velocity| {args.max_velocity_rad_s:g} rad/s, "
        f"MOS {args.max_t_mos_c} C, rotor {args.max_t_rotor_c} C, "
        f"{args.max_consecutive_response_misses} consecutive response misses"
    )
    print("Make sure the joint has clearance and a physical emergency stop is available.")
    return input("Type 'yes' to continue: ").strip().lower() == "yes"


def _sleep_until(target: float) -> None:
    remaining = target - time.monotonic()
    if remaining > 0.0:
        time.sleep(remaining)


def _vel_sample(
    api: Any,
    device: Any,
    target_velocity_rad_s: float,
    timeout_us: int,
    start_time: float,
) -> _StateSample:
    device.flush_rx()
    device.vel_control_one(0, api.VelParam(target_velocity_rad_s))
    recv = device.recv_all(timeout_us)
    now = time.monotonic()
    if not recv.ok:
        return _StateSample(now - start_time, math.nan, math.nan, math.nan, 0, 0, False)
    motor = device.get_motor(0)
    return _StateSample(
        time_s=now - start_time,
        position_rad=float(motor.get_position()),
        velocity_rad_s=float(motor.get_velocity()),
        torque_nm=float(motor.get_torque()),
        t_mos_c=int(motor.get_state_tmos()),
        t_rotor_c=int(motor.get_state_trotor()),
        valid=True,
    )


def _run_vel_phase(
    api: Any,
    device: Any,
    target_velocity_rad_s: float,
    duration_s: float,
    sample_rate_hz: float,
    timeout_us: int,
    safety: _SafetyLimits,
    start_time: float,
    *,
    collect: bool,
) -> list[_StateSample]:
    if duration_s <= 0.0:
        return []
    dt = 1.0 / sample_rate_hz
    count = max(1, int(math.ceil(duration_s * sample_rate_hz)))
    phase_start = time.monotonic()
    output: list[_StateSample] = []
    response_guard = _ResponseGuard(safety.max_consecutive_response_misses)
    for index in range(count):
        _sleep_until(phase_start + index * dt)
        sample = _vel_sample(
            api,
            device,
            target_velocity_rad_s,
            timeout_us,
            start_time,
        )
        response_guard.observe(sample.valid, "VEL command/response")
        if sample.valid:
            _check_safety(
                safety,
                position_rad=sample.position_rad,
                velocity_rad_s=sample.velocity_rad_s,
                t_mos_c=sample.t_mos_c,
                t_rotor_c=sample.t_rotor_c,
            )
        if collect:
            output.append(sample)
    return output


def _linear_fit(xs: Sequence[float], ys: Sequence[float]) -> tuple[float, float]:
    if len(xs) != len(ys) or len(xs) < 2:
        raise ValueError("linear fit requires at least two paired samples")
    x_mean = statistics.fmean(xs)
    y_mean = statistics.fmean(ys)
    denominator = sum((x - x_mean) ** 2 for x in xs)
    if denominator <= 0.0:
        raise ValueError("linear fit requires distinct x values")
    slope = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denominator
    intercept = y_mean - slope * x_mean
    return intercept, slope


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")


def _stage1(args: argparse.Namespace) -> int:
    if len(args.velocity_fractions) < 2 or len(set(args.velocity_fractions)) < 2:
        raise ValueError("--velocity-fractions needs at least two distinct values")
    if any(f <= 0.0 or f > 1.0 for f in args.velocity_fractions):
        raise ValueError("--velocity-fractions values must be in (0, 1]")
    if max(args.velocity_fractions) * args.test_velocity_rad_s >= args.max_velocity_rad_s:
        raise ValueError("largest Stage 1 velocity target must be below --max-velocity-rad-s")

    from .common import load_api

    api = load_api()
    device = _make_device(api, args, api.ControlMode.VEL, api.CallbackMode.STATE)
    limits = device.get_motor(0).get_limits()
    if args.test_velocity_rad_s > limits.vMax:
        raise ValueError(
            f"--test-velocity-rad-s {args.test_velocity_rad_s:g} "
            f"exceeds motor VMAX {limits.vMax:g}"
        )

    initial = _fresh_state(device, args.response_timeout_us)
    safety = _safety_from_args(args, initial.position_rad)
    fractions = args.velocity_fractions
    targets = [-args.test_velocity_rad_s * f for f in reversed(fractions)]
    targets += [args.test_velocity_rad_s * f for f in fractions]

    if not _confirm(
        args,
        "Stage 1 friction characterization",
        [
            f"VEL targets: {', '.join(f'{v:g}' for v in targets)} rad/s",
            f"per target: {args.settle_s:g}s settle + {args.measure_s:g}s measure",
            f"sample rate: {args.sample_rate_hz:g} Hz",
        ],
    ):
        print("Stage 1 cancelled.")
        return 2

    output_dir = Path(args.output_dir)
    raw_path = output_dir / "stage1_raw.csv"
    summary_path = output_dir / "steady_torque_vs_velocity.csv"
    json_path = output_dir / "stage1_summary.json"
    output_dir.mkdir(parents=True, exist_ok=True)

    raw_rows: list[dict[str, Any]] = []
    point_rows: list[dict[str, Any]] = []
    start_time = time.monotonic()
    enabled = False
    previous_sign = 0
    try:
        device.flush_rx()
        device.enable_all()
        enabled = True

        for point_index, target in enumerate(targets):
            sign = 1 if target > 0 else -1
            if previous_sign and sign != previous_sign:
                _run_vel_phase(
                    api,
                    device,
                    0.0,
                    0.5,
                    args.sample_rate_hz,
                    args.response_timeout_us,
                    safety,
                    start_time,
                    collect=False,
                )
            previous_sign = sign

            _run_vel_phase(
                api,
                device,
                target,
                args.settle_s,
                args.sample_rate_hz,
                args.response_timeout_us,
                safety,
                start_time,
                collect=False,
            )
            samples = _run_vel_phase(
                api,
                device,
                target,
                args.measure_s,
                args.sample_rate_hz,
                args.response_timeout_us,
                safety,
                start_time,
                collect=True,
            )
            valid = [sample for sample in samples if sample.valid]
            if len(valid) < 2:
                raise RuntimeError(
                    f"too few valid steady-state samples at target {target:g} rad/s"
                )

            velocities = [sample.velocity_rad_s for sample in valid]
            torques = [sample.torque_nm for sample in valid]
            torque_mean = statistics.fmean(torques)
            torque_variance = statistics.pvariance(torques)
            torque_rms = math.sqrt(statistics.fmean(tau * tau for tau in torques))
            point_rows.append(
                {
                    "target_velocity_rad_s": target,
                    "mean_velocity_rad_s": statistics.fmean(velocities),
                    "mean_torque_nm": torque_mean,
                    "torque_rms_nm": torque_rms,
                    "torque_variance_nm2": torque_variance,
                    "samples": len(valid),
                    "max_t_mos_c": max(sample.t_mos_c for sample in valid),
                    "max_t_rotor_c": max(sample.t_rotor_c for sample in valid),
                }
            )
            for sample in samples:
                raw_rows.append(
                    {
                        "point_index": point_index,
                        "time_s": sample.time_s,
                        "target_velocity_rad_s": target,
                        "position_rad": sample.position_rad,
                        "velocity_rad_s": sample.velocity_rad_s,
                        "feedback_torque_nm": sample.torque_nm,
                        "t_mos_c": sample.t_mos_c,
                        "t_rotor_c": sample.t_rotor_c,
                        "valid": int(sample.valid),
                    }
                )
    finally:
        if enabled:
            try:
                device.vel_control_one(0, api.VelParam(0.0))
            except RuntimeError:
                pass
            try:
                device.disable_all()
            except RuntimeError:
                pass

    positive = [row for row in point_rows if row["mean_velocity_rad_s"] > 0.0]
    negative = [row for row in point_rows if row["mean_velocity_rad_s"] < 0.0]
    pos_intercept, pos_b = _linear_fit(
        [float(row["mean_velocity_rad_s"]) for row in positive],
        [float(row["mean_torque_nm"]) for row in positive],
    )
    neg_intercept, neg_b = _linear_fit(
        [float(row["mean_velocity_rad_s"]) for row in negative],
        [float(row["mean_torque_nm"]) for row in negative],
    )

    by_target_mean = {
        float(row["target_velocity_rad_s"]): float(row["mean_torque_nm"])
        for row in point_rows
    }
    noise_residuals = [
        float(row["feedback_torque_nm"]) - by_target_mean[float(row["target_velocity_rad_s"])]
        for row in raw_rows
        if row["valid"]
    ]
    torque_noise_rms = math.sqrt(
        statistics.fmean(value * value for value in noise_residuals)
    )

    with raw_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(raw_rows[0]))
        writer.writeheader()
        writer.writerows(raw_rows)
    with summary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(point_rows[0]))
        writer.writeheader()
        writer.writerows(point_rows)

    summary = {
        "coulomb_friction_positive_nm": pos_intercept,
        "coulomb_friction_negative_nm": -neg_intercept,
        "viscous_friction_positive_nms_rad": pos_b,
        "viscous_friction_negative_nms_rad": neg_b,
        "torque_noise_rms_nm": torque_noise_rms,
        "steady_torque_vs_velocity_csv": str(summary_path),
        "raw_csv": str(raw_path),
    }
    _write_json(json_path, summary)

    print(
        "Stage 1 complete: "
        f"tau_c+={pos_intercept:.6g} Nm, tau_c-={-neg_intercept:.6g} Nm, "
        f"B+={pos_b:.6g}, B-={neg_b:.6g} Nms/rad"
    )
    print(f"steady-state CSV: {summary_path}")
    print(f"summary JSON: {json_path}")
    return 0


def _mit_zero(api: Any, device: Any) -> None:
    device.mit_control_one(0, api.MITParam(0.0, 0.0, 0.0, 0.0, 0.0))


def _stage2_direction(
    api: Any,
    device: Any,
    args: argparse.Namespace,
    direction: int,
    safety: _SafetyLimits,
    experiment_start: float,
    rows: list[dict[str, Any]],
) -> dict[str, float]:
    dt = 1.0 / args.sample_rate_hz
    zero_start = time.monotonic()
    zero_count = max(1, int(math.ceil(args.zero_settle_s * args.sample_rate_hz)))
    last_zero_velocity: float | None = None
    zero_response_guard = _ResponseGuard(safety.max_consecutive_response_misses)
    for index in range(zero_count):
        _sleep_until(zero_start + index * dt)
        feedback = device.exchange_mit(
            0,
            api.MITParam(0.0, 0.0, 0.0, 0.0, 0.0),
            args.response_timeout_us,
        )
        zero_response_guard.observe(feedback.valid, "breakaway zero-torque settle")
        if feedback.valid:
            last_zero_velocity = float(feedback.velocity)
            _check_safety(
                safety,
                position_rad=float(feedback.position),
                velocity_rad_s=float(feedback.velocity),
                t_mos_c=int(feedback.t_mos),
                t_rotor_c=int(feedback.t_rotor),
            )
    if last_zero_velocity is None:
        raise RuntimeError("no valid feedback while settling at zero torque")
    if abs(last_zero_velocity) >= args.motion_threshold_rad_s:
        raise RuntimeError(
            f"motor is still moving at {last_zero_velocity:.6g} rad/s before breakaway ramp"
        )

    ramp_start = time.monotonic()
    consecutive = 0
    onset_candidate: dict[str, float] | None = None
    response_guard = _ResponseGuard(safety.max_consecutive_response_misses)
    sample_index = 0
    while True:
        _sleep_until(ramp_start + sample_index * dt)
        elapsed = time.monotonic() - ramp_start
        magnitude = min(args.max_torque_nm, args.ramp_rate_nm_s * elapsed)
        command_tau = direction * magnitude
        feedback = device.exchange_mit(
            0,
            api.MITParam(0.0, 0.0, 0.0, 0.0, command_tau),
            args.response_timeout_us,
        )
        row = {
            "direction": "positive" if direction > 0 else "negative",
            "time_s": time.monotonic() - experiment_start,
            "command_torque_nm": command_tau,
            "feedback_torque_nm": float(feedback.torque) if feedback.valid else math.nan,
            "position_rad": float(feedback.position) if feedback.valid else math.nan,
            "velocity_rad_s": float(feedback.velocity) if feedback.valid else math.nan,
            "tx_timestamp_ns": int(feedback.tx_timestamp_ns),
            "rx_timestamp_ns": int(feedback.rx_timestamp_ns),
            "t_mos_c": int(feedback.t_mos),
            "t_rotor_c": int(feedback.t_rotor),
            "valid": int(feedback.valid),
        }
        rows.append(row)
        response_guard.observe(feedback.valid, "breakaway torque ramp")

        if feedback.valid:
            _check_safety(
                safety,
                position_rad=float(feedback.position),
                velocity_rad_s=float(feedback.velocity),
                t_mos_c=int(feedback.t_mos),
                t_rotor_c=int(feedback.t_rotor),
            )
            if abs(float(feedback.velocity)) >= args.motion_threshold_rad_s:
                if consecutive == 0:
                    onset_candidate = {
                        "torque_nm": abs(command_tau),
                        "position_rad": float(feedback.position),
                        "velocity_rad_s": float(feedback.velocity),
                    }
                consecutive += 1
            else:
                consecutive = 0
                onset_candidate = None
            if consecutive >= args.consecutive_samples:
                if onset_candidate is None:
                    raise RuntimeError("internal breakaway onset tracking error")
                return onset_candidate

        if magnitude >= args.max_torque_nm:
            raise RuntimeError(
                f"no reliable {'positive' if direction > 0 else 'negative'} breakaway "
                f"detected before --max-torque-nm {args.max_torque_nm:g}"
            )
        sample_index += 1


def _stage2(args: argparse.Namespace) -> int:
    from .common import load_api

    api = load_api()
    device = _make_device(api, args, api.ControlMode.MIT, api.CallbackMode.IGNORE)
    limits = device.get_motor(0).get_limits()
    if args.max_torque_nm >= limits.tMax:
        raise ValueError(
            f"--max-torque-nm must be below protocol TMAX {limits.tMax:g}; "
            "TMAX is not a safe sustained-torque target"
        )

    device.set_callback_mode_all(api.CallbackMode.STATE)
    initial = _fresh_state(device, args.response_timeout_us)
    device.set_callback_mode_all(api.CallbackMode.IGNORE)
    safety = _safety_from_args(args, initial.position_rad)
    worst_case_s = args.max_torque_nm / args.ramp_rate_nm_s
    if not _confirm(
        args,
        "Stage 2 breakaway test",
        [
            f"MIT torque-only ramps: 0 -> +/-{args.max_torque_nm:g} Nm",
            f"ramp rate: {args.ramp_rate_nm_s:g} Nm/s (up to {worst_case_s:.1f}s per direction)",
            f"motion threshold: {args.motion_threshold_rad_s:g} rad/s for "
            f"{args.consecutive_samples} samples",
        ],
    ):
        print("Stage 2 cancelled.")
        return 2

    output_dir = Path(args.output_dir)
    csv_path = output_dir / "breakaway_test.csv"
    json_path = output_dir / "stage2_summary.json"
    output_dir.mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, Any]] = []
    start = time.monotonic()
    enabled = False
    try:
        device.flush_rx()
        device.enable_all()
        enabled = True
        positive = _stage2_direction(api, device, args, 1, safety, start, rows)
        _mit_zero(api, device)
        time.sleep(args.zero_settle_s)
        negative = _stage2_direction(api, device, args, -1, safety, start, rows)
    finally:
        if enabled:
            try:
                _mit_zero(api, device)
            except RuntimeError:
                pass
            try:
                device.disable_all()
            except RuntimeError:
                pass

    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "breakaway_torque_positive_nm": positive["torque_nm"],
        "breakaway_torque_negative_nm": negative["torque_nm"],
        "breakaway_position_positive_rad": positive["position_rad"],
        "breakaway_position_negative_rad": negative["position_rad"],
        "breakaway_velocity_positive_rad_s": positive["velocity_rad_s"],
        "breakaway_velocity_negative_rad_s": negative["velocity_rad_s"],
        "breakaway_test_csv": str(csv_path),
    }
    _write_json(json_path, summary)
    print(
        "Stage 2 complete: "
        f"breakaway +={positive['torque_nm']:.6g} Nm, "
        f"-={negative['torque_nm']:.6g} Nm"
    )
    print(f"raw CSV: {csv_path}")
    print(f"summary JSON: {json_path}")
    return 0


def _mean_valid(samples: Iterable[_StateSample], attr: str) -> float:
    values = [float(getattr(sample, attr)) for sample in samples if sample.valid]
    if not values:
        raise RuntimeError("no valid samples available")
    return statistics.fmean(values)


def _stage3(args: argparse.Namespace) -> int:
    if not (0.0 < args.tracking_ratio_min <= 1.0):
        raise ValueError("--tracking-ratio-min must be in (0, 1]")
    if not (0.0 < args.safety_margin <= 1.0):
        raise ValueError("--safety-margin must be in (0, 1]")
    if any(abs(v) >= args.max_velocity_rad_s for v in args.velocities_rad_s):
        raise ValueError("every --velocities-rad-s value must be below --max-velocity-rad-s")

    from .common import load_api

    api = load_api()
    device = _make_device(api, args, api.ControlMode.VEL, api.CallbackMode.STATE)
    limits = device.get_motor(0).get_limits()
    if args.max_perturbation_nm >= limits.tMax:
        raise ValueError(
            f"--max-perturbation-nm must be below protocol TMAX {limits.tMax:g}"
        )
    if any(abs(v) > limits.vMax for v in args.velocities_rad_s):
        raise ValueError(f"an operating velocity exceeds protocol VMAX {limits.vMax:g}")

    initial = _fresh_state(device, args.response_timeout_us)
    safety = _safety_from_args(args, initial.position_rad)
    if not _confirm(
        args,
        "Stage 3 torque capability envelope",
        [
            f"VEL operating points: {', '.join(f'{v:g}' for v in args.velocities_rad_s)} rad/s",
            f"MIT perturbations: {args.steps} levels up to "
            f"{args.max_perturbation_nm:g} Nm in both directions",
            f"pulse {args.pulse_s:g}s; tracking-ratio threshold {args.tracking_ratio_min:g}",
            "this is a linearity test, not a stall/locked-rotor test",
        ],
    ):
        print("Stage 3 cancelled.")
        return 2

    output_dir = Path(args.output_dir)
    csv_path = output_dir / "torque_linear_limit_vs_velocity.csv"
    raw_path = output_dir / "stage3_raw.csv"
    json_path = output_dir / "stage3_summary.json"
    output_dir.mkdir(parents=True, exist_ok=True)

    summary_rows: list[dict[str, Any]] = []
    raw_rows: list[dict[str, Any]] = []
    start = time.monotonic()
    enabled = False
    try:
        device.flush_rx()
        device.enable_all()
        enabled = True
        for velocity in args.velocities_rad_s:
            device.set_control_mode_one(0, api.ControlMode.VEL)
            device.set_callback_mode_all(api.CallbackMode.STATE)
            _run_vel_phase(
                api,
                device,
                velocity,
                args.settle_s,
                args.sample_rate_hz,
                args.response_timeout_us,
                safety,
                start,
                collect=False,
            )
            baseline_samples = _run_vel_phase(
                api,
                device,
                velocity,
                args.baseline_s,
                args.sample_rate_hz,
                args.response_timeout_us,
                safety,
                start,
                collect=True,
            )
            baseline_velocity = _mean_valid(baseline_samples, "velocity_rad_s")
            baseline_torque = _mean_valid(baseline_samples, "torque_nm")

            for direction in (-1, 1):
                last_linear_amp = 0.0
                first_nonlinear_amp: float | None = None
                observed_limit = False
                boundary_reason = "configured_max"
                for step in range(1, args.steps + 1):
                    amplitude = args.max_perturbation_nm * step / args.steps
                    command_tau = baseline_torque + direction * amplitude
                    if abs(command_tau) >= limits.tMax:
                        boundary_reason = "protocol_tmax"
                        break

                    device.set_control_mode_one(0, api.ControlMode.MIT)
                    device.set_callback_mode_all(api.CallbackMode.IGNORE)
                    device.flush_rx()
                    pulse_start = time.monotonic()
                    dt = 1.0 / args.sample_rate_hz
                    count = max(1, int(math.ceil(args.pulse_s * args.sample_rate_hz)))
                    pulse_feedback: list[float] = []
                    valid_count = 0
                    response_guard = _ResponseGuard(
                        safety.max_consecutive_response_misses
                    )
                    try:
                        for index in range(count):
                            _sleep_until(pulse_start + index * dt)
                            feedback = device.exchange_mit(
                                0,
                                api.MITParam(0.0, 0.0, 0.0, 0.0, command_tau),
                                args.response_timeout_us,
                            )
                            raw_rows.append(
                                {
                                    "time_s": time.monotonic() - start,
                                    "operating_velocity_rad_s": velocity,
                                    "baseline_torque_nm": baseline_torque,
                                    "perturbation_direction": direction,
                                    "perturbation_nm": amplitude,
                                    "command_torque_nm": command_tau,
                                    "feedback_torque_nm": float(feedback.torque)
                                    if feedback.valid
                                    else math.nan,
                                    "position_rad": float(feedback.position)
                                    if feedback.valid
                                    else math.nan,
                                    "velocity_rad_s": float(feedback.velocity)
                                    if feedback.valid
                                    else math.nan,
                                    "tx_timestamp_ns": int(feedback.tx_timestamp_ns),
                                    "rx_timestamp_ns": int(feedback.rx_timestamp_ns),
                                    "t_mos_c": int(feedback.t_mos),
                                    "t_rotor_c": int(feedback.t_rotor),
                                    "valid": int(feedback.valid),
                                }
                            )
                            response_guard.observe(
                                feedback.valid, "Stage 3 MIT perturbation"
                            )
                            if feedback.valid:
                                valid_count += 1
                                pulse_feedback.append(float(feedback.torque))
                                _check_safety(
                                    safety,
                                    position_rad=float(feedback.position),
                                    velocity_rad_s=float(feedback.velocity),
                                    t_mos_c=int(feedback.t_mos),
                                    t_rotor_c=int(feedback.t_rotor),
                                )
                    finally:
                        try:
                            _mit_zero(api, device)
                        except RuntimeError:
                            pass

                    if valid_count == 0:
                        raise RuntimeError(
                            f"no valid MIT feedback at {velocity:g} rad/s, "
                            f"perturbation {amplitude:g} Nm"
                        )
                    measured_delta = statistics.fmean(pulse_feedback) - baseline_torque
                    tracking_ratio = abs(measured_delta) / amplitude
                    linear = tracking_ratio >= args.tracking_ratio_min
                    if linear:
                        last_linear_amp = amplitude
                    else:
                        first_nonlinear_amp = amplitude
                        observed_limit = True
                        boundary_reason = "tracking_ratio"

                    device.set_control_mode_one(0, api.ControlMode.VEL)
                    device.set_callback_mode_all(api.CallbackMode.STATE)
                    _run_vel_phase(
                        api,
                        device,
                        velocity,
                        args.recovery_s,
                        args.sample_rate_hz,
                        args.response_timeout_us,
                        safety,
                        start,
                        collect=False,
                    )
                    if not linear:
                        break

                recommended = args.safety_margin * last_linear_amp
                summary_rows.append(
                    {
                        "operating_velocity_command_rad_s": velocity,
                        "operating_velocity_measured_rad_s": baseline_velocity,
                        "perturbation_direction": direction,
                        "baseline_torque_nm": baseline_torque,
                        "largest_linear_perturbation_nm": last_linear_amp,
                        "first_nonlinear_perturbation_nm": ""
                        if first_nonlinear_amp is None
                        else first_nonlinear_amp,
                        "recommended_excitation_limit_nm": recommended,
                        "limit_observed": int(observed_limit),
                        "boundary_reason": boundary_reason,
                    }
                )
    finally:
        if enabled:
            try:
                device.set_control_mode_one(0, api.ControlMode.VEL)
                device.vel_control_one(0, api.VelParam(0.0))
            except RuntimeError:
                pass
            try:
                device.disable_all()
            except RuntimeError:
                pass

    if raw_rows:
        with raw_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(raw_rows[0]))
            writer.writeheader()
            writer.writerows(raw_rows)
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary_rows[0]))
        writer.writeheader()
        writer.writerows(summary_rows)

    observed_limits = [
        float(row["recommended_excitation_limit_nm"])
        for row in summary_rows
        if row["limit_observed"] and float(row["recommended_excitation_limit_nm"]) > 0.0
    ]
    payload = {
        "torque_linear_limit_vs_velocity_csv": str(csv_path),
        "raw_csv": str(raw_path),
        "minimum_observed_recommended_excitation_limit_nm": min(observed_limits)
        if observed_limits
        else None,
        "note": (
            "Rows with limit_observed=0 are censored: the configured maximum perturbation "
            "was still linear, so the true upper boundary was not found."
        ),
    }
    _write_json(json_path, payload)
    print(f"Stage 3 complete: envelope CSV: {csv_path}")
    if observed_limits:
        print(
            "minimum observed recommended excitation upper bound: "
            f"{min(observed_limits):.6g} Nm"
        )
    else:
        print("no nonlinear boundary was observed within the configured perturbation range")
    print(f"summary JSON: {json_path}")
    return 0


def _solve_3x3(matrix: list[list[float]], vector: list[float]) -> tuple[float, float, float]:
    augmented = [row[:] + [value] for row, value in zip(matrix, vector)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-15:
            raise ValueError("singular sinusoid-fit matrix")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        for j in range(column, 4):
            augmented[column][j] /= divisor
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            for j in range(column, 4):
                augmented[row][j] -= factor * augmented[column][j]
    return augmented[0][3], augmented[1][3], augmented[2][3]


def _fit_sinusoid(phases: Sequence[float], values: Sequence[float]) -> _FitResult:
    if len(phases) != len(values) or len(values) < 6:
        raise ValueError("sinusoid fit requires at least six paired samples")
    sines = [math.sin(phase) for phase in phases]
    cosines = [math.cos(phase) for phase in phases]
    n = float(len(values))
    matrix = [
        [sum(s * s for s in sines), sum(s * c for s, c in zip(sines, cosines)), sum(sines)],
        [sum(s * c for s, c in zip(sines, cosines)), sum(c * c for c in cosines), sum(cosines)],
        [sum(sines), sum(cosines), n],
    ]
    vector = [
        sum(s * y for s, y in zip(sines, values)),
        sum(c * y for c, y in zip(cosines, values)),
        sum(values),
    ]
    sin_coeff, cos_coeff, offset = _solve_3x3(matrix, vector)
    amplitude = math.hypot(sin_coeff, cos_coeff)
    phase = math.atan2(cos_coeff, sin_coeff)
    residuals = [
        y - (sin_coeff * s + cos_coeff * c + offset)
        for y, s, c in zip(values, sines, cosines)
    ]
    residual_rms = math.sqrt(statistics.fmean(r * r for r in residuals))
    return _FitResult(amplitude, phase, offset, residual_rms)


def _wrap_phase(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value <= -math.pi:
        value += 2.0 * math.pi
    return value


def _db(value: float) -> float:
    if value <= 0.0:
        return -math.inf
    return 20.0 * math.log10(value)


def _stage4(args: argparse.Namespace) -> int:
    if args.points < 2:
        raise ValueError("--points must be at least 2")
    if args.stop_hz <= args.start_hz:
        raise ValueError("--stop-hz must be greater than --start-hz")
    if args.amplitude_reduce_at > 1.0:
        raise ValueError("--amplitude-reduce-at must be <= 1")
    if not (0.0 < args.amplitude_reduce_factor < 1.0):
        raise ValueError("--amplitude-reduce-factor must be in (0, 1)")
    if args.sample_rate_hz < 10.0 * args.stop_hz:
        raise ValueError(
            "--sample-rate-hz must provide at least 10 command/response slots "
            "per highest-frequency cycle"
        )
    if not (args.breakaway_nm < args.amplitude_nm < args.linear_limit_nm):
        raise ValueError(
            "require --breakaway-nm < --amplitude-nm < --linear-limit-nm; "
            "Stage 4 has no fixed 0.1 Nm fallback"
        )
    if abs(args.bias_nm) + args.amplitude_nm >= args.linear_limit_nm:
        raise ValueError(
            "abs(--bias-nm) + --amplitude-nm must remain below --linear-limit-nm"
        )

    from .common import load_api

    api = load_api()
    device = _make_device(api, args, api.ControlMode.MIT, api.CallbackMode.IGNORE)
    limits = device.get_motor(0).get_limits()
    if abs(args.bias_nm) + args.amplitude_nm >= limits.tMax:
        raise ValueError(
            f"requested torque reaches protocol TMAX {limits.tMax:g}; reduce bias/amplitude"
        )

    device.set_callback_mode_all(api.CallbackMode.STATE)
    initial = _fresh_state(device, args.response_timeout_us)
    device.set_callback_mode_all(api.CallbackMode.IGNORE)
    safety = _safety_from_args(args, initial.position_rad)
    ratio = args.stop_hz / args.start_hz
    frequencies = [
        args.start_hz * ratio ** (index / (args.points - 1))
        for index in range(args.points)
    ]
    duration = sum(
        (args.settling_cycles + args.measure_cycles) / frequency
        for frequency in frequencies
    )
    if not _confirm(
        args,
        "Stage 4 MIT torque FRF",
        [
            f"stepped-sine: {args.start_hz:g} -> {args.stop_hz:g} Hz "
            f"({args.points} log-spaced points)",
            f"torque bias {args.bias_nm:g} Nm, excitation {args.amplitude_nm:g} Nm",
            f"validated bounds: breakaway {args.breakaway_nm:g} < amplitude "
            f"< linear {args.linear_limit_nm:g} Nm",
            f"cycles/point: {args.settling_cycles} settle + {args.measure_cycles} measure",
            f"sample rate: {args.sample_rate_hz:g} Hz; nominal duration {duration:.1f}s",
        ],
    ):
        print("Stage 4 cancelled.")
        return 2

    output_dir = Path(args.output_dir)
    raw_path = output_dir / "torque_frf_raw.csv"
    frf_path = output_dir / "torque_frf.csv"
    json_path = output_dir / "stage4_summary.json"
    output_dir.mkdir(parents=True, exist_ok=True)

    raw_rows: list[dict[str, Any]] = []
    frf_rows: list[dict[str, Any]] = []
    experiment_start = time.monotonic()
    current_amplitude = args.amplitude_nm
    enabled = False
    try:
        device.flush_rx()
        device.enable_all()
        enabled = True
        for frequency_index, frequency in enumerate(frequencies):
            total_cycles = args.settling_cycles + args.measure_cycles
            point_duration = total_cycles / frequency
            measure_start = args.settling_cycles / frequency
            dt = 1.0 / args.sample_rate_hz
            sample_count = max(1, int(math.floor(point_duration * args.sample_rate_hz)) + 1)
            point_start = time.monotonic()
            point_measure_rows: list[dict[str, Any]] = []
            max_position_delta = 0.0
            max_velocity = 0.0
            response_guard = _ResponseGuard(safety.max_consecutive_response_misses)

            for sample_index in range(sample_count):
                scheduled = sample_index * dt
                _sleep_until(point_start + scheduled)
                phase = 2.0 * math.pi * frequency * scheduled
                command_tau = args.bias_nm + current_amplitude * math.sin(phase)
                feedback = device.exchange_mit(
                    0,
                    api.MITParam(0.0, 0.0, 0.0, 0.0, command_tau),
                    args.response_timeout_us,
                )
                measurement = scheduled + 0.5 * dt >= measure_start
                row = {
                    "time_s": time.monotonic() - experiment_start,
                    "frequency_index": frequency_index,
                    "frequency_hz": frequency,
                    "phase_rad": phase,
                    "measurement": int(measurement),
                    "command_torque_nm": command_tau,
                    "excitation_amplitude_nm": current_amplitude,
                    "feedback_torque_nm": float(feedback.torque)
                    if feedback.valid
                    else math.nan,
                    "position_rad": float(feedback.position) if feedback.valid else math.nan,
                    "velocity_rad_s": float(feedback.velocity)
                    if feedback.valid
                    else math.nan,
                    "tx_timestamp_ns": int(feedback.tx_timestamp_ns),
                    "rx_timestamp_ns": int(feedback.rx_timestamp_ns),
                    "t_mos_c": int(feedback.t_mos),
                    "t_rotor_c": int(feedback.t_rotor),
                    "valid": int(feedback.valid),
                }
                raw_rows.append(row)
                response_guard.observe(feedback.valid, "Stage 4 MIT FRF")
                if feedback.valid:
                    _check_safety(
                        safety,
                        position_rad=float(feedback.position),
                        velocity_rad_s=float(feedback.velocity),
                        t_mos_c=int(feedback.t_mos),
                        t_rotor_c=int(feedback.t_rotor),
                    )
                    max_position_delta = max(
                        max_position_delta,
                        abs(float(feedback.position) - safety.origin_position_rad),
                    )
                    max_velocity = max(max_velocity, abs(float(feedback.velocity)))
                    if measurement:
                        point_measure_rows.append(row)

            if len(point_measure_rows) < 6:
                raise RuntimeError(
                    f"too few valid measurement samples at {frequency:.6g} Hz"
                )
            phases = [float(row["phase_rad"]) for row in point_measure_rows]
            command_fit = _fit_sinusoid(
                phases,
                [float(row["command_torque_nm"]) for row in point_measure_rows],
            )
            feedback_fit = _fit_sinusoid(
                phases,
                [float(row["feedback_torque_nm"]) for row in point_measure_rows],
            )
            position_fit = _fit_sinusoid(
                phases,
                [float(row["position_rad"]) for row in point_measure_rows],
            )
            velocity_fit = _fit_sinusoid(
                phases,
                [float(row["velocity_rad_s"]) for row in point_measure_rows],
            )
            if command_fit.amplitude <= 0.0:
                raise RuntimeError(f"zero fitted command amplitude at {frequency:g} Hz")

            position_gain = position_fit.amplitude / command_fit.amplitude
            velocity_gain = velocity_fit.amplitude / command_fit.amplitude
            feedback_gain = feedback_fit.amplitude / command_fit.amplitude
            frf_rows.append(
                {
                    "frequency_hz": frequency,
                    "excitation_amplitude_nm": current_amplitude,
                    "command_amplitude_nm": command_fit.amplitude,
                    "feedback_torque_amplitude_nm": feedback_fit.amplitude,
                    "feedback_torque_gain": feedback_gain,
                    "feedback_torque_phase_deg": math.degrees(
                        _wrap_phase(feedback_fit.phase_rad - command_fit.phase_rad)
                    ),
                    "position_amplitude_rad": position_fit.amplitude,
                    "position_gain_rad_per_nm": position_gain,
                    "position_gain_db": _db(position_gain),
                    "position_phase_deg": math.degrees(
                        _wrap_phase(position_fit.phase_rad - command_fit.phase_rad)
                    ),
                    "velocity_amplitude_rad_s": velocity_fit.amplitude,
                    "velocity_gain_rad_s_per_nm": velocity_gain,
                    "velocity_gain_db": _db(velocity_gain),
                    "velocity_phase_deg": math.degrees(
                        _wrap_phase(velocity_fit.phase_rad - command_fit.phase_rad)
                    ),
                    "position_fit_residual_rms_rad": position_fit.residual_rms,
                    "velocity_fit_residual_rms_rad_s": velocity_fit.residual_rms,
                    "feedback_torque_fit_residual_rms_nm": feedback_fit.residual_rms,
                    "valid_measure_samples": len(point_measure_rows),
                }
            )

            if args.auto_amplitude:
                utilization = max(
                    max_position_delta / safety.max_position_delta_rad,
                    max_velocity / safety.max_velocity_rad_s,
                )
                if utilization >= args.amplitude_reduce_at:
                    reduced = current_amplitude * args.amplitude_reduce_factor
                    minimum = args.breakaway_nm * 1.05
                    if reduced <= minimum:
                        raise ExperimentAbort(
                            "response approached a safety limit, but reducing excitation further "
                            "would fall back into the measured breakaway deadband"
                        )
                    current_amplitude = reduced
                    print(
                        f"{frequency:g} Hz response used {utilization:.0%} of a motion limit; "
                        f"next excitation reduced to {current_amplitude:.6g} Nm"
                    )
    finally:
        if enabled:
            try:
                _mit_zero(api, device)
            except RuntimeError:
                pass
            try:
                device.disable_all()
            except RuntimeError:
                pass

    with raw_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(raw_rows[0]))
        writer.writeheader()
        writer.writerows(raw_rows)
    with frf_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(frf_rows[0]))
        writer.writeheader()
        writer.writerows(frf_rows)

    payload = {
        "raw_csv": str(raw_path),
        "frf_csv": str(frf_path),
        "breakaway_lower_bound_nm": args.breakaway_nm,
        "linear_upper_bound_nm": args.linear_limit_nm,
        "requested_excitation_amplitude_nm": args.amplitude_nm,
        "final_excitation_amplitude_nm": current_amplitude,
        "note": (
            "FRF acquisition does not rename a -3 dB point as controller omega. "
            "Select omega_usable only after combining this plant FRF with the arm inertia model, "
            "controller C(s), stability margins, flexible resonance, noise, and delay."
        ),
    }
    _write_json(json_path, payload)
    print(f"Stage 4 complete: raw CSV: {raw_path}")
    print(f"FRF CSV: {frf_path}")
    print(f"summary JSON: {json_path}")
    return 0


def _register_stage1(subparsers: Any) -> None:
    parser = subparsers.add_parser(
        "friction",
        aliases=["stage1"],
        help="Stage 1: VEL steady-state friction characterization",
        description=(
            "Measure steady feedback torque at positive/negative VEL operating points and fit "
            "directional Coulomb + viscous friction."
        ),
    )
    _add_single_motor_options(parser)
    _add_safety_options(parser)
    _add_output_dir(parser)
    parser.add_argument(
        "--test-velocity-rad-s",
        required=True,
        type=_positive_float,
        metavar="RAD_S",
        help="V_test used to form the default +/-0.25/0.50/0.75 operating points",
    )
    parser.add_argument(
        "--velocity-fractions",
        type=_float_list,
        default=[0.25, 0.50, 0.75],
        metavar="LIST",
        help="positive comma-separated fractions of V_test (default: 0.25,0.5,0.75)",
    )
    parser.add_argument(
        "--settle-s",
        type=_positive_float,
        default=DEFAULT_STAGE1_SETTLE_S,
        help=f"settling time per velocity point (default: {DEFAULT_STAGE1_SETTLE_S:g}s)",
    )
    parser.add_argument(
        "--measure-s",
        type=_positive_float,
        default=DEFAULT_STAGE1_MEASURE_S,
        help=f"steady-state measurement time per point (default: {DEFAULT_STAGE1_MEASURE_S:g}s)",
    )
    parser.add_argument(
        "--sample-rate-hz",
        type=_positive_float,
        default=DEFAULT_SAMPLE_RATE_HZ,
        help=f"command/response rate (default: {DEFAULT_SAMPLE_RATE_HZ:g} Hz)",
    )
    parser.set_defaults(func=_stage1)


def _register_stage2(subparsers: Any) -> None:
    parser = subparsers.add_parser(
        "breakaway",
        aliases=["stage2"],
        help="Stage 2: MIT torque-only positive/negative breakaway ramps",
    )
    _add_single_motor_options(parser)
    _add_safety_options(parser)
    _add_output_dir(parser)
    parser.add_argument(
        "--max-torque-nm",
        required=True,
        type=_positive_float,
        metavar="NM",
        help="explicit abort ceiling for each breakaway ramp; must be below TMAX",
    )
    parser.add_argument(
        "--ramp-rate-nm-s",
        type=_positive_float,
        default=DEFAULT_STAGE2_RAMP_RATE_NM_S,
        metavar="NM_S",
        help=f"slow torque ramp rate (default: {DEFAULT_STAGE2_RAMP_RATE_NM_S:g} Nm/s)",
    )
    parser.add_argument(
        "--motion-threshold-rad-s",
        type=_positive_float,
        default=DEFAULT_STAGE2_MOTION_THRESHOLD_RAD_S,
        metavar="RAD_S",
        help=(
            "absolute measured velocity required to count as motion "
            f"(default: {DEFAULT_STAGE2_MOTION_THRESHOLD_RAD_S:g} rad/s)"
        ),
    )
    parser.add_argument(
        "--consecutive-samples",
        type=_positive_int,
        default=DEFAULT_STAGE2_CONSECUTIVE_SAMPLES,
        metavar="N",
        help=(
            "consecutive above-threshold samples required for breakaway "
            f"(default: {DEFAULT_STAGE2_CONSECUTIVE_SAMPLES})"
        ),
    )
    parser.add_argument(
        "--zero-settle-s",
        type=_nonnegative_float,
        default=DEFAULT_STAGE2_ZERO_SETTLE_S,
        help=(
            "zero-torque settle before each direction "
            f"(default: {DEFAULT_STAGE2_ZERO_SETTLE_S:g}s)"
        ),
    )
    parser.add_argument(
        "--sample-rate-hz",
        type=_positive_float,
        default=DEFAULT_SAMPLE_RATE_HZ,
        help=f"command/response rate (default: {DEFAULT_SAMPLE_RATE_HZ:g} Hz)",
    )
    parser.set_defaults(func=_stage2)


def _register_stage3(subparsers: Any) -> None:
    parser = subparsers.add_parser(
        "envelope",
        aliases=["stage3"],
        help="Stage 3: torque linearity envelope at representative velocities",
        description=(
            "Establish each velocity in VEL mode, measure baseline torque, then apply short MIT "
            "feed-forward torque perturbations. Stops increasing a direction when measured torque "
            "tracking falls below the configured ratio. This is not a stall-torque test."
        ),
    )
    _add_single_motor_options(parser)
    _add_safety_options(parser)
    _add_output_dir(parser)
    parser.add_argument(
        "--velocities-rad-s",
        required=True,
        type=_float_list,
        metavar="LIST",
        help="comma-separated representative operating velocities",
    )
    parser.add_argument(
        "--max-perturbation-nm",
        required=True,
        type=_positive_float,
        metavar="NM",
        help="largest perturbation to test; explicit experiment ceiling, not TMAX",
    )
    parser.add_argument(
        "--steps",
        type=_positive_int,
        default=DEFAULT_STAGE3_STEPS,
        help=f"perturbation levels from small to max (default: {DEFAULT_STAGE3_STEPS})",
    )
    parser.add_argument(
        "--tracking-ratio-min",
        type=_positive_float,
        default=DEFAULT_STAGE3_TRACKING_RATIO_MIN,
        help=(
            "minimum |feedback torque change| / |command perturbation| considered linear "
            f"(default: {DEFAULT_STAGE3_TRACKING_RATIO_MIN:g})"
        ),
    )
    parser.add_argument(
        "--safety-margin",
        type=_positive_float,
        default=DEFAULT_STAGE3_SAFETY_MARGIN,
        help=(
            "fraction of the largest observed linear perturbation reported as the recommended "
            f"Stage 4 upper bound (default: {DEFAULT_STAGE3_SAFETY_MARGIN:g})"
        ),
    )
    parser.add_argument(
        "--settle-s",
        type=_positive_float,
        default=DEFAULT_STAGE3_SETTLE_S,
        help=f"VEL settle time at each operating point (default: {DEFAULT_STAGE3_SETTLE_S:g}s)",
    )
    parser.add_argument(
        "--baseline-s",
        type=_positive_float,
        default=DEFAULT_STAGE3_BASELINE_S,
        help=f"baseline torque measurement window (default: {DEFAULT_STAGE3_BASELINE_S:g}s)",
    )
    parser.add_argument(
        "--pulse-s",
        type=_positive_float,
        default=DEFAULT_STAGE3_PULSE_S,
        help=f"duration of each MIT perturbation pulse (default: {DEFAULT_STAGE3_PULSE_S:g}s)",
    )
    parser.add_argument(
        "--recovery-s",
        type=_nonnegative_float,
        default=DEFAULT_STAGE3_RECOVERY_S,
        help=f"VEL recovery after each pulse (default: {DEFAULT_STAGE3_RECOVERY_S:g}s)",
    )
    parser.add_argument(
        "--sample-rate-hz",
        type=_positive_float,
        default=DEFAULT_SAMPLE_RATE_HZ,
        help=f"command/response rate (default: {DEFAULT_SAMPLE_RATE_HZ:g} Hz)",
    )
    parser.set_defaults(func=_stage3)


def _register_stage4(subparsers: Any) -> None:
    parser = subparsers.add_parser(
        "frf",
        aliases=["stage4"],
        help="Stage 4: MIT torque stepped-sine command-torque -> joint-motion FRF",
        description=(
            "Run a logarithmically spaced MIT torque sinestream. There is deliberately no fixed "
            "torque-amplitude default: supply the measured breakaway lower bound, linear upper "
            "bound, and an excitation strictly between them."
        ),
    )
    _add_single_motor_options(parser)
    _add_safety_options(parser)
    _add_output_dir(parser)
    parser.add_argument(
        "--breakaway-nm",
        required=True,
        type=_positive_float,
        metavar="NM",
        help="Stage 2 excitation lower bound for the tested direction/load condition",
    )
    parser.add_argument(
        "--linear-limit-nm",
        required=True,
        type=_positive_float,
        metavar="NM",
        help="Stage 3 excitation upper bound after safety margin",
    )
    parser.add_argument(
        "--amplitude-nm",
        required=True,
        type=_positive_float,
        metavar="NM",
        help="sinusoidal torque excitation; must lie strictly between measured bounds",
    )
    parser.add_argument("--bias-nm", type=_finite_float, default=0.0, metavar="NM")
    parser.add_argument(
        "--start-hz",
        type=_positive_float,
        default=DEFAULT_STAGE4_START_HZ,
        help=f"first frequency (default: {DEFAULT_STAGE4_START_HZ:g} Hz)",
    )
    parser.add_argument(
        "--stop-hz",
        type=_positive_float,
        default=DEFAULT_STAGE4_STOP_HZ,
        help=f"last frequency (default: {DEFAULT_STAGE4_STOP_HZ:g} Hz)",
    )
    parser.add_argument(
        "--points",
        type=_positive_int,
        default=DEFAULT_STAGE4_POINTS,
        help=f"log-spaced frequency points (default: {DEFAULT_STAGE4_POINTS})",
    )
    parser.add_argument(
        "--settling-cycles",
        type=_nonnegative_int,
        default=DEFAULT_STAGE4_SETTLING_CYCLES,
        help=f"discarded cycles per point (default: {DEFAULT_STAGE4_SETTLING_CYCLES})",
    )
    parser.add_argument(
        "--measure-cycles",
        type=_positive_int,
        default=DEFAULT_STAGE4_MEASURE_CYCLES,
        help=f"FRF-fit cycles per point (default: {DEFAULT_STAGE4_MEASURE_CYCLES})",
    )
    parser.add_argument(
        "--sample-rate-hz",
        type=_positive_float,
        default=DEFAULT_STAGE4_SAMPLE_RATE_HZ,
        help=f"command/response rate (default: {DEFAULT_STAGE4_SAMPLE_RATE_HZ:g} Hz)",
    )
    parser.add_argument(
        "--no-auto-amplitude",
        dest="auto_amplitude",
        action="store_false",
        default=True,
        help="disable automatic excitation reduction when motion approaches a safety limit",
    )
    parser.add_argument(
        "--amplitude-reduce-at",
        type=_positive_float,
        default=0.70,
        metavar="RATIO",
        help=(
            "reduce the next point when position/velocity uses this fraction "
            "of its hard limit (default: 0.70)"
        ),
    )
    parser.add_argument(
        "--amplitude-reduce-factor",
        type=_positive_float,
        default=0.70,
        metavar="RATIO",
        help="multiply excitation by this factor when reducing (default: 0.70)",
    )
    parser.set_defaults(func=_stage4)


def register(subparsers: Any) -> None:
    parser = subparsers.add_parser(
        "identify",
        help="run the four-stage mechanical identification workflow",
        description=(
            "Supplement trusted DAMIAO register parameters with targeted mechanical experiments. "
            "The workflow is VEL friction -> breakaway -> torque linearity "
            "envelope -> MIT torque FRF."
        ),
    )
    stages = parser.add_subparsers(dest="identify_stage", metavar="STAGE", required=True)
    _register_stage1(stages)
    _register_stage2(stages)
    _register_stage3(stages)
    _register_stage4(stages)
