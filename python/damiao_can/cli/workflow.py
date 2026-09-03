# Copyright 2026 Enactic, Inc.
# Licensed under the Apache License, Version 2.0

"""One-command mechanical identification workflow."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

from . import identify as stages

_POSITION_LIMIT_FRACTION_OF_PMAX = 0.08
_POSITION_LIMIT_CAP_RAD = 1.0
_VELOCITY_LIMIT_FRACTION_OF_VMAX = 0.20
_VELOCITY_LIMIT_CAP_RAD_S = 2.0
_TEST_VELOCITY_FRACTION_OF_LIMIT = 0.50
_TEST_VELOCITY_CAP_RAD_S = 1.0
_BREAKAWAY_CEILING_FRACTION_OF_TMAX = 0.25
_ENVELOPE_CEILING_FRACTION_OF_TMAX = 0.40
_ENVELOPE_MIN_BREAKAWAY_MULTIPLIER = 2.5
_STAGE3_VELOCITY_FRACTIONS = (-0.5, 0.0, 0.5)


def _copy_args(args: argparse.Namespace, **updates: Any) -> argparse.Namespace:
    values = vars(args).copy()
    values.update(updates)
    return argparse.Namespace(**values)


def _read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _preflight_limits(args: argparse.Namespace) -> tuple[float, float, float]:
    from .common import load_api

    api = load_api()
    device = stages._make_device(
        api,
        args,
        api.ControlMode.VEL,
        api.CallbackMode.STATE,
    )
    limits = device.get_motor(0).get_limits()
    pmax = float(limits.pMax)
    vmax = float(limits.vMax)
    tmax = float(limits.tMax)
    if not all(
        math.isfinite(value) and value > 0.0
        for value in (pmax, vmax, tmax)
    ):
        raise ValueError("motor PMAX/VMAX/TMAX must be positive finite values")
    return pmax, vmax, tmax


def _automatic_motion_limits(
    pmax: float,
    vmax: float,
) -> tuple[float, float, float]:
    max_position_delta = min(
        _POSITION_LIMIT_CAP_RAD,
        _POSITION_LIMIT_FRACTION_OF_PMAX * pmax,
    )
    max_velocity = min(
        _VELOCITY_LIMIT_CAP_RAD_S,
        _VELOCITY_LIMIT_FRACTION_OF_VMAX * vmax,
    )
    test_velocity = min(
        _TEST_VELOCITY_CAP_RAD_S,
        _TEST_VELOCITY_FRACTION_OF_LIMIT * max_velocity,
    )
    if min(max_position_delta, max_velocity, test_velocity) <= 0.0:
        raise ValueError(
            "automatic motion limits resolved to a non-positive value"
        )
    return max_position_delta, max_velocity, test_velocity


def _minimum_tested_linear_limit(csv_path: Path) -> float:
    limits: list[float] = []
    with csv_path.open("r", newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            value = float(row["recommended_excitation_limit_nm"])
            if math.isfinite(value) and value > 0.0:
                limits.append(value)
    if not limits:
        raise RuntimeError(
            "Stage 3 did not establish any tested linear excitation range"
        )
    return min(limits)


def _run(args: argparse.Namespace) -> int:
    output_root = Path(args.output_dir)
    pmax, vmax, tmax = _preflight_limits(args)
    max_position_delta, max_velocity, test_velocity = _automatic_motion_limits(
        pmax,
        vmax,
    )

    args.max_position_delta_rad = max_position_delta
    args.max_velocity_rad_s = max_velocity
    args.max_t_mos_c = stages.DEFAULT_T_MOS_LIMIT_C
    args.max_t_rotor_c = stages.DEFAULT_T_ROTOR_LIMIT_C
    args.max_consecutive_response_misses = (
        stages.DEFAULT_MAX_CONSECUTIVE_RESPONSE_MISSES
    )

    print(
        "automatic setup: "
        f"PMAX={pmax:g}, VMAX={vmax:g}, TMAX={tmax:g}, "
        f"position_abort={max_position_delta:g} rad, "
        f"velocity_abort={max_velocity:g} rad/s, "
        f"V_test={test_velocity:g} rad/s"
    )

    common = {
        "yes": True,
        "max_position_delta_rad": max_position_delta,
        "max_velocity_rad_s": max_velocity,
        "max_t_mos_c": stages.DEFAULT_T_MOS_LIMIT_C,
        "max_t_rotor_c": stages.DEFAULT_T_ROTOR_LIMIT_C,
        "max_consecutive_response_misses": (
            stages.DEFAULT_MAX_CONSECUTIVE_RESPONSE_MISSES
        ),
    }

    stage1_dir = output_root / "stage1"
    stage1_args = _copy_args(
        args,
        **common,
        output_dir=str(stage1_dir),
        test_velocity_rad_s=test_velocity,
        velocity_fractions=[0.25, 0.50, 0.75],
        settle_s=stages.DEFAULT_STAGE1_SETTLE_S,
        measure_s=stages.DEFAULT_STAGE1_MEASURE_S,
        sample_rate_hz=stages.DEFAULT_SAMPLE_RATE_HZ,
    )
    print("[1/4] friction")
    if stages._stage1(stage1_args) != 0:
        return 1

    stage2_dir = output_root / "stage2"
    max_breakaway_torque = _BREAKAWAY_CEILING_FRACTION_OF_TMAX * tmax
    stage2_args = _copy_args(
        args,
        **common,
        output_dir=str(stage2_dir),
        max_torque_nm=max_breakaway_torque,
        ramp_rate_nm_s=stages.DEFAULT_STAGE2_RAMP_RATE_NM_S,
        motion_threshold_rad_s=stages.DEFAULT_STAGE2_MOTION_THRESHOLD_RAD_S,
        consecutive_samples=stages.DEFAULT_STAGE2_CONSECUTIVE_SAMPLES,
        zero_settle_s=stages.DEFAULT_STAGE2_ZERO_SETTLE_S,
        sample_rate_hz=stages.DEFAULT_SAMPLE_RATE_HZ,
    )
    print("[2/4] breakaway")
    if stages._stage2(stage2_args) != 0:
        return 1

    stage2_summary = _read_json(stage2_dir / "stage2_summary.json")
    breakaway = max(
        float(stage2_summary["breakaway_torque_positive_nm"]),
        float(stage2_summary["breakaway_torque_negative_nm"]),
    )

    stage3_dir = output_root / "stage3"
    max_perturbation = min(
        _ENVELOPE_CEILING_FRACTION_OF_TMAX * tmax,
        max(
            _BREAKAWAY_CEILING_FRACTION_OF_TMAX * tmax,
            _ENVELOPE_MIN_BREAKAWAY_MULTIPLIER * breakaway,
        ),
    )
    if max_perturbation <= breakaway:
        raise RuntimeError(
            "automatic Stage 3 ceiling is not above measured breakaway torque"
        )
    stage3_args = _copy_args(
        args,
        **common,
        output_dir=str(stage3_dir),
        velocities_rad_s=[
            fraction * test_velocity for fraction in _STAGE3_VELOCITY_FRACTIONS
        ],
        max_perturbation_nm=max_perturbation,
        steps=stages.DEFAULT_STAGE3_STEPS,
        tracking_ratio_min=stages.DEFAULT_STAGE3_TRACKING_RATIO_MIN,
        safety_margin=stages.DEFAULT_STAGE3_SAFETY_MARGIN,
        settle_s=stages.DEFAULT_STAGE3_SETTLE_S,
        baseline_s=stages.DEFAULT_STAGE3_BASELINE_S,
        pulse_s=stages.DEFAULT_STAGE3_PULSE_S,
        recovery_s=stages.DEFAULT_STAGE3_RECOVERY_S,
        sample_rate_hz=stages.DEFAULT_SAMPLE_RATE_HZ,
    )
    print("[3/4] envelope")
    if stages._stage3(stage3_args) != 0:
        return 1

    linear_limit = _minimum_tested_linear_limit(
        stage3_dir / "torque_linear_limit_vs_velocity.csv"
    )
    if linear_limit <= breakaway:
        raise RuntimeError(
            "measured breakaway torque is not below the tested Stage 3 "
            "linear limit"
        )
    amplitude = 0.5 * (breakaway + linear_limit)

    stage4_dir = output_root / "stage4"
    stage4_args = _copy_args(
        args,
        **common,
        output_dir=str(stage4_dir),
        breakaway_nm=breakaway,
        linear_limit_nm=linear_limit,
        amplitude_nm=amplitude,
        bias_nm=0.0,
        start_hz=stages.DEFAULT_STAGE4_START_HZ,
        stop_hz=stages.DEFAULT_STAGE4_STOP_HZ,
        points=stages.DEFAULT_STAGE4_POINTS,
        settling_cycles=stages.DEFAULT_STAGE4_SETTLING_CYCLES,
        measure_cycles=stages.DEFAULT_STAGE4_MEASURE_CYCLES,
        sample_rate_hz=stages.DEFAULT_STAGE4_SAMPLE_RATE_HZ,
        auto_amplitude=True,
        amplitude_reduce_at=0.70,
        amplitude_reduce_factor=0.70,
    )
    print(
        "[4/4] frf "
        f"(breakaway={breakaway:.6g} Nm, linear={linear_limit:.6g} Nm, "
        f"A={amplitude:.6g} Nm)"
    )
    if stages._stage4(stage4_args) != 0:
        return 1

    output_root.mkdir(parents=True, exist_ok=True)
    stages._write_json(
        output_root / "identification_summary.json",
        {
            "protocol_limits": {"pmax": pmax, "vmax": vmax, "tmax": tmax},
            "automatic_safety": {
                "max_position_delta_rad": max_position_delta,
                "max_velocity_rad_s": max_velocity,
                "max_t_mos_c": stages.DEFAULT_T_MOS_LIMIT_C,
                "max_t_rotor_c": stages.DEFAULT_T_ROTOR_LIMIT_C,
            },
            "stage1_test_velocity_rad_s": test_velocity,
            "breakaway_nm": breakaway,
            "stage3_max_perturbation_nm": max_perturbation,
            "tested_linear_limit_nm": linear_limit,
            "stage4_amplitude_nm": amplitude,
            "stage1_summary": str(stage1_dir / "stage1_summary.json"),
            "stage2_summary": str(stage2_dir / "stage2_summary.json"),
            "stage3_summary": str(stage3_dir / "stage3_summary.json"),
            "stage4_summary": str(stage4_dir / "stage4_summary.json"),
        },
    )
    print(f"complete: {output_root / 'identification_summary.json'}")
    return 0


def register(subparsers: Any) -> None:
    parser = subparsers.add_parser(
        "identify",
        help="run the complete four-stage mechanical identification workflow",
        description=(
            "Run friction -> breakaway -> torque envelope -> torque FRF "
            "continuously. Only --interface and --id are required; experiment "
            "bounds are derived "
            "from motor PMAX/VMAX/TMAX and earlier stage measurements."
        ),
    )
    stages._add_single_motor_options(parser)
    stages._add_output_dir(parser)
    parser.set_defaults(func=_run)
