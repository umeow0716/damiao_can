# Copyright 2026 Enactic, Inc.
# Licensed under the Apache License, Version 2.0

"""Probe DAMIAO identity registers without assuming a motor type."""

from __future__ import annotations

import argparse

from .common import add_common_options, configure_interface, format_id, id_range, load_api

DEFAULT_WAIT_US = 100_000


def _fmt(value: object, digits: int = 6) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        return f"{value:.{digits}g}"
    return str(value)


def _confidence_name(value: object) -> str:
    name = getattr(value, "name", None)
    if name:
        return str(name).lower()
    return str(value).rsplit(".", 1)[-1].lower()


def run(args: argparse.Namespace) -> int:
    api = load_api()
    configure_interface(api, args)
    device = api.DamiaoCAN(args.interface, args.fd)

    any_response = False
    for send_id in id_range(args.from_id, args.to_id):
        result = device.probe_motor_identity(send_id, args.wait_us)
        if not result.responded:
            print(f"{format_id(send_id)}  no response")
            continue

        any_response = True
        regs = result.registers
        limits = result.protocol_limits
        protocol_status = "ready" if limits is not None else "unavailable"
        print(
            f"{format_id(send_id)}  protocol={result.protocol_family}  "
            f"limits={protocol_status}  model={result.model_name} "
            f"confidence={_confidence_name(result.confidence)}"
        )
        print(
            "  "
            f"sw={result.sw_version_ascii or '-'} "
            f"hw={result.hw_version_ascii or '-'} "
            f"sn={result.serial_ascii or '-'} "
            f"NPP={_fmt(regs.npp)} Gr={_fmt(regs.gr)}"
        )
        print(
            "  "
            f"PMAX={_fmt(regs.pmax)} VMAX={_fmt(regs.vmax)} TMAX={_fmt(regs.tmax)} "
            f"Rs={_fmt(regs.rs)} Ls={_fmt(regs.ls)} Flux={_fmt(regs.flux)}"
        )
        print(f"  reason: {result.reason}")

    return 0 if any_response else 1


def register(subparsers: argparse._SubParsersAction[argparse.ArgumentParser]) -> None:
    parser = subparsers.add_parser(
        "probe",
        help="read motor registers and resolve protocol limits without a MotorType preset",
        description=(
            "Read DAMIAO PMAX/VMAX/TMAX and identity registers through CAN ID 0x7FF. "
            "Protocol scaling comes directly from the registers; the physical model label is optional."
        ),
    )
    add_common_options(parser)
    parser.add_argument(
        "--wait-us",
        type=int,
        default=DEFAULT_WAIT_US,
        metavar="US",
        help="maximum response wait for each register read (default: 100000 us)",
    )
    parser.set_defaults(func=run)
