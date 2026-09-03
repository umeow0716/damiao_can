# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Shared helpers for the DAMIAO command-line interface."""

from __future__ import annotations

import argparse
from typing import Any, Sequence

DEFAULT_BITRATE = 1_000_000
DEFAULT_DBITRATE = 5_000_000
RECV_ID_OFFSET = 0x10


def load_api() -> Any:
    # Keep parser/testable helpers importable without loading the native module.
    import damiao_can as dc

    return dc


def parse_int(value: str) -> int:
    try:
        return int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"invalid integer {value!r}; use decimal or 0x-prefixed hexadecimal"
        ) from exc


def parse_rate(value: str) -> int:
    text = value.strip().lower().replace("_", "")
    multipliers = {"k": 1_000, "m": 1_000_000}
    if text and text[-1] in multipliers:
        try:
            number = float(text[:-1])
        except ValueError as exc:
            raise argparse.ArgumentTypeError(f"invalid bitrate {value!r}") from exc
        result = int(number * multipliers[text[-1]])
    else:
        try:
            result = int(text, 0)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(f"invalid bitrate {value!r}") from exc

    if result <= 0:
        raise argparse.ArgumentTypeError("bitrate must be greater than zero")
    return result


def format_id(can_id: int) -> str:
    width = max(2, (can_id.bit_length() + 3) // 4)
    return f"0x{can_id:0{width}x}"


def id_range(from_id: int, to_id: int) -> list[int]:
    if from_id < 0 or to_id < 0:
        raise ValueError("CAN IDs must be non-negative")
    if from_id > to_id:
        raise ValueError("--from must be less than or equal to --to")
    if to_id + RECV_ID_OFFSET > 0x7FF:
        raise ValueError(
            "CAN ID range is too high: recv_id = send_id + 0x10 must fit in an 11-bit CAN ID"
        )
    return list(range(from_id, to_id + 1))


def recv_ids(send_ids: Sequence[int]) -> list[int]:
    return [send_id + RECV_ID_OFFSET for send_id in send_ids]


def effective_dbitrate(fd: bool, dbitrate: int | None) -> int:
    if dbitrate is not None:
        return dbitrate
    return DEFAULT_DBITRATE if fd else DEFAULT_BITRATE


def configure_interface(api: Any, args: argparse.Namespace) -> None:
    dbitrate = effective_dbitrate(args.fd, args.dbitrate)
    helper = api.CANHelper(args.interface)
    helper.set_down()
    helper.set_bitrate(args.bitrate, dbitrate, args.fd)
    helper.set_up()


def make_device(api: Any, args: argparse.Namespace, send_ids: Sequence[int]) -> Any:
    configure_interface(api, args)
    device = api.DamiaoCAN(args.interface, args.fd)
    receive_ids = recv_ids(send_ids)

    # Motor type is irrelevant for these CLI operations because callbacks are ignored;
    # we only need the send/receive CAN-ID routing provided by DamiaoCAN.
    motor_types = [api.MotorType.DM4310] * len(send_ids)
    device.init_motors(list(send_ids), receive_ids, motor_types)
    device.set_callback_mode_all(api.CallbackMode.IGNORE)
    return device


def add_common_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "-i",
        "--interface",
        required=True,
        metavar="IFACE",
        help="SocketCAN interface to use (required, e.g. can0)",
    )
    parser.add_argument(
        "--from",
        dest="from_id",
        required=True,
        type=parse_int,
        metavar="ID",
        help="first motor send ID, inclusive (decimal or hex, e.g. 0x01)",
    )
    parser.add_argument(
        "--to",
        dest="to_id",
        required=True,
        type=parse_int,
        metavar="ID",
        help="last motor send ID, inclusive; recv ID is send ID + 0x10",
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
