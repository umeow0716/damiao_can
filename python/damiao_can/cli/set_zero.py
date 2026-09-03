# Copyright 2026 Enactic, Inc.
# Licensed under the Apache License, Version 2.0

"""Implementation of the ``set-zero`` CLI command."""

from __future__ import annotations

import argparse

from .common import add_common_options, id_range, load_api, make_device


def run(args: argparse.Namespace) -> int:
    api = load_api()
    send_ids = id_range(args.from_id, args.to_id)
    device = make_device(api, args, send_ids)

    device.flush_rx()
    device.set_zero_all()
    result = device.recv_all(1_000_000)
    print(result)
    return 0 if result.ok else 2


def register(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser(
        "set-zero",
        help="set the current position as zero for a range of motor IDs",
        description=(
            "Set zero for motor send IDs FROM..TO and report missing responses.\n"
            "Host interface defaults: CAN-FD enabled, bitrate 1M, dbitrate 5M."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Example:
  python -m damiao_can set-zero -i can0 --from 0x01 --to 0x08

Classic CAN host example:
  python -m damiao_can set-zero -i can0 --from 0x01 --to 0x08 --no-fd
""",
    )
    add_common_options(parser)
    parser.set_defaults(func=run)
