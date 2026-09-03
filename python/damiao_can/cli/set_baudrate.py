# Copyright 2026 Enactic, Inc.
# Licensed under the Apache License, Version 2.0

"""Implementation of the ``set-baudrate`` CLI command."""

from __future__ import annotations

import argparse
import time
from typing import Any

from .common import (
    add_common_options,
    configure_interface,
    format_id,
    id_range,
    load_api,
    parse_rate,
)

MOTOR_BAUDRATE_CODES = {
    1_000_000: 4,
    5_000_000: 9,
}


def _make_can_frame(api: Any, can_id: int, data: bytes) -> Any:
    if len(data) > 8:
        raise ValueError("classic CAN frame payload must not exceed 8 bytes")
    frame = api.CanFrame()
    frame.can_id = can_id
    frame.data = data
    return frame


def _write_frame(socket: Any, frame: Any, description: str) -> None:
    if not socket.write_can_frame(frame):
        raise RuntimeError(f"failed to {description}")


def _change_motor_baudrate(api: Any, socket: Any, send_id: int, baudrate: int) -> None:
    code = MOTOR_BAUDRATE_CODES[baudrate]

    write = _make_can_frame(
        api,
        0x7FF,
        bytes(
            [
                send_id & 0xFF,
                (send_id >> 8) & 0xFF,
                0x55,
                0x23,
                code,
                0x00,
                0x00,
                0x00,
            ]
        ),
    )
    _write_frame(
        socket, write, f"write baudrate for motor {format_id(send_id)}")
    time.sleep(0.020)

    disable = _make_can_frame(api, send_id, bytes([0xFF] * 7 + [0xFD]))
    _write_frame(socket, disable, f"disable motor {format_id(send_id)}")
    time.sleep(0.050)

    save = _make_can_frame(
        api,
        0x7FF,
        bytes(
            [
                send_id & 0xFF,
                (send_id >> 8) & 0xFF,
                0xAA,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
            ]
        ),
    )
    _write_frame(socket, save, f"save baudrate for motor {format_id(send_id)}")
    time.sleep(0.040)


def run(args: argparse.Namespace) -> int:
    api = load_api()
    send_ids = id_range(args.from_id, args.to_id)
    configure_interface(api, args)

    # Match the OpenArm motor-baudrate utility: configuration commands are
    # classic CAN frames even when the interface is currently CAN-FD capable.
    socket = api.CANSocket(args.interface, False)
    for send_id in send_ids:
        _change_motor_baudrate(api, socket, send_id, args.baudrate)
        print(
            f"motor {format_id(send_id)}: baudrate set to "
            f"{'5M (CAN-FD)' if args.baudrate == 5_000_000 else '1M (Classic CAN)'}"
        )

    print("Power-cycle the motor(s) before using the new saved baudrate.")
    return 0


def register(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser(
        "set-baudrate",
        help="set and save the DAMIAO motor CAN baudrate",
        description=(
            "Set the motor can_br register and save it to flash.\n"
            "Motor target: --baudrate 1M = Classic CAN, 5M = CAN-FD.\n"
            "Host interface defaults independently to CAN-FD 1M/5M; use --no-fd only "
            "when the motors must be reached over Classic CAN."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  # Motors currently reachable using the default host CAN-FD 1M/5M setup; save 5M to motors
  python -m damiao_can set-baudrate -i can0 --from 0x01 --to 0x08 --baudrate 5M

  # Save 1M to motors; host communication still defaults to CAN-FD unless --no-fd is supplied
  python -m damiao_can set-baudrate -i can0 --from 0x01 --to 0x08 --baudrate 1M

  # Reach motors over Classic CAN while changing their saved baudrate
  python -m damiao_can set-baudrate -i can0 --from 0x01 --to 0x08 --baudrate 5M --no-fd
""",
    )
    add_common_options(parser)
    parser.add_argument(
        "--baudrate",
        required=True,
        type=parse_rate,
        choices=tuple(MOTOR_BAUDRATE_CODES),
        metavar="{1M,5M}",
        help="new motor baudrate: 1M = Classic CAN, 5M = CAN-FD",
    )
    parser.set_defaults(func=run)
