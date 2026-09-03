# Copyright 2026 Enactic, Inc.
# Licensed under the Apache License, Version 2.0

"""Command-line tools for common DAMIAO CAN setup and diagnostics."""

from __future__ import annotations

import argparse
import sys
from typing import Sequence

from . import drop_test, probe, set_baudrate, set_zero, workflow


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m damiao_can",
        description="DAMIAO motor setup and CAN diagnostics.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Common interface option:
  -i, --interface IFACE   SocketCAN interface, e.g. can0

Range commands (set-zero, set-baudrate, drop-test):
  --from ID               First motor send ID, inclusive
  --to ID                 Last motor send ID, inclusive
                          recv_id is always send_id + 0x10

Identification command:
  identify -i IFACE --id ID
                          Run the complete four-stage workflow
                          friction -> breakaway -> envelope -> FRF

Common optional interface settings:
  --no-fd                 Disable host CAN-FD. Default: FD enabled
  --bitrate RATE          Host nominal bitrate. Default: 1M
  --dbitrate RATE         Host FD data bitrate. Default: 5M
                          Ignored when --no-fd is used

Commands:
  set-zero       Set current motor positions as zero
  set-baudrate   Change and save motor CAN baudrate
  drop-test      Measure per-motor response packet loss
  probe          Read identity registers and infer motor type
  identify       Run the four-stage mechanical identification workflow

Examples:
  # Set zero on can0, host defaults to CAN-FD 1M/5M
  python -m damiao_can set-zero -i can0 --from 0x01 --to 0x08

  # Set zero using Classic CAN on the host at 1M
  python -m damiao_can set-zero -i can0 --from 0x01 --to 0x08 --no-fd

  # Change motors to 5M (CAN-FD); host interface still defaults to FD 1M/5M
  python -m damiao_can set-baudrate -i can0 --from 0x01 --to 0x08 --baudrate 5M

  # Change motors to 1M (Classic CAN). --baudrate controls the motor;
  # --no-fd controls only how the host interface talks to the motors now.
  python -m damiao_can set-baudrate -i can0 --from 0x01 --to 0x08 --baudrate 1M

  # Run a 10-second packet-loss test with the default 500 us recv_all wait
  python -m damiao_can drop-test -i can0 --from 0x01 --to 0x08

  # Probe motor identity without pre-selecting MotorType
  python -m damiao_can probe -i can0 --from 0x01 --to 0x08

  # Run the complete mechanical identification workflow
  python -m damiao_can identify -i can0 --id 0x01
""",
    )
    subparsers = parser.add_subparsers(
        dest="command", metavar="COMMAND", required=True
    )
    set_zero.register(subparsers)
    set_baudrate.register(subparsers)
    drop_test.register(subparsers)
    probe.register(subparsers)
    workflow.register(subparsers)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except (RuntimeError, ValueError, IndexError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


__all__ = ["build_parser", "main"]
