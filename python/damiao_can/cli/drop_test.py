# Copyright 2026 Enactic, Inc.
# Licensed under the Apache License, Version 2.0

"""Implementation of the ``drop-test`` CLI command."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

from .common import add_common_options, format_id, id_range, load_api, make_device, recv_ids

DEFAULT_WAIT_US = 500
DEFAULT_DURATION = 10.0


@dataclass
class _LossCounter:
    send_id: int
    recv_id: int
    expected: int = 0
    received: int = 0

    @property
    def missrate(self) -> float:
        if self.expected == 0:
            return 0.0
        return (self.expected - self.received) * 100.0 / self.expected


def _render_progress(elapsed: float, duration: float, cycles: int) -> None:
    if not sys.stdout.isatty():
        return
    ratio = min(1.0, elapsed / duration) if duration > 0 else 1.0
    width = 24
    filled = int(width * ratio)
    bar = "#" * filled + "-" * (width - filled)
    sys.stdout.write(
        f"\r\033[2Kdrop-test [{bar}] {ratio * 100:5.1f}% "
        f"{min(elapsed, duration):.1f}/{duration:.1f}s cycles={cycles}"
    )
    sys.stdout.flush()


def run(args: argparse.Namespace) -> int:
    if args.duration <= 0:
        raise ValueError("--duration must be greater than zero")
    if args.wait_us < 0:
        raise ValueError("--wait-us must be non-negative")

    api = load_api()
    send_ids = id_range(args.from_id, args.to_id)
    receive_ids = recv_ids(send_ids)
    device = make_device(api, args, send_ids)
    counters = {
        send_id: _LossCounter(send_id=send_id, recv_id=recv_id)
        for send_id, recv_id in zip(send_ids, receive_ids)
    }

    start = time.monotonic()
    deadline = start + args.duration
    cycles = 0
    last_render = 0.0

    try:
        while True:
            now = time.monotonic()
            if now >= deadline:
                break

            device.flush_rx()
            device.refresh_all()
            result = device.recv_all(args.wait_us)
            missing = set(result.missing)

            cycles += 1
            for send_id, counter in counters.items():
                counter.expected += 1
                if send_id not in missing:
                    counter.received += 1

            elapsed = time.monotonic() - start
            if elapsed - last_render >= 0.1:
                _render_progress(elapsed, args.duration, cycles)
                last_render = elapsed
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        exc.args = (
            exc.args[0]
            + "\nIf error show 'No buffer space available'\nTry to set --wait-us longer for test",
        )
        raise exc
    finally:
        _render_progress(
            min(time.monotonic() - start, args.duration), args.duration, cycles
        )
        if sys.stdout.isatty():
            sys.stdout.write("\n")

    for counter in counters.values():
        print(
            f"send_id: {format_id(counter.send_id)}, "
            f"recv_id: {format_id(counter.recv_id)} "
            f"expected: {counter.expected},  "
            f"recvied: {counter.received}, "
            f"missrate: {counter.missrate:.2f}%"
        )
    return 0


def register(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser(
        "drop-test",
        help="measure per-motor response packet loss",
        description=(
            "Repeatedly flush, refresh, and recv_all() while tracking packet loss.\n"
            "Host interface defaults: CAN-FD enabled, bitrate 1M, dbitrate 5M."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  python -m damiao_can drop-test -i can0 --from 0x01 --to 0x08
  python -m damiao_can drop-test -i can0 --from 0x01 --to 0x08 --duration 30 --wait-us 1000
""",
    )
    add_common_options(parser)
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION,
        metavar="SECONDS",
        help="test duration in seconds (default: 10)",
    )
    parser.add_argument(
        "--wait-us",
        type=int,
        default=DEFAULT_WAIT_US,
        metavar="US",
        help="recv_all timeout in microseconds (default: 500)",
    )
    parser.set_defaults(func=run)
