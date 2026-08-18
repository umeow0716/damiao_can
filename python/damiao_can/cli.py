# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Command-line tools for common DAMIAO CAN setup and diagnostics."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Any, Sequence

DEFAULT_BITRATE = 1_000_000
DEFAULT_DBITRATE = 5_000_000
DEFAULT_WAIT_US = 500
DEFAULT_DURATION = 10.0
RECV_ID_OFFSET = 0x10

# DAMIAO can_br (RID 0x23) values used by the OpenArm CAN utility.
MOTOR_BAUDRATE_CODES = {
    1_000_000: 4,
    5_000_000: 9,
}


def _load_api() -> Any:
    # Keep the parser/testable helpers importable without loading the native module.
    import damiao_can as dc

    return dc


def _parse_int(value: str) -> int:
    try:
        return int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"invalid integer {value!r}; use decimal or 0x-prefixed hexadecimal"
        ) from exc


def _parse_rate(value: str) -> int:
    text = value.strip().lower().replace("_", "")
    multipliers = {"k": 1_000, "m": 1_000_000}
    if text and text[-1] in multipliers:
        try:
            number = float(text[:-1])
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"invalid bitrate {value!r}") from exc
        result = int(number * multipliers[text[-1]])
    else:
        try:
            result = int(text, 0)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"invalid bitrate {value!r}") from exc

    if result <= 0:
        raise argparse.ArgumentTypeError("bitrate must be greater than zero")
    return result


def _format_id(can_id: int) -> str:
    width = max(2, (can_id.bit_length() + 3) // 4)
    return f"0x{can_id:0{width}x}"


def _id_range(from_id: int, to_id: int) -> list[int]:
    if from_id < 0 or to_id < 0:
        raise ValueError("CAN IDs must be non-negative")
    if from_id > to_id:
        raise ValueError("--from must be less than or equal to --to")
    if to_id + RECV_ID_OFFSET > 0x7FF:
        raise ValueError(
            "CAN ID range is too high: recv_id = send_id + 0x10 must fit in an 11-bit CAN ID"
        )
    return list(range(from_id, to_id + 1))


def _recv_ids(send_ids: Sequence[int]) -> list[int]:
    return [send_id + RECV_ID_OFFSET for send_id in send_ids]


def _effective_dbitrate(fd: bool, dbitrate: int | None) -> int:
    if dbitrate is not None:
        return dbitrate
    return DEFAULT_DBITRATE if fd else DEFAULT_BITRATE


def _configure_interface(api: Any, args: argparse.Namespace) -> None:
    dbitrate = _effective_dbitrate(args.fd, args.dbitrate)
    helper = api.CANHelper(args.interface)
    helper.set_down()
    helper.set_bitrate(args.bitrate, dbitrate, args.fd)
    helper.set_up()


def _make_device(api: Any, args: argparse.Namespace, send_ids: Sequence[int]) -> Any:
    _configure_interface(api, args)
    device = api.DamiaoCAN(args.interface, args.fd)
    recv_ids = _recv_ids(send_ids)

    # Motor type is irrelevant for these CLI operations because callbacks are ignored;
    # we only need the send/receive CAN-ID routing provided by DamiaoCAN.
    motor_types = [api.MotorType.DM4310] * len(send_ids)
    device.init_motors(motor_types, list(send_ids), recv_ids)
    device.set_callback_mode_all(api.CallbackMode.IGNORE)
    return device


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

    # Write can_br (RID 0x23) using the DAMIAO parameter-write protocol.
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
        socket, write, f"write baudrate for motor {_format_id(send_id)}")
    time.sleep(0.020)

    # DAMIAO requires the motor to be disabled before saving parameters to flash.
    disable = _make_can_frame(api, send_id, bytes([0xFF] * 7 + [0xFD]))
    _write_frame(socket, disable, f"disable motor {_format_id(send_id)}")
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
    _write_frame(
        socket, save, f"save baudrate for motor {_format_id(send_id)}")
    time.sleep(0.040)


def _cmd_set_zero(args: argparse.Namespace) -> int:
    api = _load_api()
    send_ids = _id_range(args.from_id, args.to_id)
    device = _make_device(api, args, send_ids)

    device.flush_rx()
    device.set_zero_all()
    result = device.recv_all(1_000_000)
    print(result)
    return 0 if result.ok else 2


def _cmd_set_baudrate(args: argparse.Namespace) -> int:
    api = _load_api()
    send_ids = _id_range(args.from_id, args.to_id)
    _configure_interface(api, args)

    # Match the OpenArm motor-baudrate utility: configuration commands are
    # classic CAN frames even when the interface is currently CAN-FD capable.
    socket = api.CANSocket(args.interface, False)
    for send_id in send_ids:
        _change_motor_baudrate(api, socket, send_id, args.baudrate)
        print(
            f"motor {_format_id(send_id)}: baudrate set to "
            f"{'5M (CAN-FD)' if args.baudrate == 5_000_000 else '1M (Classic CAN)'}"
        )

    print("Power-cycle the motor(s) before using the new saved baudrate.")
    return 0


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


def _cmd_drop_test(args: argparse.Namespace) -> int:
    if args.duration <= 0:
        raise ValueError("--duration must be greater than zero")
    if args.wait_us < 0:
        raise ValueError("--wait-us must be non-negative")

    api = _load_api()
    send_ids = _id_range(args.from_id, args.to_id)
    recv_ids = _recv_ids(send_ids)
    device = _make_device(api, args, send_ids)
    counters = {
        send_id: _LossCounter(send_id=send_id, recv_id=recv_id)
        for send_id, recv_id in zip(send_ids, recv_ids)
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
        exc.args = (exc.args[0] + '\nIf error show \'No buffer space available\'\nTry to set --wait-us longer for test', )
        raise exc
    finally:
        _render_progress(min(time.monotonic() - start,
                         args.duration), args.duration, cycles)
        if sys.stdout.isatty():
            sys.stdout.write("\n")

    for counter in counters.values():
        print(
            f"send_id: {_format_id(counter.send_id)}, "
            f"recv_id: {_format_id(counter.recv_id)} "
            f"expected: {counter.expected},  "
            f"recvied: {counter.received}, "
            f"missrate: {counter.missrate:.2f}%"
        )
    return 0


def _add_common_options(parser: argparse.ArgumentParser) -> None:
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
        type=_parse_int,
        metavar="ID",
        help="first motor send ID, inclusive (decimal or hex, e.g. 0x01)",
    )
    parser.add_argument(
        "--to",
        dest="to_id",
        required=True,
        type=_parse_int,
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
        type=_parse_rate,
        default=DEFAULT_BITRATE,
        metavar="RATE",
        help="host nominal CAN bitrate (default: 1M)",
    )
    parser.add_argument(
        "--dbitrate",
        type=_parse_rate,
        default=None,
        metavar="RATE",
        help="host CAN-FD data bitrate (default: 5M; ignored when --no-fd is used)",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m damiao_can",
        description="DAMIAO motor setup and CAN diagnostics.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Required on every command:
  -i, --interface IFACE   SocketCAN interface, e.g. can0
  --from ID               First motor send ID, inclusive
  --to ID                 Last motor send ID, inclusive
                          recv_id is always send_id + 0x10

Common optional interface settings:
  --no-fd                 Disable host CAN-FD. Default: FD enabled
  --bitrate RATE          Host nominal bitrate. Default: 1M
  --dbitrate RATE         Host FD data bitrate. Default: 5M
                          Ignored when --no-fd is used

Commands:
  set-zero       Set current motor positions as zero
  set-baudrate   Change and save motor CAN baudrate
  drop-test      Measure per-motor response packet loss

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
""",
    )
    subparsers = parser.add_subparsers(
        dest="command", metavar="COMMAND", required=True)

    set_zero = subparsers.add_parser(
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
    _add_common_options(set_zero)
    set_zero.set_defaults(func=_cmd_set_zero)

    set_baudrate = subparsers.add_parser(
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
    _add_common_options(set_baudrate)
    set_baudrate.add_argument(
        "--baudrate",
        required=True,
        type=_parse_rate,
        choices=tuple(MOTOR_BAUDRATE_CODES),
        metavar="{1M,5M}",
        help="new motor baudrate: 1M = Classic CAN, 5M = CAN-FD",
    )
    set_baudrate.set_defaults(func=_cmd_set_baudrate)

    drop_test = subparsers.add_parser(
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
    _add_common_options(drop_test)
    drop_test.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION,
        metavar="SECONDS",
        help="test duration in seconds (default: 10)",
    )
    drop_test.add_argument(
        "--wait-us",
        type=int,
        default=DEFAULT_WAIT_US,
        metavar="US",
        help="recv_all timeout in microseconds (default: 500)",
    )
    drop_test.set_defaults(func=_cmd_drop_test)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except (RuntimeError, ValueError, IndexError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
