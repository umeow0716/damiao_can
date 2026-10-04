# DM4310 passive System Identification telemetry v1

Offline verification results and reproducible commands: [validation](SYSTEM_IDENTIFICATION_VALIDATION.md). Downstream script changes: [damiao_ws handoff](DAMIAO_WS_SYSID_HANDOFF.md).

This API targets the opt-in `dm4310_sysid` firmware, whose base is raw/no-response. It provides passive measurement and management only. It does not configure excitation, enable a motor, configure the CAN interface, change motor parameters, or stop motor motion.

The implementation was checked against the current source in `../DM4310_firmware`: `docs/SYSTEM_IDENTIFICATION_PROTOCOL.md`, `docs/SYSTEM_IDENTIFICATION_VALIDATION.md`, `app/include/system_identification.h`, and `app/src/system_identification.c`. Those firmware sources remain authoritative for the wire layout.

## API and transport ownership

C++ includes `<damiao_can/sysid/system_identification.hpp>` and uses namespace `damiao_can::sysid`. Python exports `SystemIdentification` and the `SysId*` typed records directly from `damiao_can`.

`SystemIdentification(interface, node, offset=0.0, reversed=False, queue_capacity=2048, live_only=False)` owns its own FD-enabled CANSocket. It installs exact standard-data filters for management/sample IDs, so other bus messages remain available on independently owned legacy sockets. This socket must not be shared with another receiver. Existing FD-enabled legacy sockets now accept both classic and FD frames; an unrelated classic ACK no longer causes an FD frame-size exception.

There is no automatic background receiver, heartbeat, controller, or interface configuration. Call `poll()` or `read_samples()` regularly. Public calls serialize on a mutex; do not expect parallel calls on one object to bypass a management wait. Python blocking I/O releases the GIL. Closing the object is explicit and closes its owned descriptor; destruction also closes it without transmitting operations.

Each sample, unmatched-reply, other-frame, and pending-START queue is bounded by queue_capacity. Full queues drop new data and increment separate host drop counters. Kernel receive loss uses SO_RXQ_OVFL and remains separate. Successful decoding preserves raw payload and a host timestamp in nanoseconds from C++ `std::chrono::steady_clock`, recorded immediately after receive. Its epoch is unspecified; do not subtract Python `time.monotonic_ns()` or UNIX time from it without a separate clock alignment.

| Method | Return / behavior |
| --- | --- |
| `info(timeout_us)`, `status(timeout_us)` | SysIdReply: information on success, typed ACK result on rejection |
| `start(rate_hz=500, timeout_us)` | SysIdStartResult: session_id, accepted, recovery status if START ACK timed out |
| `heartbeat(timeout_us)` | SysIdReply; requires an owned session |
| `stop(timeout_us, drain_timeout_us, max_drain_frames)` | SysIdStopResult: STOP reply, drained measurements, final STATUS, drain-limit indication |
| `poll(timeout_us=0, max_frames=256)` | Number of frames demultiplexed; does not return measurements |
| `read_samples(timeout_us=0, max_frames=256)` | Pumps reception, then returns and removes queued SysIdMeasurement records |
| `take_replies()`, `take_other_frames()` | Removes unmatched queued records; production filters exclude other IDs |
| `diagnostics()` | Snapshot of host/kernel loss, invalid frames, gaps, and timeline diagnostics |
| `owned_session` (Python), `owned_session()` (C++) | Current owned session or None/nullopt |
| `close()` | Releases reception resources; no STOP or motor-control side effect |

Timeouts are microseconds per management request. START may also issue baseline/recovery STATUS, and STOP issues ownership/final STATUS plus the bounded drain, so a whole method may last longer than one timeout_us. The drain cap and deadline bound draining; reaching a cap does not prove all bus queues empty. Defaults are 100 ms per management request and 50 ms / 4096 frames for STOP draining.

Drain ends on a receive quiet interval of up to 2 ms or the requested time/frame limit. drain_limit_reached marks a hard limit; a quiet interval is not proof that all firmware/hardware queues are empty. Final STATUS requests continue to demultiplex arriving samples, and delayed data can still be read after STOP while the same session remains owned.

START checks STATUS for inactivity and fences old data before transmitting one START. If its ACK is lost, a new STATUS must match the attempted session; no retry or unrelated STOP is sent. An accepted recovered session may already be inactive: inspect recovery_status.active/stop_reason. New sessions reset the host timeline. Unexpected period/counter regression or session change invalidates attribution and requires recovery/new START.

Firmware ACK rejections are values in SysIdReply, not guessed success. Malformed ID/version/type/length/flags and non-finite payload values raise SysIdProtocolError. Missing management replies raise SysIdTimeoutError; unconfirmed ownership/session transitions raise SysIdSessionError. Transport failures raise CANSocketException. Exceptions do not invoke motor shutdown.

## Single-motor passive Python example

The interface must already be configured through CANHelper and all bus nodes must be FD-compatible. This example performs no control or excitation. The application's separately authorized controller must maintain commands, hold the other axes, and satisfy the original motor watchdog throughout.

```python
import time
import damiao_can as dc


def record_passive_capture(interface, node, duration_s, offset=0.0, reversed=False):
    capture = dc.SystemIdentification(interface, node, offset, reversed)
    measurements = []
    initial_info = None
    stopped = None
    try:
        reply = capture.info()
        if not reply.accepted:
            raise RuntimeError(f"INFO rejected: {reply.ack.result}")
        initial_info = reply.information
        started = capture.start(rate_hz=500)
        if not started.accepted:
            raise RuntimeError(f"START rejected: {started.reply.ack.result}")
        if started.recovery_status is not None and not started.recovery_status.active:
            raise RuntimeError("START accepted, but capture already stopped; save recovery status")
        deadline = time.monotonic() + duration_s
        next_heartbeat = time.monotonic() + 0.1
        while time.monotonic() < deadline:
            # Motor control is a separate application responsibility; no recv_all here.
            measurements.extend(capture.read_samples(timeout_us=2000))
            if time.monotonic() >= next_heartbeat:
                heartbeat = capture.heartbeat()
                if not heartbeat.accepted:
                    raise RuntimeError(f"HEARTBEAT rejected: {heartbeat.ack.result}")
                next_heartbeat = time.monotonic() + 0.1
    finally:
        try:
            if capture.owned_session is not None:
                # STOP checks current ownership before sending. It does not stop motion.
                stopped = capture.stop()
                measurements.extend(stopped.samples)
        finally:
            diagnostics = capture.diagnostics()
            capture.close()
        # The controlling application performs motor shutdown separately, even on failure.
    return initial_info, measurements, stopped, diagnostics
```

Persist `measurement.sample.raw.payload`, raw sample fields, adjusted measurement fields, session_id/timeline_epoch, raw/unwrapped counters, gap diagnostics, INFO and final STATUS. On errors also preserve the application's partial log and exception. Do not treat rejected STOP, a drain cap, or missing final STATUS as a complete capture.

## C++ usage

```cpp
#include <damiao_can/sysid/system_identification.hpp>

// Only call this lifecycle in an explicitly authorized application.
damiao_can::sysid::SystemIdentification capture("can0", 1, 0.15, false);
auto information = capture.info();
auto started = capture.start(500);
// Check typed results; service read_samples() and heartbeat() in the host loop.
auto measurements = capture.read_samples(2000);
auto heartbeat = capture.heartbeat();
auto stopped = capture.stop();
capture.close();
// Motor control, excitation, and actual motor shutdown are independent.
```

Pure offline decoders are available as C++ decode_ack/decode_information/decode_sample and Python decode_sysid_ack/decode_sysid_information/decode_sysid_sample. SysIdFrame explicitly includes CAN ID, classic/FD indicator, FD flags, payload bytes, and receive timestamp, so offline replay need not parse SocketCAN headers. encode_sysid_request and sysid_command_sequence_distance expose the request format and reserved-command-counter arithmetic.

## Wire contract

Requests use standard data ID `0x6F0`, 8 bytes. Responses use `0x6F1`: classic8 ACK, or FD64+BRS INFO/STATUS. Samples use `0x6F2`, FD64+BRS. All multi-byte payload fields are little-endian and floats are IEEE-754 binary32.

| Operation | Value | Argument |
| --- | --- | --- |
| CAPTURE_START | 1 | 0: 500 Hz; 1: 1 kHz |
| CAPTURE_STOP | 2 | 0 |
| CAPTURE_HEARTBEAT | 3 | 0 |
| INFO | 4 | 0 |
| STATUS | 5 | 0 |

There are no firmware excitation operations in v1. Only one target may stream on a bus. A successful START uses the request sequence as its session ID. Request sequence is an echo tag, not a deduplication key: repeating START is not idempotent.

ACK results distinguish accepted, unsupported protocol version, invalid argument, busy, FD transport not ready, unsupported operation, and measurements not ready. INFO/STATUS success returns its FD64 response directly, without an additional ACK. ACK has no node field; only operation and request sequence can be matched. INFO/STATUS additionally identifies the node.

## Coordinate and measurement conventions

Raw samples retain firmware output-side position, velocity, averaged Iq, torque estimate, MIT feedforward torque, and instantaneous Iq. Firmware direction and zero calibration are already applied. An application-coordinate view uses the same convention as Motor: `q = sign * wire_position + offset`, `dq = sign * wire_velocity`, and `tau = sign * wire_torque`. Apply that view once; retain wire values and raw payload for audit. Output-side torque is averaged Iq times the output torque constant; do not multiply gear ratio again. This is an electrical torque estimate, not an independent shaft torque measurement.

The adjusted measurement view also applies sign to averaged/instantaneous Iq and MIT feedforward torque, keeping current and torque direction consistent. Offset affects position only. Gains, temperature, filter settings, timestamps, and counters retain their wire meanings.

The feedforward torque field is only the endpoint MIT feedforward command. It excludes MIT feedback terms and is not the full VEL/POSVEL controller torque. Applied command sequence means the control path read the command, not that the row's measured current has responded to it.

## Time, filtering, and integrity

Firmware control tick is a nominal 20 kHz invocation count: 50 microseconds per tick. It is not host wall time. Samples also retain velocity update tick, position publication marker, command-consumption tick, and interval ticks; these are different measurement times.

- Position is the latest encoder DMA publication. Its marker is not a measured encoder hardware latch timestamp.
- Velocity is an encoder difference/average window ending at velocity tick: nominal N=20 ticks at 1 kHz or N=40 at 500 Hz. Telemetry velocity does not use the factory velocity LPF.
- Averaged Iq and torque use four cascaded 20 kHz IIR stages followed by an interval average.
- Instantaneous Iq is the endpoint ADC/Park value without anti-alias filtering and is diagnostic only.
- Temperature uses the firmware's existing filtered temperature.

INFO/STATUS preserve output torque constant, gear ratio, factory velocity LPF previous weight, telemetry Iq beta, current limit, and sample period. The Iq transfer function is:

```text
H_iq(z) = [beta / (1 - (1-beta) z^-1)]^4
          * (1/N) sum(j=0..N-1) z^-j
```

Current beta is approximately 0.015465039. At 20 Hz the gain is approximately 0.739 at 1 kHz and 0.738 at 500 Hz. Preserve these settings for later analysis; the API neither removes filtering nor fits a model. Instantaneous Iq is not an automatic replacement for averaged Iq. Velocity window and encoder publication delay also affect mechanical FRF.

Keep raw counters when using diagnostics. Sample sequence and tick wrap modulo 2^32; accepted/applied legacy command sequence skips 0xFFFFFFFF, wrapping 0xFFFFFFFE to 0. The reserved command value means unknown and cannot establish command correspondence. Host-observed sequence gaps, firmware sample_drop_count, software tx_retry_count, kernel drops, and application queue drops are different quantities. In particular, tx_retry_count measures software enqueue retry attempts, not lost samples.

Sample frames contain neither session ID nor boot ID. Session attribution requires management fences and exclusive capture ownership. Observable session changes/counter regressions invalidate the old timeline. A reboot or session change that is indistinguishable from normal forward counters on the wire cannot be detected perfectly by v1; do not claim continuous unwrap across a reconnect or suspected reboot.

## Watchdogs and cleanup

Call capture heartbeat approximately every 100 ms; firmware stops generating samples when the interval reaches 500 ms (10,000 nominal ticks). This does not satisfy the separate motor control watchdog.

Capture STOP, capture timeout, API close, and receiver cleanup do not send zero torque, disable, or guarantee motor standstill. Real motor shutdown remains an explicit responsibility of the controlling application. Firmware queued samples can arrive after STOP; bounded draining followed by a final STATUS records final firmware drop counters and stop reason.

No examples or tests in this change constitute hardware authorization. Hardware FD interoperability, bus load with seven holding axes, ISR timing, actual tick frequency, acquisition delay, calibration, and safe loaded operation still require separate validation.

## Independent guard / latest-state API

New sysid firmware requires an explicit guard before enabling any node. Existing calibration layouts and reliable ACK semantics are preserved. Eight-node capability/guard setup belongs to the controlling application. The SDK does not enable or automatically clear faults.

```python
import damiao_can as dc
with dc.SystemIdentification("can0", 1, live_only=True) as live:
    s = live.safety_status().safety  # require enabled deadman, no latch
    live.configure_guard(-0.25, 0.25, 0.3, 2.0)  # raw q, |v|, Iq-derived Nm
    # configure_guard stages all fields, commits atomically, reads back f32 values
    # Normal application must safely enable/control the motor separately.
    live.start_live()
    live.heartbeat()
    state = live.read_latest(timeout_us=2000)
    if state is not None:
        print(state.sample.endpoint_tick, state.position, state.mos_temperature)
    live.stop_live()
```

Instantiate a separate recording object and live object, and run separate receivers. Python I/O releases the GIL, but calls on one object serialize. `live_only=True` filters management 0x6F1 and latest-state 0x6E0, avoiding calibration replay traffic. `read_latest()` skips older/foreign snapshots and has no canonical-record ordering gate; it never releases live packets through `read_samples()`. Position, velocity, Iq and torque transform offset/reversed once; MOS temperature does not transform. MOS temperature is only available in live snapshots; calibration Measurement.mos_temperature defaults to zero and must not be treated as a calibration temperature reading.

Firmware checks 250 ms control-command lease independently of heartbeat/ACK/refresh, plus raw position/velocity/torque/temperature envelope. Live sends every 40 nominal ticks (500 Hz), keeps newest pending state and counts overwrites; it is deliberately not reliable recorded data. `SysIdSafetyInformation` exposes active bounds, lease, control tick, live state, latch and reason. Existing legacy Motor/MotorStateResult now expose actual feedback `get_fault()`/`fault` (0 disabled, 1 enabled, >1 fault), so holder checks need not infer enable from stale client-side flags.

Wire layout and exact firmware behavior: sibling firmware `docs/SYSTEM_IDENTIFICATION_PROTOCOL.md` §10. Physical stopping, real bus latency and WCET remain hardware acceptance tasks; SDK close/capture STOP are not motor shutdown commands.
