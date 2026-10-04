# Single-axis raw-coordinate offset scan extension

This opt-in extension is built for dm4310_sysid, dm4340_sysid and dm8009_sysid. Factory/raw/no-response targets do not include it. It does not estimate, persist or apply an offset, and never enables a motor. One writer and one capture source per CAN bus are required. Node 8 remains a holder, not a scan target.

Existing telemetry v1 request/ACK/INFO/sample layouts are unchanged. Requests remain classic/FD8 at 0x6F0: version=1, node, operation, argument=0, payload32 LE. ACK8 echoes operation and payload32. No FD64 request reception is assumed.

## Operations

- 6–15: write lower, upper, home, speed, acceleration, torque limit, temperature limit, repetitions, tracking error, maximum seconds, respectively. payload32 is IEEE754 float bits. Every write has its own operation; operations 7–15 are idempotent. Operation 6 begins a fresh configuration and clears its completion mask, so partial reconfiguration cannot inherit previously completed fields. Writes require capture and scan inactive. All ten fields must have been configured.
- 16: START scan; payload32 is a host-generated scan session tag. Requires owned/active telemetry, warmup complete, armed SPEED mode, fault 0/1, near home with velocity <=0.05. No automatic retry; reconcile lost ACK with STATUS and session identity.
- 17: scan STATUS; payload32 is request sequence. Responds with FD64+BRS type 0x83 at 0x6F1.
- 18: ABORT; payload32 request sequence; SDK checks ownership first. Healthy in-range motion decelerates; severe fault/bound violation requests zero immediately. Existing fault monitor and hardware protections remain authoritative.
- Existing HEARTBEAT 3 maintains capture/trajectory liveness. STOP capture also aborts active trajectory. It does not independently disable a motor.

## Trajectory

Raw coordinates mean the output-side position already reported by firmware, including its existing direction/zero settings, with no host offset or additional reversed transform. They are not ADC/encoder counts.

Let m = tracking_error + speed*0.002 + speed^2/(2*acceleration). Actual position is guarded by lower/upper; scan endpoints are lower+m and upper-m. Home must lie inside these endpoints. Motor/profile PMAX/VMAX/TMAX and physical limits must also be checked by the host. These margins are nominal planning margins, not a hardware-verified stopping-distance guarantee.

At the existing 1 kHz outer loop: preposition to the lower interior endpoint; perform repetitions of lower→upper→lower; return to home; keep zero velocity. Planned position integrates the acceleration-limited velocity reference. Tracking error is checked against actual position; the trajectory is velocity-controlled, not a position servo. Completion requires planned arrival and low measured velocity; the host additionally checks actual home error. It then explicitly takes over and disables all axes.

Phase: 0 none, 1 staged configuration, 2 preposition, 3 forward, 4 reverse, 5 return, 6 completed, 7 aborted, 8 decelerating.
Reason: 0 none, 1 explicit abort/STOP, 2 unhealthy/nonfinite, 3 heartbeat loss, 4 bound violation, 5 torque/temperature/velocity limit, 6 tracking error, 7 elapsed timeout, 8 external legacy setpoint takeover, 9 reliable retention buffer full.

A legacy SETPOINT ends local trajectory ownership. Disable/mode changes are detected by the next outer-loop scan step. No synthesized SETPOINT increments the legacy accepted/applied sequence: the scan record supplies local provenance instead. While local ownership and a live capture heartbeat are present, local control refreshes the original communication-age counter; after heartbeat expiry it no longer does so. Existing watchdog/fault behavior can then disable output. Zero VEL does not imply a mechanical brake or verified position hold.

## Scan STATUS / streaming record

Every captured base sample has an additional FD64+BRS record at 0x6F3, type 0x91, if a scan was staged. The shared bounded queue preserves base/extension pairing; it sends base first, extension second, and in legacy mode only advances the slot after both are accepted by transport. Reliable mode releases storage only after host acknowledgement. Drop counters still count sample slots, not individual bus frames. The host must detect missing pairs separately. Starting an ordinary capture after legacy takeover clears unstaged previous scan configuration.

| Offset | Field |
| --- | --- |
| 0–3 | version 1, type 0x83/0x91, node, phase |
| 4 | u32 request sequence (STATUS) or base sample sequence (stream) |
| 8 | u32 endpoint/control tick |
| 12 | u32 capture session |
| 16 | u32 scan session |
| 20 | u16 completed sweep legs; preposition/return excluded |
| 22 | reserved u16 |
| 24 | float planned raw position |
| 28 | float local commanded raw velocity |
| 32 | float instantaneous Iq * output torque constant |
| 36 | float actual raw position |
| 40 | float home |
| 44 | float current leg target |
| 48 | u32 scan start tick |
| 52 | u32 scan end tick, zero while running |
| 56–58 | reason, owned bool, active bool |
| 59–63 | reserved |

Join streaming extension with base telemetry by (capture session, sequence, endpoint tick). Use INFO torque constant and base instantaneous Iq for raw torque, averaged Iq/torque for anti-aliased analysis. Preserve raw payloads, encoder/Iq window markers, saturation/fault flags and all host/firmware loss diagnostics. The raw instantaneous value is not an anti-aliased 500 Hz torque sensor.

## Validation

Native scan tests simulate all legs/return, check slew/bounds, invalid configs and abort behavior. Native telemetry tests check config/START/STATUS, paired wire identity and external takeover. Existing telemetry fixture/filter tests remain passing. C++ SDK tests cover config ACK matching, START ACK loss reconciliation, foreign abort rejection, streaming decode and invalid phase. Python host mock tests verify eight-axis setup/hold, capture/scan/logging and cleanup. The three opt-in profiles cross-compile; baseline DM4310/DM4340/DM8009 APP hashes are checked unchanged. No motor, bus-load, ISR WCET, real stopping-distance, encoder calibration or flashing test has been executed.

## Reliable capture extension

START argument 0/1 retains legacy best-effort 500/1000 Hz behavior. Argument 2/3 requests reliable 500/1000 Hz capture. Older firmware rejects these arguments; hosts must never fall back silently. Base and scan wire layouts are unchanged.

The firmware retains up to 127 complete sample slots in a 128-slot ring (128 bytes/slot including extension). At 1000 Hz this gives nominally 127 ms of unacknowledged retention; at 500 Hz 254 ms. Sampling remains fixed to control ticks. Transmission runs in the main loop, not the current IRQ. Fully submitted frames advance a send cursor, not the retention tail. When all pending frames have been submitted but remain unacknowledged, a 20 ms timer restarts transmission from the retention tail. Replays preserve original bytes/timestamps. Cumulative ACKs can release several slots at once, and can be retried idempotently after lost replies.

- Operation 19 DATA_ACK, argument 0: payload32 = capture_session XOR next_sequence. next_sequence is the exclusive contiguous durable prefix (ACK 8 confirms sequences 0..7). Firmware rejects counts beyond its fully submitted high-water mark and treats old acknowledgements as no-ops. Half-submitted scan pairs are not ACKable.
- Operation 20 DATA_REPLAY, argument 0: payload32 = capture_session XOR first_missing_sequence. Firmware restarts the send cursor at this retained sequence. Already released or unsent ranges are rejected.

The token is a 32-bit random-session attribution fence, not authentication. One host writer per node/bus is required. Sessions here are bounded well below sequence wrap.

SDK reliable receive buffers reorder partial pairs, checks equal tick/session, and delivers only chronological contiguous samples once. Ordinary non-scan capture requires only the base sample. Scan capture requires both frames. Received duplicates are ignored; repaired losses do not create permanent sequence gaps. No automatic data ACK is issued by the SDK: the application must persist first, then acknowledge. Host queue saturation blocks release; it must not acknowledge unwritten data.

Before retention fills (127 slots), firmware stops capture with STATUS stop_reason=4 and requests scan abort reason=9; it preserves already generated data. Passive capture does not itself command the motor to stop: the host must stop/disable its command loop when capture stops. All new sysid nodes additionally require the independent guard and 250 ms command/scan lease described in SYSTEM_IDENTIFICATION_PROTOCOL.md §10. STOP does not discard retained data or overwrite an earlier abnormal stop reason. After STOP, replay/ACK remain available. The host reconciles next_sample_sequence with the durable/ACKed prefix; a successful dataset requires equality, no firmware sample drops, and normal STOP. Buffer exhaustion is a failed experiment even if its retained prefix is fully recovered.

## Scan current command cap

While the trajectory owns control, the existing 20 kHz current reference clamp also applies min(profile current limit, configured torque_limit / output_torque_constant). Invalid torque constant fails closed to zero reference. The measured Iq-derived torque threshold remains an independent abort check. This is a current-reference cap, not a calibrated hard shaft-torque guarantee; controller overshoot, gravity, conversion accuracy and real stopping distance remain unverified. The independent guard additionally caps current in every guarded holder/test node and mode; it does not persist EEPROM settings.

## Python usage

```python
capture.configure_scan(config)  # only for a paired scan session
started = capture.start(500, reliable=True)  # or 1000; no silent legacy fallback
# Receive/persist complete chronological pairs; ACK only a durable prefix:
capture.acknowledge_data(next_sequence)
# Optional explicit recovery; automatic firmware replay also remains active:
capture.replay_data(first_missing_sequence)
```

Normal non-scan reliable capture omits configure_scan. Applications must keep capture heartbeat alive, stop motor output separately, drain after STOP, and compare final next_sample_sequence with the durable acknowledged count. This SDK never fsyncs application files and never automatically acknowledges their persistence.

## Independent protection prerequisite

Configure and read back the independent guard on all eight nodes before enable. Initialize with mechanical/low-speed limits, then atomically tighten to the test/holder envelope while POSVEL holds the pose. Use a separate 0x6E0 live socket/thread for newest-state monitoring; reliable 0x6F2/0x6F3 recording may wait for replay without owning that path. See SYSTEM_IDENTIFICATION_PROTOCOL.md §10 for operation 21–28, fields and latch/reset semantics.
