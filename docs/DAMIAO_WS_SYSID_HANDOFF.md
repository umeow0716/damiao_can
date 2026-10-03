# Handoff: migrate damiao_ws to DM4310 sysid telemetry v1

This document is for the next agent modifying `../damiao_ws`. That project was inspected read-only for this change. Do not treat the library examples as authorization to run hardware.

The library branch is `feat/dm4310-sysid-telemetry`. Read [the API/lifecycle guide](SYSTEM_IDENTIFICATION.md) first. The implementation deliberately uses explicit polling and manual heartbeat; there are no background control or capture operations. C++ API is in `include/damiao_can/sysid/system_identification.hpp`; Python exports SystemIdentification, SysIdMeasurement, SysIdSample, SysIdInformation, SysIdReply, and related typed results.

Use `capture = dc.SystemIdentification(interface, motor_id, offset=offsets[motor_id-1], reversed=direction)`, `capture.info()`, `capture.start(500)`, `capture.read_samples(timeout_us=2000)`, explicit `capture.heartbeat()`, and `capture.stop()`. SysIdReply.accepted is a Python property; information/ack contain the corresponding typed response. SysIdMeasurement.sample preserves wire values; its direct position/velocity/torque_estimate fields are host-adjusted. SysIdStopResult.samples contains the drain and final_status records final counters. Host timestamps use C++ steady_clock's unspecified epoch, not a promise of Python monotonic_ns epoch compatibility.

## Existing integration points

`friction.py` owns the shared eight-axis `Rig`; `velocity.py` and `frf.py` reuse it. The inspected Rig initializes a test axis and seven holding axes, configures its interface via CANHelper, and uses `flush_rx -> control command -> recv_all` in startup, holding updates, sampling, and shutdown. Raw/no-response firmware does not emit automatic feedback after control commands, so this pattern must be replaced wherever the affected firmware is used.

Keep IDs 1–8, allow identification of IDs 1–7 only, and retain ID8 among the seven non-test holding axes. Their application-coordinate target remains POSVEL position 0.0 and maximum velocity 20.0. Continue using CANHelper for host interface configuration; do not introduce Python SocketCAN header parsing. Do not assume that enabling host FD configures motor FD or that every node on the bus is FD-compatible.

The library capture API is separate from motor control. Use existing DamiaoCAN/DamiaoCANGroup for explicit commands and explicit refresh/LIVE, and the independent capture receiver for telemetry. Do not wait for a response that the no-response variant does not generate. If holding/safety logic needs legacy feedback, issue explicit refresh and handle its bounded result separately. Do not use telemetry reception as evidence that a control command was accepted or that a motor is safe.

## Capture lifecycle and ownership

1. Confirm one capture writer on the bus. INFO records protocol, torque constant, gear ratio, filter coefficients, current limit, and sample period.
2. STATUS must establish the target is inactive before START. A successful START's host request sequence becomes the session ID.
3. Maintain motor control/watchdog traffic explicitly in the control loop. Independently call capture HEARTBEAT approximately every 100 ms. The 500 ms capture timeout is not motor shutdown.
4. Service telemetry often enough for the bounded host queue, including while waiting for management responses. Management waits demultiplex samples rather than discarding them.
5. STOP stops generating new samples only. Collect the bounded drain, then persist final STATUS/drop counters. Mark incomplete drains/timeouts in the log.
6. Perform explicitly chosen motor shutdown separately; closing capture does not disable motors or send zero torque.

A lost START ACK must be reconciled using STATUS and the requested session ID. Do not retry START blindly or STOP an active session owned by somebody else. Samples lack session/boot IDs; never merge captures across reconnects, suspicious counter resets, or session ownership changes. An ambiguous outcome must remain an error in the run metadata rather than a guessed success.

Management methods can wait up to their request timeout (and START/STOP use multiple requests). Keep those waits outside any thread responsible for meeting a 500 Hz control deadline or the motor watchdog. The API provides no scheduler: the host application must explicitly coordinate control, capture servicing, heartbeat, and shutdown. Multiple calls on the same capture object serialize, while a separately owned legacy control object remains independent.

## Logging schema

Save raw payload (64 bytes), raw wire values, management snapshots, and capture/session identity. Also save application-coordinate position, velocity, and torque as a separate view, with the exact host offset/reversed settings. Do not apply offset/direction twice, and do not multiply gear ratio into the already output-side torque estimate.

Preserve at least:

- Host receive timestamp and its documented clock source; host command transmission times remain separate.
- Raw and diagnosed sample sequence/firmware endpoint tick.
- Applied command sequence and consumption tick, including the unknown sentinel 0xFFFFFFFF.
- Velocity tick, position publication marker, interval ticks.
- Averaged Iq, instantaneous Iq, estimated torque, MIT feedforward torque, temperature.
- Mode, fault, armed, warmup, current/voltage saturation, command unknown.
- Firmware sample/request drop counters and tx retries, host queue loss, and any kernel loss diagnostics exposed by the transport.
- INFO output torque constant, gear ratio, factory velocity filter coefficient, telemetry beta, sample period; initial and final STATUS.

Legacy accepted/applied command sequence skips 0xFFFFFFFF. It is neither a host command tag nor ordinary modulo 2^32. Exact host-to-command association needs a single writer, a STATUS baseline, a record of every accepted SETPOINT, and explicit handling of missed commands/reboot. Applied sequence indicates control consumption, not a proven physical current response.

## Analysis changes

`friction.py`: breakaway velocity is a windowed measurement, while torque is filtered Iq times torque constant with roughly 13 ms low-frequency delay. Save the entire ramp and timing fields. Do not use the same row's first velocity crossing as an automatically time-aligned friction estimate.

`velocity.py`: exclude transients, warmup, fault/mode changes, saturation, unknown command provenance, and gaps. Preserve raw platform data and calibration inputs. Steady averages reduce sensitivity to filter delay but do not validate torque calibration.

`frf.py`: current `fit_rows` constructs measurement time from host receive timestamps. The new mechanical response timeline must instead use firmware endpoint tick and signal-specific windows/publication markers, with host timestamps retained for transport diagnostics. Keep feedback torque→velocity/position and command→feedback torque as separate estimates.

If reported torque is H_iq times true electrical torque and reported velocity is H_velocity times true velocity, then:

```text
G_report = G_true * H_velocity / H_iq
G_true = G_report * H_iq / H_velocity
```

H_iq is four cascaded 20 kHz IIR stages (beta from INFO), followed by an N=20/40 average. Its gain near 20 Hz is about 0.738–0.739. Telemetry velocity does not use the INFO factory LPF coefficient; it uses encoder difference/average windows. Publication/acquisition delays still require measurement. Keep raw and corrected analysis separately; constrain compensation by coherence/SNR rather than boosting noisy frequencies blindly. Instantaneous Iq is diagnostic, not an anti-aliased substitute.

## Remaining hardware work

Library offline verification cannot establish device FD configuration, loaded-bus performance, ISR timing, actual control frequency, encoder delay, current calibration, physical response, original motor watchdog safety, or holding/shutdown behavior. The firmware validation document lists the pending hardware checks. Perform them only under separately authorized safe procedures.
