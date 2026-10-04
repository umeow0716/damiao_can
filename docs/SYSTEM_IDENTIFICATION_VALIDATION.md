# Host telemetry v1 offline verification

Date: 2026-10-04. Branch: `feat/dm4310-sysid-telemetry`.
Source basis includes the existing offset and reversed host-coordinate implementation.
No hardware control or flashing was performed. The current branch also integrates the matching independent guard/live firmware and damiao_ws runtime.

## Reproduce

From the damiao_can repository root:

```sh
cmake -S . -B build/sysid -GNinja -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build/sysid
ctest --test-dir build/sysid --output-on-failure
cmake --install build/sysid --prefix "$PWD/build/sysid-install"

uv venv --python 3.12 build/sysid-venv
uv pip install --python build/sysid-venv/bin/python ./python
build/sysid-venv/bin/python tests/test_sysid_binding.py -v

cmake -S . -B build/sysid-asan -GNinja -DBUILD_TESTING=ON \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_FLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer' \
    -DCMAKE_EXE_LINKER_FLAGS='-fsanitize=address,undefined'
cmake --build build/sysid-asan
ctest --test-dir build/sysid-asan --output-on-failure
```

An ordinary Python virtual environment and pip installation can replace uv. The tests import the installed package, not a mock Python decoder. Build/install outputs stay inside ignored build/ directories. The fixture/transport tests never open a real CAN interface: the production CANSocket setup is substituted with local AF_UNIX datagram socketpairs.

## Results

| Check | Result |
| --- | --- |
| C++ Debug build (GCC 14.2) and local header/library install | Passed |
| CTest protocol/management/session executable | Passed |
| CTest actual CANSocket/mixed-frame/legacy-control executable | Passed |
| AddressSanitizer + UndefinedBehaviorSanitizer build and both CTest executables | Passed |
| Python 3.12.12 wheel build/install | Passed |
| Five Python binding fixture/validation/export test methods | Passed |
| Existing pre-commit formatting/check hooks, including new files explicitly | Passed |
| git diff --check | Passed |
| Hardware, bus load, timing, calibration, motor/holding safety | Not executed |

CTest logs are under `build/sysid/Testing/Temporary/LastTest.log` and `build/sysid-asan/Testing/Temporary/LastTest.log`. CI executes CTest and the Python offline binding tests after its existing builds. The results above are local verification results; remote CI results are recorded separately in GitHub Actions.

## Coverage and boundaries

- The firmware document's complete 64-byte sample fixture is checked field by field in C++ and through the compiled Python binding. INFO/STATUS fields, ACK8, request encoding, raw payload, host timestamp, flags, and filter/scaling fields are checked.
- Decoders reject invalid ID/extended/RTR/error indicators, version/type, classic/FD mismatch, lengths/FD flags, invalid node/period/mode/unknown-command consistency, and NaN/Inf measurements.
- Fake transport management tests interleave sample, ACK, INFO/STATUS, wrong operation/sequence/node/type, and unrelated messages. Samples survive ACK waits; unmatched messages have bounded separate queues.
- Tests cover typed firmware rejection results, timeout, START ACK loss reconciled by STATUS, inactive recovered capture, already-active foreign capture, ownership changes before STOP, pending samples before START ACK, and no blind START retry or foreign STOP.
- Session tests cover stale old samples, new timeline epochs, observable reboot/counter rollback, u32 sample/tick wrap, received sequence gap, host drop-new overflow, unmatched/other queue drops, 500 Hz/1 kHz periods, STOP bounded drain and final STATUS/drop counters.
- Coordinate tests retain raw wire values and compare adjusted position/velocity/torque with existing Motor offset/reversed behavior.
- Local socketpair tests exercise real mixed CAN_MTU/CANFD_MTU reads and fd cleanup. Actual legacy control/mode send paths are checked to perform zero receive syscalls when no feedback is supplied; legacy recv_all ignores ACK/sample IDs and accepts explicit motor feedback.
- Close is resource-only and idempotent; management access after close fails explicitly. The default destructor has no capture STOP or motor-control operation.

Kernel SO_RXQ_OVFL reporting is implemented but real CAN/kernel queue overflow has not been induced. Thread serialization and GIL-release behavior are part of the implementation; real-time scheduling and loaded multi-thread execution have not been established by these tests. V1 has no sample session/boot ID, so indistinguishable resets and concurrent external writers cannot be completely disambiguated. Drain quiet intervals/hard limits cannot prove all firmware/controller queues empty. These constraints are documented in [the API guide](SYSTEM_IDENTIFICATION.md).

The downstream migration plan is in [the damiao_ws handoff](DAMIAO_WS_SYSID_HANDOFF.md). No de-filtering, model fitting, excitation operation, hardware setup, or automatic motor shutdown is provided by this library change.

## Independent safety extension verification (2026-10-04)

New C++ tests cover live-session attribution, monotonic newest-only delivery across out-of-order frames, independence from canonical recording gaps, reverse/offset transformation, explicit MOS temperature, live-only heartbeat, stop fencing, safety-status reserved fields and exact guard configuration readback. The mixed SocketCAN boundary test additionally checks that actual holder fault 0xD survives decoding/callback into Motor.get_fault(). Python tests exercise the new decoder fields and exports. Both CTest executables pass under normal and address/undefined sanitizer builds.

The workspace separately exercises independent live monitoring during a blocked storage sink, stale/torque shutdown, per-node capability checks and joint-to-raw phase limits. These are offline software tests, not hardware bus/stop-time certification.

Follow-up legacy freshness tests reject truncated/oversized motor payloads before buffer access, extended/RTR aliases, and replies that did not update state. Explicit recv_all counts a motor only when its decoded-reply revision changes; valid parameter responses remain supported. Normal and sanitizer CTest exercise the boundary using local sockets and callback fixtures.
