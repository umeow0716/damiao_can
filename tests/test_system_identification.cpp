#include <cmath>
#include <cstring>
#include <damiao_can/damiao_motor/dm_motor.hpp>
#include <damiao_can/sysid/system_identification.hpp>
#include <functional>
#include <iostream>

using namespace damiao_can::sysid;

namespace {
void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
template <typename Error, typename Function>
void expect_error(Function function) {
    try {
        function();
    } catch (const Error&) {
        return;
    }
    throw std::runtime_error("expected exception was not thrown");
}
void put_u32(Frame& frame, size_t offset, uint32_t value) {
    for (size_t i = 0; i < 4; ++i) frame.payload[offset + i] = uint8_t(value >> (8 * i));
}
void put_float(Frame& frame, size_t offset, float value) {
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    put_u32(frame, offset, bits);
}
uint32_t request_sequence(const std::array<uint8_t, 8>& request) {
    return uint32_t(request[4]) | uint32_t(request[5]) << 8 | uint32_t(request[6]) << 16 |
           uint32_t(request[7]) << 24;
}
Frame sample_fixture() {
    return {sample_id,
            true,
            CANFD_BRS,
            {0x01, 0x90, 0x01, 0x09, 0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x01,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x3F, 0x00, 0x00,
             0x00, 0xC0, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0x80,
             0xBF, 0x00, 0x00, 0x40, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x41,
             0x28, 0x00, 0x00, 0x00, 0x27, 0x00, 0x00, 0x00, 0x28, 0x00, 0x01, 0x00},
            123456789};
}
Frame sample(uint32_t sequence, uint32_t tick, uint16_t period = 40) {
    Frame frame = sample_fixture();
    put_u32(frame, 4, sequence);
    put_u32(frame, 8, tick);
    frame.payload[60] = uint8_t(period);
    frame.payload[61] = uint8_t(period >> 8);
    return frame;
}
Frame ack(Operation operation, uint32_t sequence, Result result = Result::ACCEPTED) {
    Frame frame{
        response_id, false, 0, {1, 0x80, uint8_t(operation), uint8_t(result), 0, 0, 0, 0}, 20};
    put_u32(frame, 4, sequence);
    return frame;
}
Frame information(Operation operation, uint32_t sequence, uint32_t session, bool active,
                  uint32_t tick = 0) {
    Frame frame{response_id, true, CANFD_BRS, std::vector<uint8_t>(64), 10};
    frame.payload[0] = 1;
    frame.payload[1] = operation == Operation::INFO ? 0x81 : 0x82;
    frame.payload[2] = 1;
    frame.payload[3] = active;
    put_u32(frame, 4, sequence);
    put_u32(frame, 8, session);
    put_u32(frame, 12, tick);
    put_float(frame, 32, 0.5f);
    put_float(frame, 36, 10);
    put_float(frame, 40, 0.8f);
    put_float(frame, 44, 0.015465039f);
    put_float(frame, 48, 20);
    frame.payload[60] = active || session ? 40 : 0;
    frame.payload[62] = 1;
    return frame;
}
struct Mock : Transport {
    std::deque<Frame> frames;
    std::vector<std::array<uint8_t, 8>> sent;
    std::function<void(const std::array<uint8_t, 8>&)> responder;
    uint32_t session = 0;
    uint32_t tick = 0;
    bool active = false;
    bool lose_start_ack = false;
    bool wrong_recovery_session = false;
    Result start_result = Result::ACCEPTED;
    int destroyed = 0;
    int* destruction_count = nullptr;
    ~Mock() override {
        if (destruction_count) ++*destruction_count;
    }
    void send(const std::array<uint8_t, 8>& payload) override {
        sent.push_back(payload);
        if (responder) {
            responder(payload);
            return;
        }
        const auto operation = static_cast<Operation>(payload[2]);
        const auto sequence = request_sequence(payload);
        if (operation == Operation::START) {
            if (start_result == Result::ACCEPTED) {
                active = true;
                session = sequence;
            }
            if (!lose_start_ack) frames.push_back(ack(operation, sequence, start_result));
        } else if (operation == Operation::INFO || operation == Operation::STATUS) {
            auto frame =
                information(operation, sequence,
                            wrong_recovery_session && active ? session + 1 : session, active, tick);
            put_u32(frame, 16, active ? 0 : 7);
            frames.push_back(frame);
        } else {
            if (operation == Operation::STOP) active = false;
            frames.push_back(ack(operation, sequence));
        }
    }
    std::optional<Frame> receive(int) override {
        if (frames.empty()) return std::nullopt;
        auto result = frames.front();
        frames.pop_front();
        return result;
    }
};

void test_fixture_and_validation() {
    const Frame frame = sample_fixture();
    const auto decoded = decode_sample(frame);
    check(decoded.version == 1 && decoded.node == 1 && decoded.flags == 9 &&
              decoded.sequence == 0 && decoded.endpoint_tick == 40,
          "sample prefix");
    check(decoded.applied_command_sequence == 1 && decoded.sample_drop_count == 0,
          "sample counters");
    check(decoded.position == 1.25 && decoded.velocity == -2 && decoded.averaged_iq == 3 &&
              decoded.torque_estimate == 1.5 && decoded.mit_feedforward_torque == -1 &&
              decoded.instantaneous_iq == 3,
          "sample float layout");
    check(decoded.applied_command_tick == 1 && decoded.temperature == 30 &&
              decoded.velocity_tick == 40 && decoded.position_publication_tick == 39 &&
              decoded.interval_ticks == 40 && decoded.mode == 1 && decoded.fault == 0,
          "sample suffix");
    check(decoded.armed() && decoded.warmup() && !decoded.command_unknown() &&
              !decoded.current_saturated() && !decoded.voltage_saturated(),
          "flags");
    check(decoded.raw.payload == frame.payload && decoded.raw.host_receive_time_ns == 123456789,
          "raw bytes timestamp");
    for (const auto flag : {CAN_EFF_FLAG, CAN_RTR_FLAG, CAN_ERR_FLAG}) {
        auto invalid = frame;
        invalid.can_id |= flag;
        expect_error<ProtocolError>([&] { decode_sample(invalid); });
    }
    for (uint32_t id : {response_id, 0x123U}) {
        auto invalid = frame;
        invalid.can_id = id;
        expect_error<ProtocolError>([&] { decode_sample(invalid); });
    }
    for (size_t length : {0U, 8U, 63U, 65U}) {
        auto invalid = frame;
        invalid.payload.resize(length);
        expect_error<ProtocolError>([&] { decode_sample(invalid); });
    }
    for (uint8_t flags : {0, CANFD_ESI, CANFD_BRS | CANFD_ESI, 0x80}) {
        auto invalid = frame;
        invalid.flags = flags;
        expect_error<ProtocolError>([&] { decode_sample(invalid); });
    }
    auto fdf = frame;
    fdf.flags |= CANFD_FDF;
    decode_sample(fdf);
    for (const auto& [offset, value] : {std::pair<size_t, uint8_t>{0, 2},
                                        {1, 0x81},
                                        {2, 0},
                                        {2, 9},
                                        {3, 0x80},
                                        {60, 0},
                                        {62, 0},
                                        {62, 5}}) {
        auto invalid = frame;
        invalid.payload[offset] = value;
        expect_error<ProtocolError>([&] { decode_sample(invalid); });
    }
    auto classic = frame;
    classic.fd = false;
    expect_error<ProtocolError>([&] { decode_sample(classic); });
    for (size_t offset : {20U, 24U, 28U, 32U, 36U, 40U, 48U}) {
        for (uint32_t bits : {0x7FC00000U, 0x7F800000U, 0xFF800000U}) {
            auto invalid = frame;
            put_u32(invalid, offset, bits);
            expect_error<ProtocolError>([&] { decode_sample(invalid); });
        }
    }
    auto unknown = frame;
    unknown.payload[3] |= 16;
    put_u32(unknown, 12, UINT32_MAX);
    decode_sample(unknown);
    auto bad_unknown = frame;
    bad_unknown.payload[3] |= 16;
    expect_error<ProtocolError>([&] { decode_sample(bad_unknown); });
    const auto request = encode_request(1, Operation::START, 0, 0x12345678);
    check(request == std::array<uint8_t, 8>{1, 1, 1, 0, 0x78, 0x56, 0x34, 0x12},
          "request wire layout");
    check(command_sequence_distance(0xFFFFFFFE, 0) == 1 && command_sequence_distance(4, 7) == 3,
          "command skipped reserved wrap");
    expect_error<std::invalid_argument>([] { command_sequence_distance(UINT32_MAX, 1); });
    expect_error<std::invalid_argument>([] { encode_request(1, Operation::STOP, 1, 0); });
    const auto a = decode_ack(ack(Operation::START, 0x12345678, Result::BUSY));
    check(a.operation == Operation::START && a.result == Result::BUSY &&
              a.request_sequence == 0x12345678,
          "ACK layout");
    auto fd_ack = ack(Operation::START, 1);
    fd_ack.fd = true;
    expect_error<ProtocolError>([&] { decode_ack(fd_ack); });
    auto info = information(Operation::INFO, 9, 8, true, 123);
    put_u32(info, 16, 2);
    put_u32(info, 20, 3);
    put_u32(info, 24, 4);
    put_u32(info, 28, 5);
    put_u32(info, 52, 6);
    put_u32(info, 56, 7);
    info.payload[61] = 2;
    info.payload[63] = 8;
    const auto i = decode_information(info);
    check(i.version == 1 && i.message_type == 0x81 && i.node == 1 && i.active &&
              i.request_sequence == 9 && i.session_id == 8 && i.control_tick == 123,
          "info prefix");
    check(i.sample_drop_count == 2 && i.request_drop_count == 3 && i.tx_retry_count == 4 &&
              i.next_sample_sequence == 5 && i.accepted_command_sequence == 6 &&
              i.applied_command_sequence == 7,
          "info counters");
    check(i.output_torque_constant == 0.5 && i.gear_ratio == 10 &&
              i.factory_velocity_previous_weight == 0.8f && i.iq_filter_beta == 0.015465039f &&
              i.current_limit == 20 && i.sample_period_ticks == 40 && i.stop_reason == 2 &&
              i.mode == 1 && i.fault == 8,
          "info suffix");
    for (size_t offset : {32U, 36U, 40U, 44U, 48U}) {
        auto invalid = info;
        put_u32(invalid, offset, 0x7F800000);
        expect_error<ProtocolError>([&] { decode_information(invalid); });
    }
}

void test_management_demux() {
    auto mock = std::make_unique<Mock>();
    auto* transport = mock.get();
    SystemIdentification api(std::move(mock), 1, 0.5, true, 8);
    const auto started = api.start();
    check(started.accepted && !started.recovered && api.owned_session() == started.session_id,
          "start owned");
    transport->responder = [&](const auto& request) {
        const auto op = static_cast<Operation>(request[2]);
        const auto sequence = request_sequence(request);
        transport->frames.push_back(sample(0, 40));
        transport->frames.push_back(ack(Operation::STOP, sequence));  // Wrong op, preserved.
        transport->frames.push_back(ack(op, sequence - 1));           // Wrong sequence, preserved.
        auto wrong_node = information(Operation::STATUS, sequence, started.session_id, true, 40);
        wrong_node.payload[2] = 2;
        transport->frames.push_back(wrong_node);
        auto wrong_type = information(Operation::INFO, sequence, started.session_id, true, 40);
        transport->frames.push_back(wrong_type);
        transport->frames.push_back(sample(1, 80));
        if (op == Operation::HEARTBEAT)
            transport->frames.push_back(ack(op, sequence));
        else
            transport->frames.push_back(information(op, sequence, started.session_id, true, 80));
        transport->frames.push_back(Frame{0x123, false, 0, {1, 2, 3}, 10});
    };
    check(api.heartbeat().accepted(), "interleaved heartbeat ACK");
    auto rows = api.read_samples();
    check(rows.size() == 2 && rows[0].sample.sequence == 0 && rows[1].sample.sequence == 1,
          "samples preserved while waiting ACK");
    check(rows[0].position == -0.75 && rows[0].velocity == 2 && rows[0].averaged_iq == -3 &&
              rows[0].torque_estimate == -1.5 && rows[0].mit_feedforward_torque == 1 &&
              rows[0].instantaneous_iq == -3,
          "coordinate transform");
    struct TestMotor : damiao_can::damiao_motor::Motor {
        using Motor::Motor;
        using Motor::update_state;
    };
    TestMotor motor(damiao_can::damiao_motor::MotorType::DM4310, 1, 17, 0.5, true);
    motor.update_state(1.25, -2, 1.5, 0, 0);
    check(rows[0].position == motor.get_position() && rows[0].velocity == motor.get_velocity() &&
              rows[0].torque_estimate == motor.get_torque(),
          "legacy coordinate consistency");
    check(rows[0].sample.position == 1.25 && rows[0].sample.raw.payload == sample_fixture().payload,
          "wire untransformed");
    check(api.take_replies().size() == 4 && api.take_other_frames().size() == 1,
          "nonmatching replies and others retained");
    transport->responder = [&](const auto& request) {
        transport->frames.push_back(
            information(Operation::INFO, request_sequence(request), started.session_id, true, 80));
    };
    check(api.info().information.has_value(), "INFO completes without extra ACK");
    transport->responder = {};
    transport->active = true;
    transport->tick = 80;
    check(api.status().information.has_value(), "STATUS completes without extra ACK");
}

void test_errors_and_start_recovery() {
    for (auto result :
         {Result::BAD_VERSION, Result::BAD_ARGUMENT, Result::BUSY, Result::TRANSPORT_NOT_READY,
          Result::UNSUPPORTED_OPERATION, Result::MEASUREMENTS_NOT_READY}) {
        auto mock = std::make_unique<Mock>();
        mock->start_result = result;
        SystemIdentification api(std::move(mock), 1);
        const auto rejected = api.start();
        check(!rejected.accepted && rejected.reply.ack->result == result && !api.owned_session(),
              "rejected start results exposed");
    }
    auto mock = std::make_unique<Mock>();
    auto* transport = mock.get();
    mock->lose_start_ack = true;
    SystemIdentification api(std::move(mock), 1);
    const auto recovered = api.start(500, 10000);
    check(recovered.accepted && recovered.recovered && recovered.recovery_status->active &&
              recovered.recovery_status->session_id == recovered.session_id,
          "lost start ACK status recovery");
    check(transport->sent.size() == 3 && transport->sent[0][2] == 5 && transport->sent[1][2] == 1 &&
              transport->sent[2][2] == 5,
          "START never resent");
    transport->responder = [](const auto&) {};
    expect_error<TimeoutError>([&] { api.heartbeat(1000); });
    auto other = std::make_unique<Mock>();
    auto* peer = other.get();
    other->lose_start_ack = true;
    other->wrong_recovery_session = true;
    SystemIdentification conflict(std::move(other), 1);
    expect_error<SessionError>([&] { conflict.start(500, 1000); });
    check(peer->sent.size() == 3 && !conflict.owned_session(), "ambiguous start no retry no stop");
    expect_error<SessionError>([&] { conflict.stop(); });
    check(peer->sent.size() == 3, "STOP forbidden without ownership");
    auto busy = std::make_unique<Mock>();
    auto* foreign = busy.get();
    busy->active = true;
    busy->session = 20;
    SystemIdentification blocked(std::move(busy), 1);
    expect_error<SessionError>([&] { blocked.start(); });
    check(foreign->sent.size() == 1 && foreign->sent[0][2] == 5,
          "preflight protects active foreign session");
}

void test_boundaries_wrap_overflow_stop() {
    auto mock = std::make_unique<Mock>();
    auto* transport = mock.get();
    transport->tick = 0xFFFFFF00;
    SystemIdentification api(std::move(mock), 1, 0, false, 2);
    auto started = api.start();
    transport->frames.push_back(sample(0xFFFFFFFE, 0xFFFFFF50));
    transport->frames.push_back(sample(0xFFFFFFFF, 0xFFFFFF78));
    api.poll();
    auto rows = api.read_samples();
    check(rows.size() == 2, "pre-wrap rows");
    transport->frames.push_back(sample(0, 0xFFFFFFA0));
    transport->frames.push_back(sample(2, 0x10));
    transport->frames.push_back(sample(3, 0x38));
    api.poll();
    rows = api.read_samples();
    check(rows.size() == 2 && rows[0].unwrapped_sequence == 0x100000000ULL &&
              rows[1].unwrapped_endpoint_tick == 0x100000010ULL && rows[1].sequence_gap == 1,
          "u32 wraps gap");
    check(api.diagnostics().host_sample_dropped == 1, "bounded drop-new queue");
    transport->tick = 0x40;
    transport->frames.push_back(sample(4, 0x60));
    auto stopped = api.stop(10000, 1000, 1);
    check(stopped.reply.accepted() && stopped.final_status.sample_drop_count == 7 &&
              !stopped.final_status.active && stopped.samples.size() == 1,
          "STOP drain final firmware drops");
    // Delayed old sample below next START fence must not contaminate next capture.
    transport->tick = 0x100;
    transport->responder = [&](const auto& request) {
        const auto seq = request_sequence(request);
        const auto op = static_cast<Operation>(request[2]);
        if (op == Operation::STATUS)
            transport->frames.push_back(
                information(op, seq, transport->session, false, transport->tick));
        if (op == Operation::START) {
            transport->session = seq;
            transport->active = true;
            transport->frames.push_back(sample(5, 0x80));   // previous session leftover
            transport->frames.push_back(sample(0, 0x128));  // new sample arrived before ACK
            transport->frames.push_back(ack(op, seq));
        }
    };
    const auto next = api.start();
    rows = api.read_samples();
    check(next.accepted && rows.size() == 1 && rows[0].session_id == next.session_id &&
              rows[0].sample.sequence == 0 &&
              rows[0].timeline_epoch != stopped.samples[0].timeline_epoch,
          "session fence new epoch");
    check(api.diagnostics().session_discarded_samples >= 1, "stale sample discard visible");
    transport->frames.push_back(sample(1, 0x150));
    api.poll();
    api.read_samples();
    transport->frames.push_back(sample(0, 40));
    api.poll();
    check(!api.owned_session() && api.read_samples().empty(),
          "reboot backward counter invalidates attribution");
}

void test_queue_replies_and_cleanup() {
    int destructions = 0;
    auto mock = std::make_unique<Mock>();
    auto* transport = mock.get();
    mock->destruction_count = &destructions;
    SystemIdentification api(std::move(mock), 1, 0, false, 1);
    transport->frames.push_back(ack(Operation::INFO, 1));
    transport->frames.push_back(ack(Operation::INFO, 2));
    transport->frames.push_back(Frame{5, false, 0, {}, 0});
    transport->frames.push_back(Frame{6, false, 0, {}, 0});
    api.poll();
    check(api.diagnostics().host_reply_dropped == 1 && api.diagnostics().host_other_dropped == 1,
          "bounded all receive queues");
    transport->frames.push_back(sample_fixture());
    transport->frames.front().payload[0] = 2;
    expect_error<ProtocolError>([&] { api.poll(); });
    check(api.diagnostics().invalid_frames == 1, "invalid frame surfaced counted");
    api.close();
    api.close();
    check(destructions == 1, "close resources idempotent");
    expect_error<damiao_can::canbus::CANSocketException>([&] { api.info(); });
}

void test_stop_drain_and_foreign_ownership() {
    auto mock = std::make_unique<Mock>();
    auto* transport = mock.get();
    SystemIdentification api(std::move(mock), 1);
    const auto started = api.start();
    bool stopped = false;
    transport->responder = [&](const auto& request) {
        const auto op = static_cast<Operation>(request[2]);
        const auto sequence = request_sequence(request);
        if (op == Operation::STOP) {
            stopped = true;
            transport->frames.push_back(ack(op, sequence));
            transport->frames.push_back(sample(0, 40));
            transport->frames.push_back(sample(1, 80));
        } else {
            auto response = information(op, sequence, started.session_id, !stopped, 120);
            put_u32(response, 16, stopped ? 9 : 3);
            if (stopped) transport->frames.push_back(sample(2, 120));
            transport->frames.push_back(response);
        }
    };
    const auto drained = api.stop(10000, 10000, 1);
    check(drained.drain_limit_reached && drained.samples.size() == 3,
          "bounded drain keeps queued samples and final STATUS interleaving");
    check(drained.final_status.sample_drop_count == 9 && !drained.final_status.active,
          "final STATUS counters distinct from earlier sample snapshots");
    const auto sent_before = transport->sent.size();
    transport->responder = [&](const auto& request) {
        transport->frames.push_back(information(Operation::STATUS, request_sequence(request),
                                                started.session_id + 1, true, 160));
    };
    expect_error<SessionError>([&] { api.stop(); });
    check(transport->sent.size() == sent_before + 1 &&
              transport->sent.back()[2] == static_cast<uint8_t>(Operation::STATUS),
          "ownership change checked before STOP, foreign session not stopped");
}

void test_fast_rate_and_inactive_recovery() {
    auto mock = std::make_unique<Mock>();
    auto* transport = mock.get();
    SystemIdentification api(std::move(mock), 1);
    const auto started = api.start(1000);
    check(started.accepted && transport->sent.back()[3] == 1, "1 kHz START argument");
    transport->frames.push_back(sample(0, 20, 20));
    auto rows = api.read_samples();
    check(rows.size() == 1 && rows[0].sample.interval_ticks == 20, "1 kHz interval accepted");

    auto lost = std::make_unique<Mock>();
    auto* peer = lost.get();
    SystemIdentification recovery(std::move(lost), 1);
    uint32_t accepted_session = 0;
    peer->responder = [&](const auto& request) {
        const auto op = static_cast<Operation>(request[2]);
        if (op == Operation::START) {
            accepted_session = request_sequence(request);
            peer->frames.push_back(sample(0, 40));
        } else {
            auto response = information(op, request_sequence(request), accepted_session, false,
                                        accepted_session ? 80 : 0);
            response.payload[61] = accepted_session ? 2 : 0;
            peer->frames.push_back(response);
        }
    };
    const auto recovered = recovery.start(500, 1000);
    check(recovered.accepted && recovered.recovered && !recovered.recovery_status->active &&
              recovered.recovery_status->stop_reason == 2,
          "lost ACK accepted session already inactive is exposed, not retried");
    check(recovery.read_samples().size() == 1 && peer->sent.size() == 3,
          "pending samples retained during recovery and START transmitted once");
}
}  // namespace

int main() {
    try {
        test_fixture_and_validation();
        test_management_demux();
        test_errors_and_start_recovery();
        test_boundaries_wrap_overflow_stop();
        test_queue_replies_and_cleanup();
        test_stop_drain_and_foreign_ownership();
        test_fast_rate_and_inactive_recovery();
        std::cout << "sysid offline protocol/management/session tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
