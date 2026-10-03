#include <sys/socket.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <damiao_can/sysid/system_identification.hpp>
#include <limits>
#include <random>

namespace damiao_can::sysid {
namespace {

using Clock = std::chrono::steady_clock;
uint64_t timestamp() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
        .count();
}

int remaining_us(Clock::time_point deadline) {
    const auto remaining =
        std::chrono::duration_cast<std::chrono::microseconds>(deadline - Clock::now());
    return static_cast<int>(std::max<int64_t>(0, remaining.count()));
}

uint32_t u32(const std::vector<uint8_t>& data, size_t offset) {
    return uint32_t(data[offset]) | (uint32_t(data[offset + 1]) << 8) |
           (uint32_t(data[offset + 2]) << 16) | (uint32_t(data[offset + 3]) << 24);
}

float f32(const std::vector<uint8_t>& data, size_t offset) {
    const uint32_t bits = u32(data, offset);
    float value;
    static_assert(sizeof(value) == sizeof(bits) && std::numeric_limits<float>::is_iec559);
    std::memcpy(&value, &bits, sizeof(value));
    if (!std::isfinite(value))
        throw ProtocolError("non-finite sysid field at offset " + std::to_string(offset));
    return value;
}

void validate(const Frame& frame, uint32_t id, size_t length, bool fd) {
    if (frame.can_id != id) throw ProtocolError("expected standard sysid data frame ID");
    if (frame.payload.size() != length || frame.fd != fd)
        throw ProtocolError("invalid sysid frame format or payload length");
    // CANFD_FDF is metadata in newer SocketCAN kernels; ESI/error/reserved flags are rejected.
    if (fd ? ((frame.flags & ~(CANFD_BRS | CANFD_FDF)) || !(frame.flags & CANFD_BRS))
           : frame.flags != 0)
        throw ProtocolError("invalid sysid CAN-FD flags (FD64 requires BRS)");
    if (frame.payload[0] != 1) throw ProtocolError("unsupported sysid protocol version");
}

void validate_node(uint8_t node) {
    if (node < 1 || node > 8) throw ProtocolError("sysid node must be 1 through 8");
}

void validate_timeout(int timeout_us) {
    if (timeout_us < 0) throw std::invalid_argument("timeout_us must be non-negative");
}

uint32_t initial_sequence() {
    // Random session tags reduce accidental attribution to a previous host process.
    std::random_device entropy;
    return entropy();
}

class SocketTransport final : public Transport {
public:
    explicit SocketTransport(const std::string& interface) : socket_(interface, true) {
        const can_filter filters[] = {
            {response_id, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG},
            {sample_id, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG}};
        const int enabled = 1;
        if (::setsockopt(socket_.get_socket_fd(), SOL_CAN_RAW, CAN_RAW_FILTER, filters,
                         sizeof(filters)) < 0 ||
            ::setsockopt(socket_.get_socket_fd(), SOL_SOCKET, SO_RXQ_OVFL, &enabled,
                         sizeof(enabled)) < 0)
            throw canbus::CANSocketException(
                "failed to configure sysid receive filters/overflow reporting");
    }

    void send(const std::array<uint8_t, 8>& payload) override {
        can_frame frame{};
        frame.can_id = request_id;
        frame.can_dlc = 8;
        std::copy(payload.begin(), payload.end(), frame.data);
        ssize_t size;
        do {
            size = ::send(socket_.get_socket_fd(), &frame, sizeof(frame), MSG_DONTWAIT);
        } while (size < 0 && errno == EINTR);
        if (size != CAN_MTU)
            throw canbus::CANSocketException("failed nonblocking sysid management send: " +
                                             std::string(std::strerror(errno)));
    }

    std::optional<Frame> receive(int timeout_us) override {
        if (!socket_.is_data_available(timeout_us)) return std::nullopt;
        canfd_frame wire{};
        alignas(cmsghdr) char control[CMSG_SPACE(sizeof(uint32_t))]{};
        iovec vector{&wire, sizeof(wire)};
        msghdr message{};
        message.msg_iov = &vector;
        message.msg_iovlen = 1;
        message.msg_control = control;
        message.msg_controllen = sizeof(control);
        ssize_t size;
        do {
            size = ::recvmsg(socket_.get_socket_fd(), &message, MSG_DONTWAIT);
        } while (size < 0 && errno == EINTR);
        if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return std::nullopt;
        if (size < 0)
            throw canbus::CANSocketException("sysid recvmsg: " + std::string(std::strerror(errno)));
        const auto receive_time = timestamp();
        for (cmsghdr* header = CMSG_FIRSTHDR(&message); header;
             header = CMSG_NXTHDR(&message, header)) {
            if (header->cmsg_level == SOL_SOCKET && header->cmsg_type == SO_RXQ_OVFL &&
                header->cmsg_len >= CMSG_LEN(sizeof(uint32_t))) {
                uint32_t count;
                std::memcpy(&count, CMSG_DATA(header), sizeof(count));
                dropped_ += uint32_t(count - previous_dropped_);
                previous_dropped_ = count;
            }
        }
        if (size != CAN_MTU && size != CANFD_MTU)
            throw ProtocolError("invalid SocketCAN frame size");
        if ((message.msg_flags & (MSG_TRUNC | MSG_CTRUNC)) || wire.len > (size == CAN_MTU ? 8 : 64))
            throw ProtocolError("truncated SocketCAN frame");
        Frame frame;
        frame.can_id = wire.can_id;
        frame.fd = size == CANFD_MTU;
        frame.flags = frame.fd ? wire.flags : 0;
        frame.payload.assign(wire.data, wire.data + wire.len);
        frame.host_receive_time_ns = receive_time;
        return frame;
    }
    uint64_t socket_dropped() const override { return dropped_; }

private:
    canbus::CANSocket socket_;
    uint32_t previous_dropped_ = 0;
    uint64_t dropped_ = 0;
};

}  // namespace

Ack decode_ack(const Frame& frame) {
    validate(frame, response_id, 8, false);
    const auto& data = frame.payload;
    if (data[1] != 0x80 || data[2] < 1 || data[2] > 5 || data[3] > 6)
        throw ProtocolError("invalid sysid ACK type, operation or result");
    return {data[0], static_cast<Operation>(data[2]), static_cast<Result>(data[3]), u32(data, 4),
            frame};
}

Information decode_information(const Frame& frame) {
    validate(frame, response_id, 64, true);
    const auto& data = frame.payload;
    if ((data[1] != 0x81 && data[1] != 0x82) || data[3] > 1 ||
        (data[60] != 0 && data[60] != 20 && data[60] != 40) || data[61] > 3 || data[62] > 4)
        throw ProtocolError("invalid sysid INFO/STATUS fields");
    validate_node(data[2]);
    Information result;
    result.version = data[0];
    result.message_type = data[1];
    result.node = data[2];
    result.active = data[3];
    result.request_sequence = u32(data, 4);
    result.session_id = u32(data, 8);
    result.control_tick = u32(data, 12);
    result.sample_drop_count = u32(data, 16);
    result.request_drop_count = u32(data, 20);
    result.tx_retry_count = u32(data, 24);
    result.next_sample_sequence = u32(data, 28);
    result.output_torque_constant = f32(data, 32);
    result.gear_ratio = f32(data, 36);
    result.factory_velocity_previous_weight = f32(data, 40);
    result.iq_filter_beta = f32(data, 44);
    result.current_limit = f32(data, 48);
    result.accepted_command_sequence = u32(data, 52);
    result.applied_command_sequence = u32(data, 56);
    result.sample_period_ticks = data[60];
    result.stop_reason = data[61];
    result.mode = data[62];
    result.fault = data[63];
    result.raw = frame;
    return result;
}

Sample decode_sample(const Frame& frame) {
    validate(frame, sample_id, 64, true);
    const auto& data = frame.payload;
    const auto period = uint16_t(data[60]) | (uint16_t(data[61]) << 8);
    if (data[1] != 0x90 || (data[3] & 0xE0) || (period != 20 && period != 40) || data[62] < 1 ||
        data[62] > 4)
        throw ProtocolError("invalid sysid sample type, flags, period or mode");
    validate_node(data[2]);
    Sample result;
    result.version = data[0];
    result.node = data[2];
    result.flags = data[3];
    result.sequence = u32(data, 4);
    result.endpoint_tick = u32(data, 8);
    result.applied_command_sequence = u32(data, 12);
    result.sample_drop_count = u32(data, 16);
    result.position = f32(data, 20);
    result.velocity = f32(data, 24);
    result.averaged_iq = f32(data, 28);
    result.torque_estimate = f32(data, 32);
    result.mit_feedforward_torque = f32(data, 36);
    result.instantaneous_iq = f32(data, 40);
    result.applied_command_tick = u32(data, 44);
    result.temperature = f32(data, 48);
    result.velocity_tick = u32(data, 52);
    result.position_publication_tick = u32(data, 56);
    result.interval_ticks = period;
    result.mode = data[62];
    result.fault = data[63];
    result.raw = frame;
    if (result.command_unknown() != (result.applied_command_sequence == UINT32_MAX) ||
        (!result.armed() && !result.command_unknown()))
        throw ProtocolError("inconsistent sysid command-unknown flag");
    return result;
}

std::array<uint8_t, 8> encode_request(uint8_t node, Operation operation, uint8_t argument,
                                      uint32_t sequence) {
    validate_node(node);
    const uint8_t op = static_cast<uint8_t>(operation);
    if (op < 1 || op > 5 || (operation == Operation::START ? argument > 1 : argument != 0))
        throw std::invalid_argument("invalid sysid operation/argument");
    std::array<uint8_t, 8> data{1, node, op, argument, 0, 0, 0, 0};
    for (size_t i = 0; i < 4; ++i) data[4 + i] = static_cast<uint8_t>(sequence >> (8 * i));
    return data;
}

uint32_t command_sequence_distance(uint32_t previous, uint32_t current) {
    if (previous == UINT32_MAX || current == UINT32_MAX)
        throw std::invalid_argument("unknown legacy command sequence");
    return current >= previous ? current - previous
                               : uint32_t(uint64_t(UINT32_MAX) - previous + current);
}

SystemIdentification::SystemIdentification(const std::string& interface, uint8_t node,
                                           double offset, bool reversed, size_t capacity)
    : SystemIdentification(std::make_unique<SocketTransport>(interface), node, offset, reversed,
                           capacity) {}

SystemIdentification::SystemIdentification(std::unique_ptr<Transport> transport, uint8_t node,
                                           double offset, bool reversed, size_t capacity)
    : transport_(std::move(transport)),
      node_(node),
      offset_(offset),
      direction_(reversed ? -1 : 1),
      capacity_(capacity),
      request_sequence_(initial_sequence()) {
    validate_node(node);
    if (!transport_ || capacity == 0 || !std::isfinite(offset))
        throw std::invalid_argument(
            "sysid requires transport, positive queue capacity and finite offset");
}

void SystemIdentification::ensure_open() const {
    if (!transport_) throw canbus::CANSocketException("SystemIdentification is closed");
}

uint32_t SystemIdentification::next_request_sequence() { return ++request_sequence_; }

void SystemIdentification::reset_timeline() {
    ++epoch_;
    ++diagnostics_.timeline_resets;
    diagnostics_.session_discarded_samples += samples_.size();
    samples_.clear();
    previous_sequence_.reset();
    session_.reset();
    start_fence_tick_.reset();
}

void SystemIdentification::observe_information(const Information& information) {
    if (information.node != node_) return;
    bool reset = session_ && information.session_id != *session_;
    if (last_information_) {
        const auto& previous = *last_information_;
        reset |= uint32_t(information.control_tick - previous.control_tick) > 0x7FFFFFFF;
        reset |=
            uint32_t(information.request_drop_count - previous.request_drop_count) > 0x7FFFFFFF;
        reset |= uint32_t(information.tx_retry_count - previous.tx_retry_count) > 0x7FFFFFFF;
    }
    if (reset) reset_timeline();
    last_information_ = information;
}

void SystemIdentification::accept_sample(const Sample& sample) {
    if (sample.node != node_) {
        ++diagnostics_.unassociated_samples;
        return;
    }
    if (pending_start_) {
        if (pending_samples_.size() == capacity_)
            ++diagnostics_.host_sample_dropped;
        else
            pending_samples_.push_back(sample);
        return;
    }
    if (!session_) {
        ++diagnostics_.unassociated_samples;
        return;
    }
    if (sample.interval_ticks != period_) {
        reset_timeline();
        ++diagnostics_.unassociated_samples;
        return;
    }
    if (start_fence_tick_ && (uint32_t(sample.endpoint_tick - *start_fence_tick_) == 0 ||
                              uint32_t(sample.endpoint_tick - *start_fence_tick_) > 0x7FFFFFFF)) {
        ++diagnostics_.session_discarded_samples;
        return;
    }
    uint32_t gap = sample.sequence;  // First sequence can reveal initial missing samples.
    if (previous_sequence_) {
        const uint32_t sequence_delta = sample.sequence - *previous_sequence_;
        const uint32_t tick_delta = sample.endpoint_tick - previous_tick_;
        if (sequence_delta == 0) {
            ++diagnostics_.session_discarded_samples;
            return;
        }
        if (sequence_delta > 0x7FFFFFFF || tick_delta == 0 || tick_delta > 0x7FFFFFFF) {
            reset_timeline();
            ++diagnostics_.unassociated_samples;
            return;
        }
        gap = sequence_delta - 1;
        unwrapped_sequence_ += sequence_delta;
        unwrapped_tick_ += tick_delta;
    } else {
        unwrapped_sequence_ = sample.sequence;
        unwrapped_tick_ = sample.endpoint_tick;
    }
    previous_sequence_ = sample.sequence;
    previous_tick_ = sample.endpoint_tick;
    // The start fence only distinguishes initial kernel/firmware leftovers; long sessions wrap.
    start_fence_tick_.reset();
    diagnostics_.sequence_gaps += gap;
    Measurement measurement;
    measurement.sample = sample;
    measurement.session_id = *session_;
    measurement.timeline_epoch = epoch_;
    measurement.unwrapped_sequence = unwrapped_sequence_;
    measurement.unwrapped_endpoint_tick = unwrapped_tick_;
    measurement.sequence_gap = gap;
    measurement.position = direction_ * sample.position + offset_;
    measurement.velocity = direction_ * sample.velocity;
    measurement.averaged_iq = direction_ * sample.averaged_iq;
    measurement.torque_estimate = direction_ * sample.torque_estimate;
    measurement.mit_feedforward_torque = direction_ * sample.mit_feedforward_torque;
    measurement.instantaneous_iq = direction_ * sample.instantaneous_iq;
    if (samples_.size() == capacity_)
        ++diagnostics_.host_sample_dropped;
    else
        samples_.push_back(std::move(measurement));  // Drop new; never silently overwrite.
}

bool SystemIdentification::receive_one(int timeout_us) {
    ensure_open();
    try {
        const auto frame = transport_->receive(timeout_us);
        diagnostics_.socket_dropped = transport_->socket_dropped();
        if (!frame) return false;
        if (frame->can_id == sample_id) {
            accept_sample(decode_sample(*frame));
        } else if (frame->can_id == response_id) {
            Reply reply;
            if (frame->payload.size() == 8)
                reply.ack = decode_ack(*frame);
            else
                reply.information = decode_information(*frame);
            if (replies_.size() == capacity_)
                ++diagnostics_.host_reply_dropped;
            else
                replies_.push_back(std::move(reply));
        } else {
            if (other_.size() == capacity_)
                ++diagnostics_.host_other_dropped;
            else
                other_.push_back(*frame);
        }
        return true;
    } catch (const ProtocolError&) {
        ++diagnostics_.invalid_frames;
        throw;  // Invalid protocol data is never silently turned into a timeout.
    }
}

Reply SystemIdentification::request(Operation operation, uint8_t argument, uint32_t sequence,
                                    int timeout_us) {
    validate_timeout(timeout_us);
    ensure_open();
    transport_->send(encode_request(node_, operation, argument, sequence));
    const auto deadline = Clock::now() + std::chrono::microseconds(timeout_us);
    while (true) {
        const auto match = std::find_if(replies_.begin(), replies_.end(), [&](const Reply& reply) {
            if (reply.ack)
                return reply.ack->operation == operation && reply.ack->request_sequence == sequence;
            const auto& information = *reply.information;
            return information.node == node_ && information.request_sequence == sequence &&
                   information.message_type == (operation == Operation::INFO ? 0x81 : 0x82) &&
                   (operation == Operation::INFO || operation == Operation::STATUS);
        });
        if (match != replies_.end()) {
            Reply result = *match;
            replies_.erase(match);
            if (result.information) observe_information(*result.information);
            return result;
        }
        const int remaining = remaining_us(deadline);
        if (remaining == 0 || !receive_one(remaining))
            throw TimeoutError("sysid management reply timeout");
    }
}

Information SystemIdentification::require_information(const Reply& reply) const {
    if (!reply.information)
        throw ProtocolError("sysid INFO/STATUS rejected with result " +
                            std::to_string(static_cast<unsigned>(reply.ack->result)));
    return *reply.information;
}

Reply SystemIdentification::info(int timeout_us) {
    std::lock_guard<std::mutex> guard(mutex_);
    return request(Operation::INFO, 0, next_request_sequence(), timeout_us);
}

Reply SystemIdentification::status(int timeout_us) {
    std::lock_guard<std::mutex> guard(mutex_);
    return request(Operation::STATUS, 0, next_request_sequence(), timeout_us);
}

StartResult SystemIdentification::start(int rate_hz, int timeout_us) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (rate_hz != 500 && rate_hz != 1000)
        throw std::invalid_argument("sysid rate_hz must be 500 or 1000");
    const auto baseline =
        require_information(request(Operation::STATUS, 0, next_request_sequence(), timeout_us));
    if (baseline.active)
        throw SessionError("capture already active; do not START or STOP another session");
    // Consume existing kernel data with a strict bound while inactive, never flush while streaming.
    poll_locked(0, capacity_);
    reset_timeline();
    pending_samples_.clear();
    start_fence_tick_ = baseline.control_tick;
    period_ = rate_hz == 500 ? 40 : 20;
    const uint32_t sequence = next_request_sequence();
    pending_start_ = true;
    StartResult result;
    result.session_id = sequence;
    try {
        try {
            result.reply = request(Operation::START, rate_hz == 500 ? 0 : 1, sequence, timeout_us);
            result.accepted = result.reply.accepted();
        } catch (const TimeoutError&) {
            // Never retry START: sequence is a session tag, not a firmware deduplication key.
            const auto recovered = require_information(
                request(Operation::STATUS, 0, next_request_sequence(), timeout_us));
            result.recovery_status = recovered;
            result.recovered = true;
            if (recovered.session_id != sequence)
                throw SessionError(
                    "START outcome unconfirmed or another session active; no retry/STOP sent");
            // It may already have timed out/stopped; preserve active and stop_reason in
            // recovery_status.
            result.accepted = true;
            start_fence_tick_ = baseline.control_tick;
        }
        pending_start_ = false;
        if (result.accepted) {
            session_ = sequence;
            auto pending = std::move(pending_samples_);
            pending_samples_.clear();
            for (const auto& sample : pending) accept_sample(sample);
        } else {
            diagnostics_.session_discarded_samples += pending_samples_.size();
            pending_samples_.clear();
            start_fence_tick_.reset();
        }
        return result;
    } catch (...) {
        pending_start_ = false;
        diagnostics_.session_discarded_samples += pending_samples_.size();
        pending_samples_.clear();
        reset_timeline();
        throw;
    }
}

Reply SystemIdentification::heartbeat(int timeout_us) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!session_) throw SessionError("no owned capture session; START required");
    return request(Operation::HEARTBEAT, 0, next_request_sequence(), timeout_us);
}

StopResult SystemIdentification::stop(int timeout_us, int drain_timeout_us,
                                      size_t max_drain_frames) {
    std::lock_guard<std::mutex> guard(mutex_);
    validate_timeout(drain_timeout_us);
    if (!session_) throw SessionError("no owned capture session; no STOP sent");
    const auto owned = *session_;
    const auto before =
        require_information(request(Operation::STATUS, 0, next_request_sequence(), timeout_us));
    if (!session_ || before.session_id != owned)
        throw SessionError("capture ownership changed; no STOP sent");
    StopResult result;
    result.reply = request(Operation::STOP, 0, next_request_sequence(), timeout_us);
    if (!result.reply.accepted()) {
        result.final_status =
            require_information(request(Operation::STATUS, 0, next_request_sequence(), timeout_us));
        return result;
    }
    const auto deadline = Clock::now() + std::chrono::microseconds(drain_timeout_us);
    size_t frames = 0;
    bool quiet = false;
    while (frames < max_drain_frames && remaining_us(deadline) > 0) {
        if (receive_one(std::min(remaining_us(deadline), 2000))) {
            ++frames;
        } else {
            quiet = true;
            break;
        }
    }
    result.drain_limit_reached =
        frames == max_drain_frames || (!quiet && remaining_us(deadline) == 0);
    result.final_status =
        require_information(request(Operation::STATUS, 0, next_request_sequence(), timeout_us));
    if (!session_ || result.final_status.session_id != owned || result.final_status.active)
        throw SessionError(
            "session changed during STOP/drain; final state is not owned inactive session");
    result.samples = take_samples();
    // Keep attribution for delayed queued samples; next START explicitly replaces the timeline.
    return result;
}

size_t SystemIdentification::poll_locked(int timeout_us, size_t max_frames) {
    validate_timeout(timeout_us);
    ensure_open();
    size_t count = 0;
    while (count < max_frames && receive_one(count == 0 ? timeout_us : 0)) ++count;
    return count;
}

size_t SystemIdentification::poll(int timeout_us, size_t max_frames) {
    std::lock_guard<std::mutex> guard(mutex_);
    return poll_locked(timeout_us, max_frames);
}

std::vector<Measurement> SystemIdentification::take_samples() {
    std::vector<Measurement> result(samples_.begin(), samples_.end());
    samples_.clear();
    return result;
}

std::vector<Measurement> SystemIdentification::read_samples(int timeout_us, size_t max_frames) {
    std::lock_guard<std::mutex> guard(mutex_);
    poll_locked(timeout_us, max_frames);
    return take_samples();
}

std::vector<Reply> SystemIdentification::take_replies() {
    std::lock_guard<std::mutex> guard(mutex_);
    std::vector<Reply> result(replies_.begin(), replies_.end());
    replies_.clear();
    for (const auto& reply : result)
        if (reply.information) observe_information(*reply.information);
    return result;
}

std::vector<Frame> SystemIdentification::take_other_frames() {
    std::lock_guard<std::mutex> guard(mutex_);
    std::vector<Frame> result(other_.begin(), other_.end());
    other_.clear();
    return result;
}

Diagnostics SystemIdentification::diagnostics() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return diagnostics_;
}

std::optional<uint32_t> SystemIdentification::owned_session() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return session_;
}

void SystemIdentification::close() {
    std::lock_guard<std::mutex> guard(mutex_);
    transport_.reset();
    reset_timeline();
    pending_samples_.clear();
    replies_.clear();
    other_.clear();
}

}  // namespace damiao_can::sysid
