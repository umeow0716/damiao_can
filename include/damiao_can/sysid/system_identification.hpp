#pragma once

#include <array>
#include <cstdint>
#include <damiao_can/canbus/can_socket.hpp>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace damiao_can::sysid {

constexpr uint32_t request_id = 0x6F0;
constexpr uint32_t response_id = 0x6F1;
constexpr uint32_t sample_id = 0x6F2;
constexpr double tick_seconds = 0.00005;  // Nominal firmware control count, not wall time.

enum class Operation : uint8_t { START = 1, STOP = 2, HEARTBEAT = 3, INFO = 4, STATUS = 5 };
enum class Result : uint8_t {
    ACCEPTED = 0,
    BAD_VERSION = 1,
    BAD_ARGUMENT = 2,
    BUSY = 3,
    TRANSPORT_NOT_READY = 4,
    UNSUPPORTED_OPERATION = 5,
    MEASUREMENTS_NOT_READY = 6
};

class ProtocolError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};
class TimeoutError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};
class SessionError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Host receive timestamp: std::chrono::steady_clock nanoseconds, immediately after read.
// It has an unspecified epoch, and is not interchangeable with Python monotonic_ns().
struct Frame {
    uint32_t can_id = 0;  // Includes SocketCAN EFF/RTR/ERR flags.
    bool fd = false;
    uint8_t flags = 0;  // SocketCAN CANFD_* flags, independent of sample flags.
    std::vector<uint8_t> payload;
    uint64_t host_receive_time_ns = 0;
};

struct Ack {
    uint8_t version = 1;
    Operation operation = Operation::START;
    Result result = Result::ACCEPTED;
    uint32_t request_sequence = 0;
    Frame raw;
};

struct Information {
    uint8_t version = 1;
    uint8_t message_type = 0;
    uint8_t node = 0;
    bool active = false;
    uint32_t request_sequence = 0;
    uint32_t session_id = 0;
    uint32_t control_tick = 0;
    uint32_t sample_drop_count = 0;
    uint32_t request_drop_count = 0;
    uint32_t tx_retry_count = 0;
    uint32_t next_sample_sequence = 0;
    float output_torque_constant = 0;
    float gear_ratio = 0;
    float factory_velocity_previous_weight = 0;
    float iq_filter_beta = 0;
    float current_limit = 0;
    uint32_t accepted_command_sequence = 0;
    uint32_t applied_command_sequence = 0;
    uint8_t sample_period_ticks = 0;
    uint8_t stop_reason = 0;
    uint8_t mode = 0;
    uint8_t fault = 0;
    Frame raw;
};

struct Sample {
    uint8_t version = 1;
    uint8_t node = 0;
    uint8_t flags = 0;
    uint32_t sequence = 0;
    uint32_t endpoint_tick = 0;
    uint32_t applied_command_sequence = 0;
    uint32_t sample_drop_count = 0;
    float position = 0;
    float velocity = 0;
    float averaged_iq = 0;
    float torque_estimate = 0;
    float mit_feedforward_torque = 0;
    float instantaneous_iq = 0;
    uint32_t applied_command_tick = 0;
    float temperature = 0;
    uint32_t velocity_tick = 0;
    uint32_t position_publication_tick = 0;
    uint16_t interval_ticks = 0;
    uint8_t mode = 0;
    uint8_t fault = 0;
    Frame raw;
    bool armed() const { return flags & 1; }
    bool current_saturated() const { return flags & 2; }
    bool voltage_saturated() const { return flags & 4; }
    bool warmup() const { return flags & 8; }
    bool command_unknown() const { return flags & 16; }
};

// Wire values remain in sample; adjusted values apply the host transform exactly once.
struct Measurement {
    Sample sample;
    uint32_t session_id = 0;
    uint64_t timeline_epoch = 0;
    uint64_t unwrapped_sequence = 0;
    uint64_t unwrapped_endpoint_tick = 0;
    uint32_t sequence_gap = 0;  // Received gap, not firmware sample_drop_count.
    double position = 0;
    double velocity = 0;
    double averaged_iq = 0;
    double torque_estimate = 0;
    double mit_feedforward_torque = 0;
    double instantaneous_iq = 0;
};

struct Reply {
    std::optional<Ack> ack;
    std::optional<Information> information;
    bool accepted() const { return information || (ack && ack->result == Result::ACCEPTED); }
};

struct StartResult {
    Reply reply;
    std::optional<Information> recovery_status;
    uint32_t session_id = 0;
    bool recovered = false;
    bool accepted = false;
};

struct Diagnostics {
    uint64_t host_sample_dropped = 0;
    uint64_t host_reply_dropped = 0;
    uint64_t host_other_dropped = 0;
    uint64_t socket_dropped = 0;  // Linux SO_RXQ_OVFL, counted independently.
    uint64_t invalid_frames = 0;
    uint64_t unassociated_samples = 0;
    uint64_t session_discarded_samples = 0;
    uint64_t sequence_gaps = 0;
    uint64_t timeline_resets = 0;
};

struct StopResult {
    Reply reply;
    std::vector<Measurement> samples;
    Information final_status;
    bool drain_limit_reached = false;
};

Ack decode_ack(const Frame& frame);
Information decode_information(const Frame& frame);
Sample decode_sample(const Frame& frame);
std::array<uint8_t, 8> encode_request(uint8_t node, Operation operation, uint8_t argument,
                                      uint32_t request_sequence);
// Command sequence skips UINT32_MAX. Unknown counters cannot be subtracted.
uint32_t command_sequence_distance(uint32_t previous, uint32_t current);

// Injectable boundary for deterministic offline tests. Production owns a separate CANSocket.
class Transport {
public:
    virtual ~Transport() = default;
    virtual void send(const std::array<uint8_t, 8>& payload) = 0;
    virtual std::optional<Frame> receive(int timeout_us) = 0;
    virtual uint64_t socket_dropped() const { return 0; }
};

class SystemIdentification {
public:
    explicit SystemIdentification(const std::string& interface, uint8_t node, double offset = 0,
                                  bool reversed = false, size_t queue_capacity = 2048);
    SystemIdentification(std::unique_ptr<Transport> transport, uint8_t node, double offset = 0,
                         bool reversed = false, size_t queue_capacity = 2048);
    ~SystemIdentification() = default;  // Closes socket; never STOP/disable/control in destructor.
    SystemIdentification(const SystemIdentification&) = delete;
    SystemIdentification& operator=(const SystemIdentification&) = delete;

    Reply info(int timeout_us = 100000);
    Reply status(int timeout_us = 100000);
    StartResult start(int rate_hz = 500, int timeout_us = 100000);
    Reply heartbeat(int timeout_us = 100000);
    StopResult stop(int timeout_us = 100000, int drain_timeout_us = 50000,
                    size_t max_drain_frames = 4096);
    size_t poll(int timeout_us = 0, size_t max_frames = 256);
    std::vector<Measurement> read_samples(int timeout_us = 0, size_t max_frames = 256);
    std::vector<Reply> take_replies();
    std::vector<Frame> take_other_frames();
    Diagnostics diagnostics() const;
    std::optional<uint32_t> owned_session() const;
    void close();  // No implicit management or motor operations.

private:
    Reply request(Operation operation, uint8_t argument, uint32_t sequence, int timeout_us);
    Information require_information(const Reply& reply) const;
    uint32_t next_request_sequence();
    bool receive_one(int timeout_us);
    size_t poll_locked(int timeout_us, size_t max_frames);
    void observe_information(const Information& information);
    void accept_sample(const Sample& sample);
    void reset_timeline();
    std::vector<Measurement> take_samples();
    void ensure_open() const;

    mutable std::mutex mutex_;
    std::unique_ptr<Transport> transport_;
    uint8_t node_;
    double offset_;
    double direction_;
    size_t capacity_;
    uint32_t request_sequence_;
    std::optional<uint32_t> session_;
    std::optional<Information> last_information_;
    std::optional<uint32_t> start_fence_tick_;
    uint16_t period_ = 0;
    bool pending_start_ = false;
    std::deque<Sample> pending_samples_;
    std::deque<Measurement> samples_;
    std::deque<Reply> replies_;
    std::deque<Frame> other_;
    Diagnostics diagnostics_;
    uint64_t epoch_ = 0;
    std::optional<uint32_t> previous_sequence_;
    uint32_t previous_tick_ = 0;
    uint64_t unwrapped_sequence_ = 0;
    uint64_t unwrapped_tick_ = 0;
};

}  // namespace damiao_can::sysid
