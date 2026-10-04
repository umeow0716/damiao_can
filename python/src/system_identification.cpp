#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <damiao_can/sysid/system_identification.hpp>

namespace nb = nanobind;
using namespace damiao_can::sysid;

void bind_system_identification(nb::module_& m) {
    nb::class_<SafetyInformation>(m, "SysIdSafetyInformation")
        .def_ro("node", &SafetyInformation::node)
        .def_ro("request_sequence", &SafetyInformation::request_sequence)
        .def_ro("control_tick", &SafetyInformation::control_tick)
        .def_ro("command_lease_ticks", &SafetyInformation::command_lease_ticks)
        .def_ro("live_period_ticks", &SafetyInformation::live_period_ticks)
        .def_ro("last_command_tick", &SafetyInformation::last_command_tick)
        .def_ro("live_overwritten", &SafetyInformation::live_overwritten)
        .def_ro("deadman_enabled", &SafetyInformation::deadman_enabled)
        .def_ro("deadman_latched", &SafetyInformation::deadman_latched)
        .def_ro("live_active", &SafetyInformation::live_active)
        .def_ro("guard_enabled", &SafetyInformation::guard_enabled)
        .def_ro("guard_reason", &SafetyInformation::guard_reason)
        .def_ro("guard_lower", &SafetyInformation::guard_lower)
        .def_ro("guard_upper", &SafetyInformation::guard_upper)
        .def_ro("guard_max_velocity", &SafetyInformation::guard_max_velocity)
        .def_ro("guard_torque_limit", &SafetyInformation::guard_torque_limit)
        .def_ro("raw", &SafetyInformation::raw);
    nb::exception<ProtocolError>(m, "SysIdProtocolError", PyExc_RuntimeError);
    nb::exception<TimeoutError>(m, "SysIdTimeoutError", PyExc_RuntimeError);
    nb::exception<SessionError>(m, "SysIdSessionError", PyExc_RuntimeError);
    nb::enum_<Operation>(m, "SysIdOperation")
        .value("START", Operation::START)
        .value("STOP", Operation::STOP)
        .value("HEARTBEAT", Operation::HEARTBEAT)
        .value("INFO", Operation::INFO)
        .value("STATUS", Operation::STATUS)
        .value("SCAN_LOWER", Operation::SCAN_LOWER)
        .value("SCAN_UPPER", Operation::SCAN_UPPER)
        .value("SCAN_HOME", Operation::SCAN_HOME)
        .value("SCAN_SPEED", Operation::SCAN_SPEED)
        .value("SCAN_ACCEL", Operation::SCAN_ACCEL)
        .value("SCAN_TORQUE", Operation::SCAN_TORQUE)
        .value("SCAN_TEMPERATURE", Operation::SCAN_TEMPERATURE)
        .value("SCAN_REPEATS", Operation::SCAN_REPEATS)
        .value("SCAN_ERROR", Operation::SCAN_ERROR)
        .value("SCAN_SECONDS", Operation::SCAN_SECONDS)
        .value("SCAN_START", Operation::SCAN_START)
        .value("SCAN_STATUS", Operation::SCAN_STATUS)
        .value("DATA_ACK", Operation::DATA_ACK)
        .value("DATA_REPLAY", Operation::DATA_REPLAY)
        .value("SAFETY_STATUS", Operation::SAFETY_STATUS)
        .value("LIVE_START", Operation::LIVE_START)
        .value("LIVE_STOP", Operation::LIVE_STOP)
        .value("GUARD_LOWER", Operation::GUARD_LOWER)
        .value("GUARD_UPPER", Operation::GUARD_UPPER)
        .value("GUARD_SPEED", Operation::GUARD_SPEED)
        .value("GUARD_TORQUE", Operation::GUARD_TORQUE)
        .value("GUARD_ARM", Operation::GUARD_ARM)
        .value("SCAN_ABORT", Operation::SCAN_ABORT);
    nb::enum_<Result>(m, "SysIdResult")
        .value("ACCEPTED", Result::ACCEPTED)
        .value("BAD_VERSION", Result::BAD_VERSION)
        .value("BAD_ARGUMENT", Result::BAD_ARGUMENT)
        .value("BUSY", Result::BUSY)
        .value("TRANSPORT_NOT_READY", Result::TRANSPORT_NOT_READY)
        .value("UNSUPPORTED_OPERATION", Result::UNSUPPORTED_OPERATION)
        .value("MEASUREMENTS_NOT_READY", Result::MEASUREMENTS_NOT_READY);

    nb::class_<Frame>(m, "SysIdFrame")
        .def(nb::init<>())
        .def_rw("can_id", &Frame::can_id)
        .def_rw("fd", &Frame::fd)
        .def_rw("flags", &Frame::flags)
        .def_rw("host_receive_time_ns", &Frame::host_receive_time_ns)
        .def_prop_rw(
            "payload",
            [](const Frame& frame) {
                return nb::bytes(frame.payload.data(), frame.payload.size());
            },
            [](Frame& frame, nb::bytes data) {
                const auto* first = reinterpret_cast<const uint8_t*>(data.c_str());
                frame.payload.assign(first, first + data.size());
            });
    nb::class_<Ack>(m, "SysIdAck")
        .def_ro("version", &Ack::version)
        .def_ro("operation", &Ack::operation)
        .def_ro("result", &Ack::result)
        .def_ro("request_sequence", &Ack::request_sequence)
        .def_ro("raw", &Ack::raw);
    nb::class_<Information>(m, "SysIdInformation")
        .def_ro("version", &Information::version)
        .def_ro("message_type", &Information::message_type)
        .def_ro("node", &Information::node)
        .def_ro("active", &Information::active)
        .def_ro("request_sequence", &Information::request_sequence)
        .def_ro("session_id", &Information::session_id)
        .def_ro("control_tick", &Information::control_tick)
        .def_ro("sample_drop_count", &Information::sample_drop_count)
        .def_ro("request_drop_count", &Information::request_drop_count)
        .def_ro("tx_retry_count", &Information::tx_retry_count)
        .def_ro("next_sample_sequence", &Information::next_sample_sequence)
        .def_ro("output_torque_constant", &Information::output_torque_constant)
        .def_ro("gear_ratio", &Information::gear_ratio)
        .def_ro("factory_velocity_previous_weight", &Information::factory_velocity_previous_weight)
        .def_ro("iq_filter_beta", &Information::iq_filter_beta)
        .def_ro("current_limit", &Information::current_limit)
        .def_ro("accepted_command_sequence", &Information::accepted_command_sequence)
        .def_ro("applied_command_sequence", &Information::applied_command_sequence)
        .def_ro("sample_period_ticks", &Information::sample_period_ticks)
        .def_ro("stop_reason", &Information::stop_reason)
        .def_ro("mode", &Information::mode)
        .def_ro("fault", &Information::fault)
        .def_ro("raw", &Information::raw);
    nb::class_<Sample>(m, "SysIdSample")
        .def_ro("version", &Sample::version)
        .def_ro("node", &Sample::node)
        .def_ro("flags", &Sample::flags)
        .def_ro("sequence", &Sample::sequence)
        .def_ro("endpoint_tick", &Sample::endpoint_tick)
        .def_ro("applied_command_sequence", &Sample::applied_command_sequence)
        .def_ro("sample_drop_count", &Sample::sample_drop_count)
        .def_ro("position", &Sample::position)
        .def_ro("velocity", &Sample::velocity)
        .def_ro("averaged_iq", &Sample::averaged_iq)
        .def_ro("torque_estimate", &Sample::torque_estimate)
        .def_ro("mit_feedforward_torque", &Sample::mit_feedforward_torque)
        .def_ro("instantaneous_iq", &Sample::instantaneous_iq)
        .def_ro("applied_command_tick", &Sample::applied_command_tick)
        .def_ro("temperature", &Sample::temperature)
        .def_ro("velocity_tick", &Sample::velocity_tick)
        .def_ro("position_publication_tick", &Sample::position_publication_tick)
        .def_ro("interval_ticks", &Sample::interval_ticks)
        .def_ro("mode", &Sample::mode)
        .def_ro("fault", &Sample::fault)
        .def_ro("raw", &Sample::raw)
        .def_prop_ro("armed", &Sample::armed)
        .def_prop_ro("warmup", &Sample::warmup)
        .def_prop_ro("current_saturated", &Sample::current_saturated)
        .def_prop_ro("voltage_saturated", &Sample::voltage_saturated)
        .def_prop_ro("command_unknown", &Sample::command_unknown);
    nb::class_<Measurement>(m, "SysIdMeasurement")
        .def_ro("sample", &Measurement::sample)
        .def_ro("session_id", &Measurement::session_id)
        .def_ro("timeline_epoch", &Measurement::timeline_epoch)
        .def_ro("unwrapped_sequence", &Measurement::unwrapped_sequence)
        .def_ro("unwrapped_endpoint_tick", &Measurement::unwrapped_endpoint_tick)
        .def_ro("sequence_gap", &Measurement::sequence_gap)
        .def_ro("position", &Measurement::position)
        .def_ro("velocity", &Measurement::velocity)
        .def_ro("averaged_iq", &Measurement::averaged_iq)
        .def_ro("torque_estimate", &Measurement::torque_estimate)
        .def_ro("mit_feedforward_torque", &Measurement::mit_feedforward_torque)
        .def_ro("instantaneous_iq", &Measurement::instantaneous_iq)
        .def_ro("mos_temperature", &Measurement::mos_temperature);
    nb::class_<ScanConfig>(m, "SysIdScanConfig")
        .def(nb::init<>())
        .def_rw("lower", &ScanConfig::lower)
        .def_rw("upper", &ScanConfig::upper)
        .def_rw("home", &ScanConfig::home)
        .def_rw("speed", &ScanConfig::speed)
        .def_rw("acceleration", &ScanConfig::acceleration)
        .def_rw("torque_limit", &ScanConfig::torque_limit)
        .def_rw("temperature_limit", &ScanConfig::temperature_limit)
        .def_rw("repeats", &ScanConfig::repeats)
        .def_rw("tracking_error", &ScanConfig::tracking_error)
        .def_rw("max_seconds", &ScanConfig::max_seconds);
    nb::class_<ScanRecord>(m, "SysIdScanRecord")
        .def_ro("node", &ScanRecord::node)
        .def_ro("phase", &ScanRecord::phase)
        .def_ro("reason", &ScanRecord::reason)
        .def_ro("owned", &ScanRecord::owned)
        .def_ro("active", &ScanRecord::active)
        .def_ro("sequence", &ScanRecord::sequence)
        .def_ro("endpoint_tick", &ScanRecord::endpoint_tick)
        .def_ro("capture_session", &ScanRecord::capture_session)
        .def_ro("scan_session", &ScanRecord::scan_session)
        .def_ro("completed_legs", &ScanRecord::completed_legs)
        .def_ro("planned_position", &ScanRecord::planned_position)
        .def_ro("command_velocity", &ScanRecord::command_velocity)
        .def_ro("raw_torque", &ScanRecord::raw_torque)
        .def_ro("position", &ScanRecord::position)
        .def_ro("home", &ScanRecord::home)
        .def_ro("target", &ScanRecord::target)
        .def_ro("start_tick", &ScanRecord::start_tick)
        .def_ro("end_tick", &ScanRecord::end_tick)
        .def_ro("raw", &ScanRecord::raw);
    nb::class_<Reply>(m, "SysIdReply")
        .def_ro("safety", &Reply::safety)
        .def_ro("ack", &Reply::ack)
        .def_ro("information", &Reply::information)
        .def_ro("scan", &Reply::scan)
        .def_prop_ro("accepted", &Reply::accepted);
    nb::class_<StartResult>(m, "SysIdStartResult")
        .def_ro("reply", &StartResult::reply)
        .def_ro("recovery_status", &StartResult::recovery_status)
        .def_ro("session_id", &StartResult::session_id)
        .def_ro("recovered", &StartResult::recovered)
        .def_ro("accepted", &StartResult::accepted);
    nb::class_<Diagnostics>(m, "SysIdDiagnostics")
        .def_ro("host_scan_dropped", &Diagnostics::host_scan_dropped)
        .def_ro("host_sample_dropped", &Diagnostics::host_sample_dropped)
        .def_ro("host_reply_dropped", &Diagnostics::host_reply_dropped)
        .def_ro("host_other_dropped", &Diagnostics::host_other_dropped)
        .def_ro("socket_dropped", &Diagnostics::socket_dropped)
        .def_ro("invalid_frames", &Diagnostics::invalid_frames)
        .def_ro("unassociated_samples", &Diagnostics::unassociated_samples)
        .def_ro("session_discarded_samples", &Diagnostics::session_discarded_samples)
        .def_ro("sequence_gaps", &Diagnostics::sequence_gaps)
        .def_ro("timeline_resets", &Diagnostics::timeline_resets);
    nb::class_<StopResult>(m, "SysIdStopResult")
        .def_ro("reply", &StopResult::reply)
        .def_ro("samples", &StopResult::samples)
        .def_ro("final_status", &StopResult::final_status)
        .def_ro("drain_limit_reached", &StopResult::drain_limit_reached);

    m.def("decode_sysid_ack", &decode_ack, nb::arg("frame"));
    m.def("decode_sysid_information", &decode_information, nb::arg("frame"));
    m.def("decode_sysid_scan_record", &decode_scan_record);
    m.def("decode_sysid_sample", &decode_sample, nb::arg("frame"));
    m.def("decode_sysid_live", &decode_live, nb::arg("frame"), nb::arg("offset") = 0.,
          nb::arg("reversed") = false);
    m.def("decode_sysid_safety", &decode_safety, nb::arg("frame"));
    m.def(
        "encode_sysid_request",
        [](uint8_t node, Operation operation, uint8_t argument, uint32_t sequence) {
            const auto payload = encode_request(node, operation, argument, sequence);
            return nb::bytes(payload.data(), payload.size());
        },
        nb::arg("node"), nb::arg("operation"), nb::arg("argument"), nb::arg("request_sequence"));
    m.def("sysid_command_sequence_distance", &command_sequence_distance, nb::arg("previous"),
          nb::arg("current"));
    nb::class_<SystemIdentification>(m, "SystemIdentification")
        .def(nb::init<const std::string&, uint8_t, double, bool, size_t, bool>(),
             nb::arg("interface"), nb::arg("node"), nb::arg("offset") = 0,
             nb::arg("reversed") = false, nb::arg("queue_capacity") = 2048,
             nb::arg("live_only") = false)
        .def("safety_status", &SystemIdentification::safety_status, nb::arg("timeout_us") = 100000,
             nb::call_guard<nb::gil_scoped_release>())
        .def("start_live", &SystemIdentification::start_live, nb::arg("timeout_us") = 100000,
             nb::call_guard<nb::gil_scoped_release>())
        .def("stop_live", &SystemIdentification::stop_live, nb::arg("timeout_us") = 100000,
             nb::call_guard<nb::gil_scoped_release>())
        .def("configure_guard", &SystemIdentification::configure_guard, nb::arg("lower"),
             nb::arg("upper"), nb::arg("max_velocity"), nb::arg("torque_limit"),
             nb::arg("timeout_us") = 100000, nb::call_guard<nb::gil_scoped_release>())
        .def("read_latest", &SystemIdentification::read_latest, nb::arg("timeout_us") = 0,
             nb::arg("max_frames") = 256, nb::call_guard<nb::gil_scoped_release>())
        .def("info", &SystemIdentification::info, nb::arg("timeout_us") = 100000,
             nb::call_guard<nb::gil_scoped_release>())
        .def("status", &SystemIdentification::status, nb::arg("timeout_us") = 100000,
             nb::call_guard<nb::gil_scoped_release>())
        .def("start", &SystemIdentification::start, nb::arg("rate_hz") = 500,
             nb::arg("timeout_us") = 100000, nb::arg("reliable") = false,
             nb::call_guard<nb::gil_scoped_release>())
        .def("acknowledge_data", &SystemIdentification::acknowledge_data, nb::arg("next_sequence"),
             nb::arg("timeout_us") = 20000, nb::call_guard<nb::gil_scoped_release>())
        .def("replay_data", &SystemIdentification::replay_data, nb::arg("next_sequence"),
             nb::arg("timeout_us") = 20000, nb::call_guard<nb::gil_scoped_release>())
        .def("heartbeat", &SystemIdentification::heartbeat, nb::arg("timeout_us") = 100000,
             nb::call_guard<nb::gil_scoped_release>())
        .def("stop", &SystemIdentification::stop, nb::arg("timeout_us") = 100000,
             nb::arg("drain_timeout_us") = 50000, nb::arg("max_drain_frames") = 4096,
             nb::call_guard<nb::gil_scoped_release>())
        .def("poll", &SystemIdentification::poll, nb::arg("timeout_us") = 0,
             nb::arg("max_frames") = 256, nb::call_guard<nb::gil_scoped_release>())
        .def("read_samples", &SystemIdentification::read_samples, nb::arg("timeout_us") = 0,
             nb::arg("max_frames") = 256, nb::call_guard<nb::gil_scoped_release>())
        .def("take_replies", &SystemIdentification::take_replies,
             nb::call_guard<nb::gil_scoped_release>())
        .def("take_other_frames", &SystemIdentification::take_other_frames,
             nb::call_guard<nb::gil_scoped_release>())
        .def("diagnostics", &SystemIdentification::diagnostics,
             nb::call_guard<nb::gil_scoped_release>())
        .def_prop_ro("owned_session", &SystemIdentification::owned_session,
                     nb::call_guard<nb::gil_scoped_release>())
        .def("configure_scan", &SystemIdentification::configure_scan, nb::arg("config"),
             nb::arg("timeout_us") = 100000, nb::call_guard<nb::gil_scoped_release>())
        .def("start_scan", &SystemIdentification::start_scan, nb::arg("timeout_us") = 100000,
             nb::call_guard<nb::gil_scoped_release>())
        .def("scan_status", &SystemIdentification::scan_status, nb::arg("timeout_us") = 100000,
             nb::call_guard<nb::gil_scoped_release>())
        .def("abort_scan", &SystemIdentification::abort_scan, nb::arg("timeout_us") = 100000,
             nb::call_guard<nb::gil_scoped_release>())
        .def("take_scan_records", &SystemIdentification::take_scan_records,
             nb::call_guard<nb::gil_scoped_release>())
        .def("close", &SystemIdentification::close, nb::call_guard<nb::gil_scoped_release>())
        .def(
            "__enter__", [](SystemIdentification& self) -> SystemIdentification& { return self; },
            nb::rv_policy::reference_internal)
        .def("__exit__",
             [](SystemIdentification& self, nb::handle, nb::handle, nb::handle) { self.close(); });
}
