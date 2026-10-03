#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <damiao_can/sysid/system_identification.hpp>

namespace nb = nanobind;
using namespace damiao_can::sysid;

void bind_system_identification(nb::module_& m) {
    nb::exception<ProtocolError>(m, "SysIdProtocolError", PyExc_RuntimeError);
    nb::exception<TimeoutError>(m, "SysIdTimeoutError", PyExc_RuntimeError);
    nb::exception<SessionError>(m, "SysIdSessionError", PyExc_RuntimeError);
    nb::enum_<Operation>(m, "SysIdOperation")
        .value("START", Operation::START)
        .value("STOP", Operation::STOP)
        .value("HEARTBEAT", Operation::HEARTBEAT)
        .value("INFO", Operation::INFO)
        .value("STATUS", Operation::STATUS);
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
        .def_ro("instantaneous_iq", &Measurement::instantaneous_iq);
    nb::class_<Reply>(m, "SysIdReply")
        .def_ro("ack", &Reply::ack)
        .def_ro("information", &Reply::information)
        .def_prop_ro("accepted", &Reply::accepted);
    nb::class_<StartResult>(m, "SysIdStartResult")
        .def_ro("reply", &StartResult::reply)
        .def_ro("recovery_status", &StartResult::recovery_status)
        .def_ro("session_id", &StartResult::session_id)
        .def_ro("recovered", &StartResult::recovered)
        .def_ro("accepted", &StartResult::accepted);
    nb::class_<Diagnostics>(m, "SysIdDiagnostics")
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
    m.def("decode_sysid_sample", &decode_sample, nb::arg("frame"));
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
        .def(nb::init<const std::string&, uint8_t, double, bool, size_t>(), nb::arg("interface"),
             nb::arg("node"), nb::arg("offset") = 0, nb::arg("reversed") = false,
             nb::arg("queue_capacity") = 2048)
        .def("info", &SystemIdentification::info, nb::arg("timeout_us") = 100000,
             nb::call_guard<nb::gil_scoped_release>())
        .def("status", &SystemIdentification::status, nb::arg("timeout_us") = 100000,
             nb::call_guard<nb::gil_scoped_release>())
        .def("start", &SystemIdentification::start, nb::arg("rate_hz") = 500,
             nb::arg("timeout_us") = 100000, nb::call_guard<nb::gil_scoped_release>())
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
        .def("close", &SystemIdentification::close, nb::call_guard<nb::gil_scoped_release>())
        .def(
            "__enter__", [](SystemIdentification& self) -> SystemIdentification& { return self; },
            nb::rv_policy::reference_internal)
        .def("__exit__",
             [](SystemIdentification& self, nb::handle, nb::handle, nb::handle) { self.close(); });
}
