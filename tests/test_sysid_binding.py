"""Offline wire fixtures only: this module never opens CAN or controls hardware."""
import struct
import unittest

import damiao_can as dc


FIXTURE = bytes.fromhex(
    "01 90 01 09 00 00 00 00 28 00 00 00 01 00 00 00 "
    "00 00 00 00 00 00 A0 3F 00 00 00 C0 00 00 40 40 "
    "00 00 C0 3F 00 00 80 BF 00 00 40 40 01 00 00 00 "
    "00 00 F0 41 28 00 00 00 27 00 00 00 28 00 01 00"
)


def frame(can_id, payload, fd=False, flags=0):
    result = dc.SysIdFrame()
    result.can_id = can_id
    result.payload = payload
    result.fd = fd
    result.flags = flags
    result.host_receive_time_ns = 123456789
    return result


class BindingTests(unittest.TestCase):
    def test_applied_velocity_v2(self):
        payload = bytearray(FIXTURE)
        payload[0] = 2
        payload[3] = 1
        payload[62] = 3
        struct.pack_into("<f", payload, 36, -.08)
        sample = dc.decode_sysid_sample(frame(0x6F2, bytes(payload), True, 1))
        self.assertTrue(sample.velocity_setpoint_valid)
        self.assertAlmostEqual(sample.applied_velocity_setpoint, -.08)
        self.assertEqual(sample.mit_feedforward_torque, 0)
        payload[3] |= 32
        self.assertTrue(dc.decode_sysid_sample(
            frame(0x6F2, bytes(payload), True, 1)).velocity_setpoint_changed)
        payload[3] = 1
        payload[1] = 0x92
        m = dc.decode_sysid_live(
            frame(0x6E0, bytes(payload), True, 1), offset=.5, reversed=True)
        self.assertAlmostEqual(m.applied_velocity_setpoint, .08)
        self.assertEqual(m.mit_feedforward_torque, 0)
        payload[0] = 1
        payload[1] = 0x90
        self.assertFalse(dc.decode_sysid_sample(
            frame(0x6F2, bytes(payload), True, 1)).velocity_setpoint_valid)

    def test_live_and_guard_exports(self):
        live = bytearray(FIXTURE)
        live[1] = 0x92
        live[3] = 1
        struct.pack_into("<I", live, 4, 123)
        struct.pack_into("<f", live, 28, 42)
        m = dc.decode_sysid_live(
            frame(0x6E0, bytes(live), True, 1), offset=.2, reversed=True)
        self.assertEqual(m.session_id, 123)
        self.assertAlmostEqual(m.position, -1.05)
        self.assertEqual(m.mos_temperature, 42)
        self.assertEqual(m.averaged_iq, -3)
        safety = bytearray(64)
        safety[:4] = bytes([1, 0x84, 1, 9])
        struct.pack_into("<II", safety, 12, 5000, 40)
        struct.pack_into("<4f", safety, 28, -.2, .4, .3, 2.)
        status = dc.decode_sysid_safety(frame(0x6F1, bytes(safety), True, 1))
        self.assertTrue(status.guard_enabled)
        self.assertEqual(status.guard_torque_limit, 2.)
        safety[3] = 11
        safety[44] = 8
        status = dc.decode_sysid_safety(frame(0x6F1, bytes(safety), True, 1))
        self.assertTrue(status.deadman_latched)
        self.assertEqual(status.guard_reason, 8)
        self.assertTrue(hasattr(dc.SystemIdentification, 'configure_guard'))
        self.assertTrue(hasattr(dc.SystemIdentification, 'read_latest'))
        self.assertIn('SysIdSafetyInformation', dc.__all__)

    def test_complete_firmware_fixture(self):
        sample = dc.decode_sysid_sample(frame(0x6F2, FIXTURE, True, 1))
        expected = dict(version=1, node=1, flags=9, sequence=0, endpoint_tick=40,
                        applied_command_sequence=1, sample_drop_count=0,
                        position=1.25, velocity=-2.0, averaged_iq=3.0,
                        torque_estimate=1.5, mit_feedforward_torque=-1.0,
                        instantaneous_iq=3.0, applied_command_tick=1,
                        temperature=30.0, velocity_tick=40,
                        position_publication_tick=39, interval_ticks=40,
                        mode=1, fault=0)
        for name, value in expected.items():
            with self.subTest(field=name):
                self.assertEqual(getattr(sample, name), value)
        self.assertTrue(sample.armed)
        self.assertTrue(sample.warmup)
        self.assertFalse(sample.current_saturated)
        self.assertFalse(sample.voltage_saturated)
        self.assertFalse(sample.command_unknown)
        self.assertEqual(sample.raw.payload, FIXTURE)
        self.assertEqual(sample.raw.host_receive_time_ns, 123456789)
        with self.assertRaises(AttributeError):
            sample.position = 0.0

    def test_request_ack_and_counter(self):
        payload = dc.encode_sysid_request(node=1, operation=dc.SysIdOperation.START,
                                          argument=0, request_sequence=0x12345678)
        self.assertEqual(payload, bytes.fromhex("01 01 01 00 78 56 34 12"))
        ack = dc.decode_sysid_ack(
            frame(0x6F1, bytes.fromhex("01 80 01 03 78 56 34 12")))
        self.assertEqual(ack.operation, dc.SysIdOperation.START)
        self.assertEqual(ack.result, dc.SysIdResult.BUSY)
        self.assertEqual(ack.request_sequence, 0x12345678)
        self.assertEqual(dc.sysid_command_sequence_distance(0xFFFFFFFE, 0), 1)
        with self.assertRaises(ValueError):
            dc.sysid_command_sequence_distance(0xFFFFFFFF, 0)

    def test_information_status_and_flags(self):
        payload = bytearray(64)
        payload[:4] = bytes([1, 0x81, 1, 1])
        struct.pack_into("<7I5f2I4B", payload, 4,
                         9, 8, 123, 2, 3, 4, 5,
                         0.5, 10.0, 0.8, 0.015465039, 20.0,
                         6, 7, 40, 2, 1, 8)
        for message_type in (0x81, 0x82):
            payload[1] = message_type
            info = dc.decode_sysid_information(
                frame(0x6F1, bytes(payload), True, 1))
            self.assertEqual(info.message_type, message_type)
            self.assertEqual(info.node, 1)
            self.assertTrue(info.active)
            for name, value in dict(request_sequence=9, session_id=8, control_tick=123,
                                    sample_drop_count=2, request_drop_count=3,
                                    tx_retry_count=4, next_sample_sequence=5,
                                    output_torque_constant=0.5, gear_ratio=10,
                                    current_limit=20, accepted_command_sequence=6,
                                    applied_command_sequence=7, sample_period_ticks=40,
                                    stop_reason=2, mode=1, fault=8).items():
                self.assertEqual(getattr(info, name), value)
            self.assertAlmostEqual(info.factory_velocity_previous_weight, 0.8)
            self.assertAlmostEqual(info.iq_filter_beta, 0.015465039)
            self.assertEqual(info.raw.payload, bytes(payload))

    def test_validation_and_exports(self):
        for name in dc.__all__:
            self.assertTrue(hasattr(dc, name), name)
        self.assertTrue(issubclass(dc.SysIdProtocolError, RuntimeError))
        for can_id, payload, fd, flags in (
            (0x6F1, FIXTURE, True, 1),
            (0x6F2 | 0x80000000, FIXTURE, True, 1),
            (0x6F2 | 0x40000000, FIXTURE, True, 1),
            (0x6F2, FIXTURE, False, 0),
            (0x6F2, FIXTURE, True, 0),
            (0x6F2, FIXTURE, True, 3),
            (0x6F2, FIXTURE[:-1], True, 1),
            (0x6F2, b"\x02" + FIXTURE[1:], True, 1),
            (0x6F2, b"\x01\x81" + FIXTURE[2:], True, 1),
        ):
            with self.subTest(can_id=can_id, size=len(payload), fd=fd, flags=flags):
                with self.assertRaises(dc.SysIdProtocolError):
                    dc.decode_sysid_sample(frame(can_id, payload, fd, flags))
        for offset in (20, 24, 28, 32, 36, 40, 48):
            invalid = bytearray(FIXTURE)
            struct.pack_into("<f", invalid, offset, float("nan"))
            with self.assertRaises(dc.SysIdProtocolError):
                dc.decode_sysid_sample(frame(0x6F2, bytes(invalid), True, 1))


if __name__ == "__main__":
    unittest.main()
