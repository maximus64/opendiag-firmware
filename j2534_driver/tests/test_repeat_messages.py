#!/usr/bin/env python3
"""Repeat-message acceptance tests through actual DLLs and the firmware core.

The transport peer dispatches into the real firmware core. See
docs/j2534_repeat_messages_tests.md for the physical validation boundary.
"""
import argparse
from contextlib import contextmanager
import ctypes as c
from pathlib import Path
import shutil
import tempfile
import unittest

from firmware_peer import Firmware
from mock_peer import pb
from run_tests import Server, configure

BUILD = None
FIRMWARE = None
U32 = c.c_uint32
START_REPEAT = 0x8004
QUERY_REPEAT = 0x8005
STOP_REPEAT = 0x8006
UNTIL_MATCH = 0
WHILE_MATCH = 1
INVALID_MESSAGE = 10
INVALID_INTERVAL = 11
EXCEEDED_LIMIT = 12
INVALID_MESSAGE_ID = 13
EMPTY = 16
REQUEST = bytes.fromhex("68 6a f1 01 00")
RESPONSE = bytes.fromhex("48 6b 10 41 00")


class PackedStructure(c.Structure):
    _pack_ = 1
    _layout_ = "ms"


class Message0404(PackedStructure):
    _fields_ = [(name, U32) for name in (
        "ProtocolID", "RxStatus", "TxFlags", "Timestamp", "DataSize", "ExtraDataIndex"
    )] + [("Data", c.c_ubyte * 4128)]


class Message0500(PackedStructure):
    _fields_ = [(name, U32) for name in (
        "ProtocolID", "MsgHandle", "RxStatus", "TxFlags", "Timestamp", "DataLength", "ExtraDataIndex"
    )] + [("DataBuffer", c.POINTER(c.c_ubyte)), ("DataBufferSize", U32)]


class Repeat0404(PackedStructure):
    _fields_ = [("TimeInterval", U32), ("Condition", U32), ("RepeatMsgData", Message0404 * 3)]


class Repeat0500(PackedStructure):
    _fields_ = [("TimeInterval", U32), ("Condition", U32), ("RepeatMsgData", Message0500 * 3)]


class Resource(PackedStructure):
    _fields_ = [("Connector", U32), ("NumOfResources", U32), ("ResourceListPtr", c.POINTER(U32))]


class Config(PackedStructure):
    _fields_ = [("Parameter", U32), ("Value", U32)]


class ConfigList(PackedStructure):
    _fields_ = [("NumOfParams", U32), ("ConfigPtr", c.POINTER(Config))]


class LogicalDescriptor(PackedStructure):
    _fields_ = [("LocalTxFlags", U32), ("RemoteTxFlags", U32),
                ("LocalAddress", c.c_ubyte * 5), ("RemoteAddress", c.c_ubyte * 5)]


class Parameter(PackedStructure):
    _fields_ = [("Parameter", U32), ("Value", U32), ("Supported", U32)]


class ParameterList(PackedStructure):
    _fields_ = [("NumOfParams", U32), ("ParamPtr", c.POINTER(Parameter))]


def require_success(status, operation):
    if status:
        raise AssertionError(f"{operation}: expected STATUS_NOERROR (0), got 0x{status:02x}")


class Session:
    def __init__(self, root, version):
        self.version = version
        self.message_type = Message0404 if version == "0404" else Message0500
        self.repeat_type = Repeat0404 if version == "0404" else Repeat0500
        name = "libopendiag0404.so" if version == "0404" else "libopendiag.so"
        self.api = c.CDLL(str(root / name))
        msg = c.POINTER(self.message_type)
        connect = [U32, U32, U32, U32]
        if version == "0500":
            connect += [Resource]
        signatures = {
            "PassThruOpen": [c.c_void_p, c.POINTER(U32)],
            "PassThruClose": [U32],
            "PassThruConnect": connect + [c.POINTER(U32)],
            "PassThruDisconnect": [U32],
            "PassThruStopMsgFilter": [U32, U32],
            "PassThruReadMsgs": [U32, msg, c.POINTER(U32), U32],
            "PassThruStartPeriodicMsg": [U32, msg, c.POINTER(U32), U32],
            "PassThruStopPeriodicMsg": [U32, U32],
            "PassThruStartMsgFilter": [U32, U32, msg, msg] + ([msg] if version == "0404" else []) + [c.POINTER(U32)],
            "PassThruIoctl": [U32, U32, c.c_void_p, c.c_void_p],
        }
        if version == "0404":
            signatures["PassThruWriteMsgs"] = [U32, msg, c.POINTER(U32), U32]
        else:
            signatures["PassThruQueueMsgs"] = [U32, msg, c.POINTER(U32)]
        for name, arguments in signatures.items():
            function = getattr(self.api, name)
            function.argtypes = arguments
            function.restype = c.c_int32
        self.device = U32()
        self.channel = None

    def connect(self, protocol=3, flags=None):
        pins = {1: [2], 2: [2, 10], 3: [7], 5: [6, 14], 6: [6, 14]}[protocol]
        rate = {1: 10400, 2: 41600, 3: 10400, 5: 500000, 6: 500000}[protocol]
        if flags is None:
            flags = 0x1000 if protocol == 3 else 0
        arguments = [self.device, protocol, flags, rate]
        if self.version == "0500":
            resources = (U32 * len(pins))(*pins)
            arguments.append(Resource(1, len(pins), resources))
        channel = U32()
        require_success(self.api.PassThruConnect(*arguments, c.byref(channel)), "PassThruConnect")
        self.channel, self.protocol = channel.value, protocol
        return self.channel

    def message(self, data, flags=0, protocol=None):
        result = self.message_type()
        result.ProtocolID = self.protocol if protocol is None else protocol
        result.TxFlags = flags
        if self.version == "0404":
            result.DataSize = len(data)
            result.Data[:len(data)] = data
        else:
            result._buffer = (c.c_ubyte * max(len(data), 1))(*data)
            result.DataBuffer = result._buffer
            result.DataBufferSize = len(result._buffer)
            result.DataLength = len(data)
            result.MsgHandle = 42
        return result

    def setup(self, interval=100, condition=UNTIL_MATCH, request=REQUEST, mask=None, pattern=RESPONSE):
        if mask is None:
            mask = b"\xff" * len(pattern)
        result = self.repeat_type()
        result.TimeInterval, result.Condition = interval, condition
        result._messages = [self.message(data) for data in (request, mask, pattern)]
        for index, item in enumerate(result._messages):
            result.RepeatMsgData[index] = item
        return result

    def ioctl(self, operation, source=None, output=None, channel=None):
        return self.api.PassThruIoctl(self.channel if channel is None else channel, operation,
                                     None if source is None else c.byref(source),
                                     None if output is None else c.byref(output))

    def start(self, setup=None):
        setup = self.setup() if setup is None else setup
        identifier = U32()
        require_success(self.ioctl(START_REPEAT, setup, identifier), "START_REPEAT_MESSAGE")
        return identifier.value

    def query(self, identifier):
        result = U32(0xffffffff)
        require_success(self.ioctl(QUERY_REPEAT, U32(identifier), result), "QUERY_REPEAT_MESSAGE")
        return result.value

    def stop(self, identifier):
        return self.ioctl(STOP_REPEAT, U32(identifier))

    def periodic(self, data=REQUEST, interval=100):
        message, identifier = self.message(data), U32()
        require_success(self.api.PassThruStartPeriodicMsg(self.channel, c.byref(message),
                                                        c.byref(identifier), interval), "StartPeriodicMsg")
        return identifier.value

    def filter(self, kind=1):
        mask, pattern, identifier = self.message(b"\0"), self.message(b"\0"), U32()
        arguments = [self.channel, kind, c.byref(mask), c.byref(pattern)]
        if self.version == "0404":
            arguments.append(None)
        require_success(self.api.PassThruStartMsgFilter(*arguments, c.byref(identifier)), "StartMsgFilter")

    def config(self, parameter, value):
        entry = Config(parameter, value)
        return self.ioctl(2, ConfigList(1, c.pointer(entry)))

    def flow_filter(self, receive=0x7e8, transmit=0x7e0):
        mask = self.message(b"\xff" * 4)
        pattern = self.message(receive.to_bytes(4, "big"))
        flow = self.message(transmit.to_bytes(4, "big"))
        identifier = U32()
        require_success(self.api.PassThruStartMsgFilter(
            self.channel, 3, c.byref(mask), c.byref(pattern), c.byref(flow),
            c.byref(identifier)), "StartMsgFilter(FLOW_CONTROL)")
        return identifier.value

    def write(self, data=REQUEST):
        message, count = self.message(data), U32(1)
        if self.version == "0404":
            status = self.api.PassThruWriteMsgs(self.channel, c.byref(message), c.byref(count), 0)
        else:
            status = self.api.PassThruQueueMsgs(self.channel, c.byref(message), c.byref(count))
        require_success(status, "WriteMsgs/QueueMsgs")
        if count.value != 1:
            raise AssertionError("Successful write did not accept one message")

    def read(self):
        message, count = self.message(bytes(4128)), U32(1)
        status = self.api.PassThruReadMsgs(self.channel, c.byref(message), c.byref(count), 0)
        if self.version == "0404":
            data = bytes(message.Data[:message.DataSize])
        else:
            data = bytes(message.DataBuffer[:message.DataLength])
        return status, count.value, data, message.RxStatus


class ContractTests(unittest.TestCase):
    @contextmanager
    def session(self, protocol=3, version="0404", peer_factory=None):
        firmware = Firmware(FIRMWARE)
        try:
            with tempfile.TemporaryDirectory(prefix="opendiag-repeat-") as temporary:
                root = Path(temporary)
                for name in ("libopendiag.so", "libopendiag0404.so"):
                    shutil.copyfile(BUILD / name, root / name)
                peer = firmware.peer_type() if peer_factory is None else peer_factory(firmware)
                with Server(peer) as server:
                    configure(root, server.port)
                    session = Session(root, version)
                    try:
                        name = b"J2534-1:" if version == "0500" else None
                        require_success(session.api.PassThruOpen(name, c.byref(session.device)), "PassThruOpen")
                        session.connect(protocol)
                        yield session, firmware
                    finally:
                        if session.device.value:
                            require_success(session.api.PassThruClose(session.device), "PassThruClose")
        finally:
            firmware.stop()


class HarnessTests(ContractTests):
    def test_fixture_abi_layouts(self):
        self.assertEqual(c.sizeof(Message0404), 4152)
        self.assertEqual(c.sizeof(Repeat0404), 12464)
        self.assertEqual(Repeat0404.RepeatMsgData.offset, 8)
        self.assertEqual(c.sizeof(Message0500), 32 + c.sizeof(c.c_void_p))
        self.assertEqual(c.sizeof(Repeat0500), 8 + 3 * c.sizeof(Message0500))

    def test_existing_periodic_reaches_real_firmware_from_both_apis(self):
        for version in ("0404", "0500"):
            with self.subTest(version=version), self.session(version=version) as (session, firmware):
                identifier = session.periodic()
                self.assertEqual(firmware.sent(), [REQUEST])
                firmware.advance(101)
                self.assertEqual(firmware.sent(), [REQUEST, REQUEST])
                self.assertEqual(session.api.PassThruStopPeriodicMsg(session.channel, identifier), 0)
                firmware.advance(101)
                self.assertEqual(len(firmware.sent()), 2)

    def test_existing_rx_filter_and_checksum_removal(self):
        with self.session() as (session, firmware):
            firmware.inject(RESPONSE)
            self.assertEqual(session.read()[:2], (EMPTY, 0))
            session.filter()
            firmware.inject(RESPONSE)
            self.assertEqual(session.read()[:3], (0, 1, RESPONSE))

    def test_existing_periodic_interval_is_start_to_start(self):
        with self.session() as (session, firmware):
            firmware.send_duration(7)
            session.periodic(interval=20)
            firmware.advance(14)
            self.assertEqual(firmware.sent(), [REQUEST, REQUEST])

    def test_existing_write_works_on_each_supported_physical_bus(self):
        for protocol, data in ((1, REQUEST), (2, REQUEST), (3, REQUEST),
                               (5, bytes.fromhex("00 00 07 df 01 00"))):
            with self.subTest(protocol=protocol), self.session(protocol) as (session, firmware):
                session.write(data)
                self.assertEqual(firmware.sent(protocol), [data])


class RepeatMessageTests(ContractTests):
    def test_null_required_arguments_are_rejected(self):
        for operation, missing in ((START_REPEAT, "input"), (START_REPEAT, "output"),
                                   (QUERY_REPEAT, "input"), (QUERY_REPEAT, "output"),
                                   (STOP_REPEAT, "input")):
            with self.subTest(operation=operation, missing=missing), self.session() as (session, firmware):
                source = session.setup() if operation == START_REPEAT else U32(1)
                output = None if operation == STOP_REPEAT else U32()
                if missing == "input":
                    source = None
                else:
                    output = None
                self.assertEqual(session.ioctl(operation, source, output), 4)
                self.assertEqual(firmware.sent(), [])

    def test_start_queues_first_copy_and_query_reports_active(self):
        with self.session() as (session, firmware):
            identifier = session.start()
            self.assertNotEqual(identifier, 0)
            self.assertEqual(firmware.sent(), [REQUEST])
            self.assertEqual(session.query(identifier), 1)

    def test_native_0500_start_query_stop_uses_pointer_message_layout(self):
        with self.session(version="0500") as (session, firmware):
            identifier = session.start()
            self.assertEqual(firmware.sent(), [REQUEST])
            self.assertEqual(session.query(identifier), 1)
            self.assertEqual(session.stop(identifier), 0)
            firmware.advance(101)
            self.assertEqual(len(firmware.sent()), 1)

    def test_interval_bounds_are_accepted(self):
        for interval in (5, 65535):
            with self.subTest(interval=interval), self.session() as (session, firmware):
                session.start(session.setup(interval=interval))
                self.assertEqual(firmware.sent(), [REQUEST])

    def test_invalid_intervals_have_no_transmit_side_effect(self):
        for interval in (0, 4, 65536, 0xffffffff):
            with self.subTest(interval=interval), self.session() as (session, firmware):
                self.assertEqual(session.ioctl(START_REPEAT, session.setup(interval=interval), U32()), INVALID_INTERVAL)
                firmware.advance(100)
                self.assertEqual(firmware.sent(), [])

    def test_invalid_condition_is_rejected_without_transmission(self):
        for condition in (2, 0xffffffff):
            with self.subTest(condition=condition), self.session() as (session, firmware):
                setup = session.setup(condition=condition)
                self.assertEqual(session.ioctl(START_REPEAT, setup, U32()), 5)
                self.assertEqual(firmware.sent(), [])

    def test_oversize_tx_mask_and_pattern_are_rejected(self):
        for index in range(3):
            with self.subTest(message=index), self.session() as (session, firmware):
                setup = session.setup()
                setup.RepeatMsgData[index].DataSize = 13
                self.assertEqual(session.ioctl(START_REPEAT, setup, U32()), INVALID_MESSAGE)
                self.assertEqual(firmware.sent(), [])

    def test_twelve_byte_kline_messages_are_accepted(self):
        with self.session() as (session, firmware):
            request = REQUEST + bytes(7)
            session.start(session.setup(request=request, pattern=RESPONSE + bytes(7)))
            self.assertEqual(firmware.sent(), [request])

    def test_invalid_flags_in_each_message_are_rejected(self):
        for index in range(3):
            with self.subTest(message=index), self.session() as (session, firmware):
                setup = session.setup()
                setup.RepeatMsgData[index].TxFlags = 0x80000000
                self.assertEqual(session.ioctl(START_REPEAT, setup, U32()), INVALID_MESSAGE)
                self.assertEqual(firmware.sent(), [])

    def test_mask_and_pattern_lengths_must_agree(self):
        with self.session() as (session, firmware):
            setup = session.setup(mask=b"\xff" * 4)
            self.assertEqual(session.ioctl(START_REPEAT, setup, U32()), INVALID_MESSAGE)
            self.assertEqual(firmware.sent(), [])

    def test_mask_and_pattern_flags_must_agree(self):
        with self.session() as (session, firmware):
            setup = session.setup()
            setup.RepeatMsgData[2].TxFlags = 0x200
            self.assertEqual(session.ioctl(START_REPEAT, setup, U32()), INVALID_MESSAGE)
            self.assertEqual(firmware.sent(), [])

    def test_until_match_continues_through_silence(self):
        with self.session() as (session, firmware):
            identifier = session.start()
            firmware.advance(101)
            self.assertEqual(firmware.sent(), [REQUEST, REQUEST])
            self.assertEqual(session.query(identifier), 1)

    def test_until_match_ignores_nonmatching_response(self):
        with self.session() as (session, firmware):
            identifier = session.start()
            firmware.inject(RESPONSE[:-1] + b"\x01")
            firmware.advance(101)
            self.assertEqual(len(firmware.sent()), 2)
            self.assertEqual(session.query(identifier), 1)

    def test_matching_response_stops_without_host_reads(self):
        with self.session() as (session, firmware):
            identifier = session.start()
            firmware.inject(RESPONSE)
            firmware.advance(101)
            self.assertEqual(firmware.sent(), [REQUEST])
            self.assertEqual(session.query(identifier), 0)

    def test_terminating_response_is_delivered_with_pass_filter(self):
        with self.session() as (session, firmware):
            session.filter()
            identifier = session.start()
            firmware.inject(RESPONSE)
            firmware.poll()
            self.assertEqual(session.query(identifier), 0)
            self.assertEqual(session.read()[:3], (0, 1, RESPONSE))

    def test_blocked_response_still_terminates_repeat(self):
        with self.session() as (session, firmware):
            session.filter()
            session.filter(kind=2)
            identifier = session.start()
            firmware.inject(RESPONSE)
            firmware.advance(101)
            self.assertEqual(session.query(identifier), 0)
            self.assertEqual(session.read()[:2], (EMPTY, 0))
            self.assertEqual(len(firmware.sent()), 1)

    def test_mask_ignores_bits_and_extra_received_bytes(self):
        with self.session() as (session, firmware):
            identifier = session.start(session.setup(mask=b"\xff\xff\xff\xf0\x00",
                                                    pattern=bytes.fromhex("48 6b 10 40 00")))
            firmware.inject(bytes.fromhex("48 6b 10 4f ab cd"))
            firmware.advance(101)
            self.assertEqual(session.query(identifier), 0)
            self.assertEqual(len(firmware.sent()), 1)

    def test_short_response_does_not_match_even_an_all_zero_mask(self):
        with self.session() as (session, firmware):
            identifier = session.start(session.setup(mask=bytes(5), pattern=bytes(5)))
            firmware.inject(RESPONSE[:4])
            firmware.advance(101)
            self.assertEqual(session.query(identifier), 1)
            self.assertEqual(len(firmware.sent()), 2)

    def test_while_match_stops_on_silence(self):
        with self.session() as (session, firmware):
            identifier = session.start(session.setup(condition=WHILE_MATCH))
            firmware.advance(101)
            self.assertEqual(session.query(identifier), 0)
            self.assertEqual(firmware.sent(), [REQUEST])

    def test_while_match_requires_response_in_each_interval(self):
        with self.session() as (session, firmware):
            identifier = session.start(session.setup(condition=WHILE_MATCH))
            firmware.inject(RESPONSE)
            firmware.advance(101)
            self.assertEqual(session.query(identifier), 1)
            self.assertEqual(len(firmware.sent()), 2)
            firmware.advance(101)
            self.assertEqual(session.query(identifier), 0)
            self.assertEqual(len(firmware.sent()), 2)

    def test_while_match_stops_on_any_nonmatch_even_after_a_match(self):
        with self.session() as (session, firmware):
            identifier = session.start(session.setup(condition=WHILE_MATCH))
            firmware.inject(RESPONSE)
            firmware.inject(RESPONSE[:-1] + b"\x01")
            firmware.advance(101)
            self.assertEqual(session.query(identifier), 0)
            self.assertEqual(len(firmware.sent()), 1)

    def test_repeat_interval_starts_after_transmission_finishes(self):
        with self.session() as (session, firmware):
            firmware.send_duration(7)
            session.start(session.setup(interval=20))
            firmware.advance(18)
            self.assertEqual(len(firmware.sent()), 1)
            firmware.advance(3)
            self.assertEqual(len(firmware.sent()), 2)

    def test_repeat_deadline_survives_microsecond_clock_wrap(self):
        with self.session() as (session, firmware):
            firmware.advance((1 << 32) // 1000 - 10)
            session.start(session.setup(interval=20))
            firmware.advance(18)
            self.assertEqual(len(firmware.sent()), 1)
            firmware.advance(3)
            self.assertEqual(len(firmware.sent()), 2)

    def test_stop_invalidates_id_and_cancels_future_transmits(self):
        with self.session() as (session, firmware):
            identifier = session.start()
            self.assertEqual(session.stop(identifier), 0)
            self.assertEqual(session.stop(identifier), INVALID_MESSAGE_ID)
            self.assertEqual(session.ioctl(QUERY_REPEAT, U32(identifier), U32()), INVALID_MESSAGE_ID)
            firmware.advance(101)
            self.assertEqual(len(firmware.sent()), 1)

    def test_stop_removes_copy_queued_before_first_transmission(self):
        with self.session() as (session, firmware):
            firmware.auto_poll = False
            identifier = session.start()
            self.assertEqual(session.stop(identifier), 0)
            firmware.advance(101)
            self.assertEqual(firmware.sent(), [])

    def test_automatic_completion_retains_id_until_stop(self):
        with self.session() as (session, firmware):
            identifier = session.start()
            firmware.inject(RESPONSE)
            firmware.advance(101)
            self.assertEqual(session.query(identifier), 0)
            self.assertEqual(session.query(identifier), 0)
            self.assertEqual(session.stop(identifier), 0)
            self.assertEqual(session.ioctl(QUERY_REPEAT, U32(identifier), U32()), INVALID_MESSAGE_ID)

    def test_duplicate_definitions_allocate_independent_ids(self):
        with self.session() as (session, firmware):
            first, second = session.start(), session.start()
            self.assertNotEqual(first, second)
            self.assertEqual(session.stop(first), 0)
            self.assertEqual(session.query(second), 1)
            before = len(firmware.sent())
            firmware.advance(101)
            self.assertEqual(len(firmware.sent()), before + 1)

    def test_ten_repeat_definitions_fit_alongside_ten_periodics(self):
        with self.session() as (session, firmware):
            firmware.auto_poll = False
            for _ in range(10):
                session.periodic()
            identifiers = {session.start() for _ in range(10)}
            self.assertEqual(len(identifiers), 10)

    def test_finished_definitions_hold_capacity_until_explicit_stop(self):
        with self.session() as (session, firmware):
            identifiers = [session.start() for _ in range(10)]
            firmware.inject(RESPONSE)
            firmware.poll()
            for identifier in identifiers:
                self.assertEqual(session.query(identifier), 0)
            # OpenDiag's planned capacity is ten; discovery must report it.
            self.assertEqual(session.ioctl(START_REPEAT, session.setup(), U32()), EXCEEDED_LIMIT)
            self.assertEqual(session.stop(identifiers[0]), 0)
            replacement = session.start()
            self.assertNotIn(replacement, identifiers)
            self.assertEqual(session.query(replacement), 1)

    def test_invalid_id_is_not_treated_as_stop_all(self):
        for identifier in (0, 0xdeadbeef):
            with self.subTest(identifier=identifier), self.session() as (session, firmware):
                self.assertEqual(session.stop(identifier), INVALID_MESSAGE_ID)
                self.assertEqual(session.ioctl(QUERY_REPEAT, U32(identifier), U32()), INVALID_MESSAGE_ID)
                self.assertEqual(firmware.sent(), [])

    def test_periodic_id_is_not_a_repeat_id(self):
        with self.session() as (session, firmware):
            identifier = session.periodic()
            self.assertEqual(session.stop(identifier), INVALID_MESSAGE_ID)
            firmware.advance(101)
            self.assertEqual(firmware.sent(), [REQUEST, REQUEST])

    def test_repeat_id_cannot_control_a_different_channel(self):
        with self.session() as (session, firmware):
            identifier = session.start()
            original = session.channel
            session.connect(5)
            self.assertEqual(session.stop(identifier), INVALID_MESSAGE_ID)
            self.assertEqual(session.ioctl(QUERY_REPEAT, U32(identifier), U32()), INVALID_MESSAGE_ID)
            session.channel = original
            self.assertEqual(session.query(identifier), 1)

    def test_disconnect_prevents_further_repeat_transmissions(self):
        with self.session() as (session, firmware):
            session.start()
            self.assertEqual(session.api.PassThruDisconnect(session.channel), 0)
            firmware.advance(101)
            self.assertEqual(len(firmware.sent()), 1)

    def test_close_prevents_further_repeat_transmissions(self):
        with self.session() as (session, firmware):
            session.start()
            self.assertEqual(session.api.PassThruClose(session.device), 0)
            session.device = U32()
            firmware.advance(101)
            self.assertEqual(len(firmware.sent()), 1)

    def test_clearing_periodics_leaves_repeat_definition_active(self):
        with self.session() as (session, firmware):
            identifier = session.start()
            session.periodic(REQUEST[:-1] + b"\x01")
            self.assertEqual(session.ioctl(9), 0)
            before = len(firmware.sent())
            firmware.advance(101)
            self.assertEqual(session.query(identifier), 1)
            self.assertEqual(firmware.sent()[before:], [REQUEST])

    def test_loopback_does_not_count_as_an_ecu_response(self):
        with self.session() as (session, firmware):
            self.assertEqual(session.config(3, 1), 0)
            identifier = session.start(session.setup(condition=WHILE_MATCH, mask=bytes(5), pattern=bytes(5)))
            firmware.advance(101)
            self.assertEqual(session.query(identifier), 0)
            status, count, data, flags = session.read()
            self.assertEqual((status, count, data), (0, 1, REQUEST))
            self.assertTrue(flags & 1)
            self.assertEqual(len(firmware.sent()), 1)

    def test_repeat_due_now_has_priority_over_ordinary_queued_message(self):
        with self.session() as (session, firmware):
            firmware.auto_poll = False
            session.write(REQUEST[:-1] + b"\x02")
            session.start()
            firmware.poll()
            self.assertEqual(firmware.sent()[0], REQUEST)

    def test_slow_bus_does_not_starve_later_repeat_definitions(self):
        with self.session() as (session, firmware):
            firmware.auto_poll = False
            firmware.send_duration(7)
            requests = [REQUEST[:-1] + bytes([index]) for index in range(10)]
            for request in requests:
                session.start(session.setup(interval=5, request=request))
            for _ in requests:
                firmware.poll()
            self.assertEqual(set(firmware.sent()), set(requests))

    def test_j1850_repeat_support_uses_same_receive_condition(self):
        for protocol in (1, 2):
            with self.subTest(protocol=protocol), self.session(protocol) as (session, firmware):
                identifier = session.start()
                firmware.inject(RESPONSE, protocol=protocol)
                firmware.advance(101)
                self.assertEqual(session.query(identifier), 0)
                self.assertEqual(firmware.sent(protocol), [REQUEST])

    def test_can_repeat_uses_address_and_data_in_matching(self):
        request = bytes.fromhex("00 00 07 df 01 00")
        response = bytes.fromhex("00 00 07 e8 41 00")
        with self.session(5) as (session, firmware):
            identifier = session.start(session.setup(request=request, pattern=response))
            firmware.inject(response, protocol=5)
            firmware.advance(101)
            self.assertEqual(session.query(identifier), 0)
            self.assertEqual(firmware.sent(5), [request])

    def test_isotp_matches_complete_payload_instead_of_can_pci_bytes(self):
        request = bytes.fromhex("00 00 07 e0 01 00")
        response = bytes.fromhex("00 00 07 e8 41 00")
        with self.session(6) as (session, firmware):
            session.flow_filter()
            identifier = session.start(session.setup(request=request, pattern=response))
            firmware.inject(bytes.fromhex("00 00 07 e8 02 41 00"), protocol=5)
            firmware.advance(101)
            self.assertEqual(session.query(identifier), 0)
            self.assertEqual(firmware.sent(5), [bytes.fromhex("00 00 07 e0 02 01 00")])

    def test_isotp_partial_response_does_not_finish_repeat(self):
        request = bytes.fromhex("00 00 07 e0 01 00")
        response = bytes.fromhex("00 00 07 e8 41 00")
        with self.session(6) as (session, firmware):
            session.flow_filter()
            identifier = session.start(session.setup(request=request, pattern=response))
            firmware.inject(bytes.fromhex("00 00 07 e8 10 08 41 00 01 02 03 04"), protocol=5)
            self.assertEqual(session.query(identifier), 1)
            firmware.inject(bytes.fromhex("00 00 07 e8 21 05 06"), protocol=5)
            self.assertEqual(session.query(identifier), 0)

    def test_discovery_reports_repeat_count_and_message_length(self):
        with self.session() as (session, firmware):
            parameters = (Parameter * 2)(Parameter(8, 0, 0), Parameter(9, 0, 0))
            output = ParameterList(2, parameters)
            self.assertEqual(session.ioctl(0x800d, U32(3), output, channel=session.device.value), 0)
            self.assertEqual([item.Supported for item in parameters], [1, 1])
            self.assertEqual([item.Value for item in parameters], [10, 12])


class IntegrationTests(ContractTests):
    def test_older_firmware_reports_zero_limits_and_rejects_all_repeat_operations(self):
        calls = []

        def old_peer(firmware):
            class Peer(firmware.peer_type()):
                def respond(self, request):
                    name = request.WhichOneof("command")
                    calls.append(name)
                    response = super().respond(request)
                    if name == "capabilities":
                        response.capabilities.ClearField("repeat_per_channel")
                        for limit in response.capabilities.protocols:
                            limit.ClearField("max_repeat_data")
                    return response
            return Peer

        for version in ("0404", "0500"):
            with self.subTest(version=version), self.session(version=version, peer_factory=old_peer) as (session, firmware):
                parameters = (Parameter * 2)(Parameter(8, 99, 99), Parameter(9, 99, 99))
                self.assertEqual(session.ioctl(0x800d, U32(3), ParameterList(2, parameters),
                                               channel=session.device.value), 0)
                self.assertEqual([(p.Value, p.Supported) for p in parameters], [(0, 1), (0, 1)])
                self.assertEqual(session.ioctl(START_REPEAT, session.setup(), U32()), 1)
                self.assertEqual(session.ioctl(QUERY_REPEAT, U32(123), U32()), 1)
                self.assertEqual(session.stop(123), 1)
                self.assertEqual(firmware.sent(), [])
                session.write()
                self.assertEqual(firmware.sent(), [REQUEST])
        self.assertFalse(set(calls) & {"start_repeat", "query_repeat", "stop_repeat"})

    def test_lost_mutating_reply_is_not_replayed(self):
        for version in ("0404", "0500"):
            for operation, command in ((START_REPEAT, "start_repeat"), (STOP_REPEAT, "stop_repeat")):
                calls = []

                def dropping_peer(firmware):
                    class Peer(firmware.peer_type()):
                        def respond(self, request):
                            name = request.WhichOneof("command")
                            calls.append(name)
                            response = super().respond(request)
                            if name == command:
                                raise EOFError("Reply lost after applying mutation")
                            return response
                    return Peer

                with self.subTest(version=version, command=command), self.session(version=version, peer_factory=dropping_peer) as (session, firmware):
                    source = session.setup() if operation == START_REPEAT else U32(session.start())
                    result = U32(0x12345678)
                    self.assertEqual(session.ioctl(operation, source, result if operation == START_REPEAT else None), 8)
                    self.assertEqual(result.value, 0x12345678)
                    self.assertEqual(session.ioctl(QUERY_REPEAT, U32(1), result), 8)
                    self.assertEqual(calls.count(command), 1)
                    self.assertEqual(session.api.PassThruClose(session.device), 8)
                    session.device.value = 0

    def test_discovery_preserves_unknown_parameters_and_uses_device_ownership(self):
        for version in ("0404", "0500"):
            with self.subTest(version=version), self.session(version=version) as (session, firmware):
                parameters = (Parameter * 3)(Parameter(8, 55, 0), Parameter(0xfedc, 42, 99), Parameter(9, 55, 0))
                output = ParameterList(3, parameters)
                self.assertEqual(session.ioctl(0x800d, U32(3), output, channel=session.device.value), 0)
                self.assertEqual([(p.Value, p.Supported) for p in parameters], [(10, 1), (42, 0), (12, 1)])
                self.assertEqual(session.ioctl(0x800d, U32(3), output), 26)
                self.assertEqual(session.ioctl(0x800d, U32(0xff), output, channel=session.device.value), 3)
                self.assertEqual(session.ioctl(0x800c, None, output, channel=session.device.value), 0)
                self.assertEqual([p.Supported for p in parameters], [0, 0, 0])
                self.assertEqual(session.ioctl(0x800d, None, output, channel=session.device.value), 4)
                self.assertEqual(session.ioctl(0x800d, U32(3), ParameterList(1, None), channel=session.device.value), 4)

    def test_device_discovery_translates_connector_and_protocol_layouts(self):
        for version in ("0404", "0500"):
            with self.subTest(version=version), self.session(version=version) as (session, firmware):
                parameters = (Parameter * 2)(Parameter(4, 0x10000, 0), Parameter(6, 1, 0))
                self.assertEqual(session.ioctl(0x800c, None, ParameterList(2, parameters),
                                               channel=session.device.value), 0)
                expected = 1 if version == "0404" else 0x10001
                self.assertEqual([(p.Value, p.Supported) for p in parameters], [(expected, 1)] * 2)
                if version == "0404":
                    iso = (Parameter * 1)(Parameter(7, 0, 0))
                    self.assertEqual(session.ioctl(0x800c, None, ParameterList(1, iso),
                                                   channel=session.device.value), 0)
                    self.assertEqual((iso[0].Value, iso[0].Supported), (1, 1))

    def test_legacy_device_discovery_reports_absent_protocol_without_failing_query(self):
        def missing_iso(firmware):
            class Peer(firmware.peer_type()):
                def respond(self, request):
                    response = super().respond(request)
                    if request.WhichOneof("command") == "capabilities":
                        limits = [item for item in response.capabilities.protocols if item.protocol != 0x200]
                        del response.capabilities.protocols[:]
                        response.capabilities.protocols.extend(limits)
                    return response
            return Peer

        with self.session(peer_factory=missing_iso) as (session, firmware):
            parameters = (Parameter * 1)(Parameter(7, 99, 99))
            self.assertEqual(session.ioctl(0x800c, None, ParameterList(1, parameters),
                                           channel=session.device.value), 0)
            self.assertEqual((parameters[0].Value, parameters[0].Supported), (0, 1))

    def test_native_message_buffer_validation_precedes_copy_and_mutation(self):
        for index in range(3):
            for invalid in ("null", "short", "oversize", "empty", "protocol"):
                with self.subTest(index=index, invalid=invalid), self.session(version="0500") as (session, firmware):
                    setup = session.setup()
                    message = setup.RepeatMsgData[index]
                    expected = 10
                    if invalid == "null":
                        message.DataBuffer = None
                        expected = 4
                    elif invalid == "short":
                        message.DataBufferSize = message.DataLength - 1
                    elif invalid == "oversize":
                        message.DataLength = 13
                        message.DataBufferSize = 13
                    elif invalid == "empty":
                        message.DataLength = 0
                    else:
                        message.ProtocolID = 5
                        expected = 21
                    result = U32(0x12345678)
                    self.assertEqual(session.ioctl(START_REPEAT, setup, result), expected)
                    self.assertEqual(result.value, 0x12345678)
                    self.assertEqual(firmware.sent(), [])

    def test_native_receive_conditions_and_owned_input_storage(self):
        for condition in (UNTIL_MATCH, WHILE_MATCH):
            with self.subTest(condition=condition), self.session(version="0500") as (session, firmware):
                setup = session.setup(condition=condition)
                identifier = session.start(setup)
                for message in setup._messages:
                    c.memset(message.DataBuffer, 0, message.DataLength)
                firmware.inject(RESPONSE)
                firmware.advance(101)
                self.assertEqual(session.query(identifier), 0 if condition == UNTIL_MATCH else 1)
                self.assertEqual(firmware.sent(), [REQUEST] if condition == UNTIL_MATCH else [REQUEST, REQUEST])
                firmware.advance(101)
                self.assertEqual(session.query(identifier), 0)

    def test_native_can_29bit_and_extended_isotp_use_normalized_repeat_matches(self):
        for logical in (False, True):
            with self.subTest(logical=logical), self.session(version="0500") as (session, firmware):
                self.assertEqual(session.api.PassThruDisconnect(session.channel), 0)
                physical = session.connect(5, flags=0x100)
                tx_address = bytes.fromhex("18 da 10 f1")
                rx_address = bytes.fromhex("18 da f1 10")
                flags = 0x100
                if logical:
                    flags |= 0x80
                    tx_address += b"\x10"
                    rx_address += b"\xf1"
                    descriptor = LogicalDescriptor(flags, flags)
                    descriptor.LocalAddress[:] = rx_address
                    descriptor.RemoteAddress[:] = tx_address
                    api = session.api.PassThruLogicalConnect
                    api.argtypes = [U32, U32, U32, c.c_void_p, c.POINTER(U32)]
                    api.restype = c.c_int32
                    channel = U32()
                    self.assertEqual(api(physical, 0x200, 0, c.byref(descriptor), c.byref(channel)), 0)
                    session.channel, session.protocol = channel.value, 0x200
                setup = session.setup(request=tx_address + b"\x01\x00", pattern=rx_address + b"\x41\x00")
                for message in setup.RepeatMsgData:
                    message.TxFlags = flags
                identifier = session.start(setup)
                # The fake CAN driver stores the EFF marker in its native frame ID.
                frame = bytes.fromhex("98 da f1 10") + (b"\xf1\x02" if logical else b"") + b"\x41\x00"
                firmware.inject(frame, protocol=5)
                self.assertEqual(session.query(identifier), 0)
                self.assertEqual(len(firmware.sent(5)), 1)
                if logical:
                    self.assertEqual(session.api.PassThruDisconnect(physical), 0)
                    self.assertEqual(session.ioctl(QUERY_REPEAT, U32(identifier), U32()), 2)

    def test_clear_tx_preserves_repeat_id_and_scheduling_in_both_apis(self):
        for version in ("0404", "0500"):
            with self.subTest(version=version), self.session(version=version) as (session, firmware):
                identifier = session.start()
                self.assertEqual(session.ioctl(7), 0)
                self.assertEqual(session.query(identifier), 1)
                firmware.advance(101)
                self.assertEqual(firmware.sent(), [REQUEST, REQUEST])
                self.assertEqual(session.stop(identifier), 0)

    def test_clear_rx_still_evaluates_repeat_conditions(self):
        for version in ("0404", "0500"):
            with self.subTest(version=version), self.session(version=version) as (session, firmware):
                session.filter()
                identifier = session.start()
                firmware.inject(RESPONSE)
                self.assertEqual(session.ioctl(8), 0)
                self.assertEqual(session.query(identifier), 0)
                self.assertEqual(session.read()[:2], (EMPTY, 0))

    def test_stop_output_must_be_null_and_error_preserves_definition(self):
        for version, error in (("0404", 5), ("0500", 28)):
            with self.subTest(version=version), self.session(version=version) as (session, firmware):
                identifier = session.start()
                self.assertEqual(session.ioctl(STOP_REPEAT, U32(identifier), U32()), error)
                self.assertEqual(session.query(identifier), 1)
                self.assertEqual(session.stop(identifier), 0)

    def test_iso_repeat_keeps_transport_after_filter_removal_and_hides_received_data(self):
        with self.session(6) as (session, firmware):
            filter_id = session.flow_filter()
            setup = session.setup(request=bytes.fromhex("00 00 07 e0 01 00"),
                                  pattern=bytes.fromhex("00 00 07 e8 41 00"))
            identifier = session.start(setup)
            self.assertEqual(session.api.PassThruStopMsgFilter(session.channel, filter_id), 0)
            firmware.advance(101)
            self.assertEqual(session.query(identifier), 1)
            self.assertEqual(len(firmware.sent(5)), 2)
            for _ in range(2):
                status, count, data, flags = session.read()
                self.assertEqual((status, count, data), (0, 1, bytes.fromhex("00 00 07 e0")))
                self.assertEqual(flags & 9, 9)  # ISO-TP TX indications remain visible.
            firmware.inject(bytes.fromhex("00 00 07 e8 02 41 00"), protocol=5)
            self.assertEqual(session.query(identifier), 0)
            self.assertEqual(session.read()[:2], (EMPTY, 0))
            self.assertEqual(session.stop(identifier), 0)
            # A different peer can now use the released endpoint/address resources.
            session.flow_filter(receive=0x7e9)

    def test_iso_filter_removal_preserves_previously_queued_responses(self):
        with self.session(6) as (session, firmware):
            filter_id = session.flow_filter()
            identifier = session.start(session.setup(request=bytes.fromhex("00 00 07 e0 01 00"),
                                                       pattern=bytes.fromhex("00 00 07 e8 41 00")))
            status, count, data, flags = session.read()
            self.assertEqual((status, count, flags & 9), (0, 1, 9))
            response = bytes.fromhex("00 00 07 e8 41 01")
            firmware.inject(bytes.fromhex("00 00 07 e8 02 41 01"), protocol=5)
            self.assertEqual(session.api.PassThruStopMsgFilter(session.channel, filter_id), 0)
            self.assertEqual(session.read()[:3], (0, 1, response))
            self.assertEqual(session.query(identifier), 1)

    def test_iso_filter_recreation_reuses_endpoint_and_discards_hidden_responses(self):
        with self.session(6) as (session, firmware):
            session.flow_filter()
            identifier = session.start(session.setup(request=bytes.fromhex("00 00 07 e0 01 00"),
                                                       pattern=bytes.fromhex("00 00 07 e8 41 00")))
            status, count, data, flags = session.read()
            self.assertEqual((status, count, data, flags & 9),
                             (0, 1, bytes.fromhex("00 00 07 e0"), 9))
            self.assertEqual(session.ioctl(10), 0)
            firmware.inject(bytes.fromhex("00 00 07 e8 02 41 00"), protocol=5)
            session.flow_filter()
            self.assertEqual(session.query(identifier), 0)
            self.assertEqual(session.read()[:2], (EMPTY, 0))
            response = bytes.fromhex("00 00 07 e8 41 01")
            firmware.inject(bytes.fromhex("00 00 07 e8 02 41 01"), protocol=5)
            self.assertEqual(session.read()[:3], (0, 1, response))

    def test_iso_repeat_without_receive_route_rejects_instead_of_matching_raw_can(self):
        with self.session(6) as (session, firmware):
            setup = session.setup(request=bytes.fromhex("00 00 07 df 01 00"),
                                  pattern=bytes.fromhex("00 00 07 e8 41 00"))
            self.assertEqual(session.ioctl(START_REPEAT, setup, U32()), 23)
            self.assertEqual(firmware.sent(5), [])

    def test_native_iso_repeat_conditions_remain_scoped_to_the_logical_channel(self):
        with self.session(5, version="0500") as (session, firmware):
            physical = session.channel
            api = session.api.PassThruLogicalConnect
            api.argtypes = [U32, U32, U32, c.c_void_p, c.POINTER(U32)]
            api.restype = c.c_int32
            channels = []
            for offset in (0, 1):
                descriptor = LogicalDescriptor()
                descriptor.LocalAddress[:4] = (0x7e8 + offset).to_bytes(4, "big")
                descriptor.RemoteAddress[:4] = (0x7e0 + offset).to_bytes(4, "big")
                channel = U32()
                self.assertEqual(api(physical, 0x200, 0, c.byref(descriptor), c.byref(channel)), 0)
                channels.append(channel.value)
            session.channel, session.protocol = channels[0], 0x200
            identifier = session.start(session.setup(condition=WHILE_MATCH,
                                                       request=bytes.fromhex("00 00 07 e0 01 00"),
                                                       pattern=bytes.fromhex("00 00 07 e8 41 00")))
            firmware.inject(bytes.fromhex("00 00 07 e9 02 41 00"), protocol=5)
            self.assertEqual(session.query(identifier), 1)
            firmware.inject(bytes.fromhex("00 00 07 e8 02 41 01"), protocol=5)
            self.assertEqual(session.query(identifier), 0)

    def test_legacy_repeat_conditions_observe_other_iso_peers_on_the_channel(self):
        for condition in (UNTIL_MATCH, WHILE_MATCH):
            for remove_filter in (False, True):
                with self.subTest(condition=condition, remove_filter=remove_filter), self.session(6) as (session, firmware):
                    session.flow_filter()
                    other_filter = session.flow_filter(receive=0x7e9, transmit=0x7e1)
                    setup = session.setup(condition=condition,
                                          request=bytes.fromhex("00 00 07 e0 01 00"),
                                          pattern=bytes.fromhex("00 00 07 e8 41 00"))
                    if condition == UNTIL_MATCH:
                        setup.RepeatMsgData[1].Data[:4] = bytes(4)
                        setup.RepeatMsgData[2].Data[:4] = bytes(4)
                    identifier = session.start(setup)
                    if remove_filter:
                        self.assertEqual(session.api.PassThruStopMsgFilter(session.channel, other_filter), 0)
                    firmware.inject(bytes.fromhex("00 00 07 e9 02 41 00"), protocol=5)
                    self.assertEqual(session.query(identifier), 0)
                    firmware.advance(101)
                    self.assertEqual(len(firmware.sent(5)), 1)

    def test_iso_repeat_quota_is_shared_across_legacy_endpoints(self):
        with self.session(6) as (session, firmware):
            firmware.auto_poll = False
            session.flow_filter()
            session.flow_filter(receive=0x7e9, transmit=0x7e1)
            for index in range(10):
                address = 0x7e0 + index % 2
                session.start(session.setup(request=address.to_bytes(4, "big") + b"\x01\x00",
                                             pattern=(address + 8).to_bytes(4, "big") + b"\x41\x00"))
            setup = session.setup(request=bytes.fromhex("00 00 07 e0 01 00"),
                                  pattern=bytes.fromhex("00 00 07 e8 41 00"))
            self.assertEqual(session.ioctl(START_REPEAT, setup, U32()), EXCEEDED_LIMIT)

    def test_wire_rejects_oversize_and_missing_repeat_messages_without_side_effects(self):
        firmware = Firmware(FIRMWARE)
        try:
            opened = firmware.respond(pb.Request(open=pb.Empty()))
            channel = firmware.respond(pb.Request(connect=pb.Connect(device=opened.id, protocol=3,
                                                                     flags=0x1000, baudrate=10400,
                                                                     connector=1, pins=[7])))
            self.assertEqual(channel.status, 0)
            for field in ("message", "mask", "pattern"):
                for malformed in ("missing", "oversize"):
                    with self.subTest(field=field, malformed=malformed):
                        repeat = pb.Repeat(channel=channel.id, interval_ms=100,
                                           message=pb.RepeatMessage(protocol=3, data=REQUEST),
                                           mask=pb.RepeatMessage(protocol=3, data=b"\xff" * 5),
                                           pattern=pb.RepeatMessage(protocol=3, data=RESPONSE))
                        if malformed == "missing":
                            repeat.ClearField(field)
                        else:
                            getattr(repeat, field).data = bytes(13)
                        response = firmware.respond(pb.Request(start_repeat=repeat))
                        self.assertEqual(response.status, INVALID_MESSAGE)
            self.assertEqual(firmware.sent(), [])
        finally:
            firmware.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--firmware", type=Path, required=True)
    args, remaining = parser.parse_known_args()
    BUILD, FIRMWARE = args.build.resolve(), args.firmware.resolve()
    unittest.main(argv=[__file__] + remaining, verbosity=2)
