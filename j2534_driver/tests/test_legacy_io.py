#!/usr/bin/env python3
"""04.04 I/O contracts and wire-operation budgets through the actual shared libraries."""
import argparse
from contextlib import contextmanager
import ctypes as c
from pathlib import Path
import shutil
import tempfile
import time
import unittest

import mock_peer
from run_tests import Server, configure

BUILD = None
U32 = c.c_uint32


class Message(c.Structure):
    _fields_ = [(name, U32) for name in (
        "ProtocolID", "RxStatus", "TxFlags", "Timestamp", "DataSize", "ExtraDataIndex"
    )] + [("Data", c.c_ubyte * 4128)]


def message(protocol=6, address=0x18DA11F1, payload=b"\x01"):
    result = Message()
    result.ProtocolID = protocol
    result.TxFlags = 0x100 if protocol in (5, 6) else 0
    data = address.to_bytes(4, "big") + payload
    result.DataSize = len(data)
    result.Data[:len(data)] = data
    return result


class Session:
    def __init__(self, library, protocol):
        self.api = c.CDLL(str(library))
        signatures = {
            "PassThruOpen": [c.c_void_p, c.POINTER(U32)],
            "PassThruClose": [U32],
            "PassThruConnect": [U32, U32, U32, U32, c.POINTER(U32)],
            "PassThruReadMsgs": [U32, c.POINTER(Message), c.POINTER(U32), U32],
            "PassThruWriteMsgs": [U32, c.POINTER(Message), c.POINTER(U32), U32],
            "PassThruStartMsgFilter": [U32, U32, c.POINTER(Message), c.POINTER(Message), c.POINTER(Message), c.POINTER(U32)],
            "PassThruStopMsgFilter": [U32, U32],
            "PassThruIoctl": [U32, U32, c.c_void_p, c.c_void_p],
        }
        for name, types in signatures.items():
            function = getattr(self.api, name)
            function.argtypes = types
            function.restype = c.c_int32
        self.device, self.channel = U32(), U32()
        assert self.api.PassThruOpen(None, c.byref(self.device)) == 0
        flags = 0x100 if protocol in (5, 6) else 0x1000 if protocol == 3 else 0
        assert self.api.PassThruConnect(self.device, protocol, flags, 500000, c.byref(self.channel)) == 0

    def flow(self, address=0x18DA11F1):
        mask, pattern, flow = message(), message(address=address + 1), message(address=address)
        for item in (mask, pattern, flow):
            item.DataSize = 4
        mask.Data[:4] = b"\xff" * 4
        identifier = U32()
        assert self.api.PassThruStartMsgFilter(self.channel, 3, c.byref(mask), c.byref(pattern),
                                             c.byref(flow), c.byref(identifier)) == 0
        return identifier.value

    def write(self, sent=None, timeout=1000):
        sent = sent or message()
        count = U32(1)
        status = self.api.PassThruWriteMsgs(self.channel, c.byref(sent), c.byref(count), timeout)
        return status, count.value

    def read(self, count=1, timeout=0):
        output = (Message * count)()
        received = U32(count)
        status = self.api.PassThruReadMsgs(self.channel, output, c.byref(received), timeout)
        return status, list(output)[:received.value]


class LegacyIOTests(unittest.TestCase):
    @contextmanager
    def session(self, protocol=6, customize=None):
        operations = []

        class Peer(mock_peer.LegacyPeer):
            def respond(self, request):
                operations.append(request.WhichOneof("command"))
                response = super().respond(request)
                if customize:
                    customize(self, request, response)
                return response

        with tempfile.TemporaryDirectory() as temporary, Server(Peer) as server:
            root = Path(temporary)
            for name in ("libopendiag.so", "libopendiag0404.so"):
                shutil.copyfile(BUILD / name, root / name)
            configure(root, server.port)
            session = Session(root / "libopendiag0404.so", protocol)
            try:
                yield session, operations
            finally:
                session.api.PassThruClose(session.device)

    def test_blocking_isotp_write_stops_at_completion_and_reads_buffered_indication(self):
        with self.session() as (session, operations):
            session.flow()
            operations.clear()
            self.assertEqual(session.write(), (0, 1))
            self.assertEqual(operations, ["queue", "read"])
            operations.clear()
            status, received = session.read()
            self.assertEqual(status, 0)
            self.assertEqual(received[0].RxStatus, 0x109)
            self.assertEqual(operations, [])
            status, received = session.read()
            self.assertEqual(status, 0)
            self.assertEqual(bytes(received[0].Data[:5]), bytes.fromhex("18da11f101"))
            self.assertEqual(operations, ["select", "read"])

    def test_single_endpoint_buses_do_not_probe_or_drain_past_requested_message(self):
        for protocol in (1, 2, 3, 5):
            with self.subTest(protocol=protocol), self.session(protocol) as (session, operations):
                operations.clear()
                self.assertEqual(session.write(message(protocol=protocol)), (0, 1))
                status, received = session.read()
                self.assertEqual((status, len(received)), (0, 1))
                self.assertEqual(operations, ["queue", "read", "read"])

    def test_read_combines_buffered_and_device_messages(self):
        with self.session() as (session, operations):
            session.flow()
            self.assertEqual(session.write(), (0, 1))
            operations.clear()
            status, received = session.read(count=2)
            self.assertEqual((status, len(received)), (0, 2))
            self.assertEqual([item.RxStatus for item in received], [0x109, 0x100])
            self.assertEqual(operations, ["select", "read"])

    def test_readable_isotp_endpoints_are_serviced_fairly(self):
        with self.session() as (session, operations):
            session.flow(0x18DA11F1)
            session.flow(0x18DA12F1)
            self.assertEqual(session.write(message(address=0x18DA11F1), timeout=0), (0, 1))
            self.assertEqual(session.write(message(address=0x18DA12F1), timeout=0), (0, 1))
            addresses = []
            for _ in range(4):
                status, received = session.read()
                self.assertEqual((status, len(received)), (0, 1))
                addresses.append(int.from_bytes(received[0].Data[:4], "big"))
            self.assertEqual(addresses, [0x18DA11F1, 0x18DA12F1, 0x18DA11F1, 0x18DA12F1])

    def test_replies_before_completion_remain_buffered(self):
        def reorder(peer, request, response):
            if request.WhichOneof("command") == "queue":
                peer.channels[request.queue.channel].rotate(1)
        with self.session(customize=reorder) as (session, operations):
            session.flow()
            self.assertEqual(session.write(), (0, 1))
            operations.clear()
            status, received = session.read(count=2)
            self.assertEqual((status, len(received)), (0, 2))
            self.assertEqual([item.RxStatus for item in received], [0x100, 0x109])
            self.assertEqual(operations, [])

    def test_async_transmit_failure_is_reported_and_does_not_poison_following_read(self):
        with self.session(5) as (session, operations):
            self.assertEqual(session.write(message(protocol=5, payload=b"\xe2"), timeout=0), (0, 1))
            self.assertEqual(session.read(), (7, []))
            self.assertEqual(session.write(message(protocol=5)), (0, 1))
            status, received = session.read()
            self.assertEqual((status, len(received)), (0, 1))

    def test_device_overflow_is_preserved_with_the_returned_message(self):
        def overflow(peer, request, response):
            if request.WhichOneof("command") == "queue":
                peer.channels[request.queue.channel][-1].rx_status |= 64
        with self.session(5, overflow) as (session, operations):
            self.assertEqual(session.write(message(protocol=5)), (0, 1))
            status, received = session.read()
            self.assertEqual((status, len(received)), (18, 1))
            self.assertEqual(received[0].RxStatus, 0x100)
            self.assertEqual(session.read(), (16, []))

    def test_bounded_host_queue_retains_overflow_until_read(self):
        def flood(peer, request, response):
            if request.WhichOneof("command") == "queue":
                queue = peer.channels[request.queue.channel]
                completion = queue.popleft()
                reply = queue.popleft()
                queue.extend([reply] * 270)
                queue.append(completion)
        with self.session(5, flood) as (session, operations):
            self.assertEqual(session.write(message(protocol=5)), (0, 1))
            status, received = session.read()
            self.assertEqual((status, len(received)), (18, 1))
            self.assertEqual(session.api.PassThruIoctl(session.channel, 8, None, None), 0)
            self.assertEqual(session.read(), (16, []))

    def test_timed_batch_checks_deadline_between_device_reads(self):
        def delayed_batch(peer, request, response):
            if request.WhichOneof("command") == "queue":
                queue = peer.channels[request.queue.channel]
                queue.extend([queue[-1]] * 100)
            elif request.WhichOneof("command") == "read":
                time.sleep(0.01)
        with self.session(5, delayed_batch) as (session, operations):
            self.assertEqual(session.write(message(protocol=5)), (0, 1))
            operations.clear()
            status, received = session.read(count=100, timeout=35)
            self.assertEqual(status, 9)
            self.assertGreater(len(received), 0)
            self.assertLessEqual(len(received), 4)
            self.assertLessEqual(operations.count("read"), 4)

    def test_stopping_filter_does_not_leave_stale_readiness_state(self):
        with self.session() as (session, operations):
            identifier = session.flow()
            self.assertEqual(session.write(), (0, 1))
            session.read(count=2)
            self.assertEqual(session.api.PassThruStopMsgFilter(session.channel, identifier), 0)
            session.flow(0x18DA12F1)
            self.assertEqual(session.write(message(address=0x18DA12F1)), (0, 1))
            status, received = session.read(count=2)
            self.assertEqual((status, len(received)), (0, 2))
            self.assertEqual(int.from_bytes(received[1].Data[:4], "big"), 0x18DA12F1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", type=Path, required=True)
    args, remaining = parser.parse_known_args()
    BUILD = args.build.resolve()
    unittest.main(argv=[__file__, *remaining])
