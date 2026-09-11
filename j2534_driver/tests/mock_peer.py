#!/usr/bin/env python3
"""Deterministic wire peer for XP DLL tests. Never connects to a vehicle."""

import argparse
from collections import deque
import json
import os
from pathlib import Path
import socket
import struct
import sys
import time
import zlib

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, os.environ.get("OPENDIAG_TEST_PROTO_DIR", str(REPO / "protocol")))
import j2534_pb2 as pb

HEADER = struct.Struct("<4sBBHIHH")


def exact(connection, size):
    result = bytearray()
    while len(result) < size:
        chunk = connection.recv(size - len(result))
        if not chunk:
            raise EOFError
        result.extend(chunk)
    return bytes(result)


def packet(op, sequence, flags, offset, data):
    header = HEADER.pack(b"J253", 1, op, flags, sequence, offset, len(data))
    return header + data + struct.pack("<I", zlib.crc32(header[4:] + data))


class Peer:
    def __init__(self):
        self.queue = deque()
        self.identifier = 10
        self.requests = 0
        self.fault = 0
        self.select_fault = 0

    def respond(self, request):
        self.requests += 1
        name = request.WhichOneof("command")
        response = pb.Response()
        if name == "capabilities":
            response.capabilities.CopyFrom(pb.Capabilities(
                wire_version=1, api_version=0x500, fragment_bytes=192, rpc_bytes=4608))
        elif name == "open":
            self.queue.clear()
            response.id = 1
        elif name in ("connect", "logical_connect", "start_filter", "start_periodic"):
            if name == "connect" and request.connect.protocol in (3, 4):
                connect = request.connect
                expected_pins = [7] if connect.flags & 0x1000 else [7, 15]
                print(json.dumps({"protocol": connect.protocol, "flags": connect.flags,
                                  "pins": list(connect.pins)}), flush=True)
                if connect.flags & ~0x1200:
                    response.status = 6
                    return response
                if list(connect.pins) != expected_pins:
                    response.status = 19
                    return response
            self.identifier += 1
            response.id = self.identifier
        elif name == "queue":
            message = request.queue.message
            if message.handle == 0xf001:
                response.status = 17
            else:
                saved = pb.Message()
                saved.CopyFrom(message)
                saved.timestamp_us = 0xfffffffe
                saved.extra_data_index = len(message.data)
                if message.handle == 0xf002:
                    saved.rx_status = 64
                self.queue.append(saved)
                response.count = 1
        elif name == "read":
            if self.queue:
                response.message.CopyFrom(self.queue.popleft())
                response.count = 1
            else:
                response.status = 16
        elif name == "select":
            channels = request.select.channels
            if self.select_fault == 0xfe01:
                response.channels.extend([channels[0], channels[0]])
            elif self.select_fault == 0xfe02:
                response.channels.extend([channels[-1], channels[-1]])
            elif self.select_fault == 0xfe03:
                response.channels.extend([channels[-1], 0xffffffff])
            elif self.select_fault == 0xfe04:
                response.channels.extend(reversed(channels))
            elif self.queue:
                response.channels.extend(request.select.channels[:1])
        elif name == "version":
            response.text = "OpenDIAG deterministic test peer"
        elif name == "voltage" and request.voltage.pin == 16:
            response.status = 0x13
        elif name == "ioctl":
            io = request.ioctl
            if io.id in (1, 2):
                for value in io.config:
                    if value.parameter == 0xffff:
                        response.status = 0x1e
                        break
                    if 0xff01 <= value.parameter <= 0xff07:
                        self.fault = value.parameter
                    if 0xfe01 <= value.parameter <= 0xfe04:
                        self.select_fault = value.parameter
                    response.config.add(parameter=value.parameter, value=500000)
            elif io.id == 8:
                self.queue.clear()
            elif io.id in (3, 14):
                response.millivolts = 12000
        return response

    def serve(self, connection):
        assembled = bytearray()
        outgoing = b""
        response_offset = 0
        connection.settimeout(60)
        while True:
            prefix = exact(connection, 4)
            if prefix == b"AT V":
                assert prefix + exact(connection, 12) == b"AT VIF PASSTHRU\r"
                print("Startup: AT VIF PASSTHRU", flush=True)
                if getattr(self, "startup_mode", "elm") == "elm":
                    connection.sendall(b"AT VIF PASSTHRU\rOK\r\r>")
                prefix = exact(connection, 4)
            header = prefix + exact(connection, 12)
            magic, version, op, flags, sequence, offset, size = HEADER.unpack(header)
            data = exact(connection, size)
            checksum = struct.unpack("<I", exact(connection, 4))[0]
            assert magic == b"J253" and version == 1 and size <= 192
            assert checksum == zlib.crc32(header[4:] + data)
            drop_startup = getattr(self, "startup_mode", "elm") == "drop-first"
            if drop_startup and not getattr(self, "startup_dropped", False):
                self.startup_dropped = True
                print("Startup: discarded first binary request", flush=True)
                continue
            if flags == 2:
                assert offset == response_offset
            else:
                if offset == 0:
                    assembled.clear()
                assert offset == len(assembled)
                assembled.extend(data)
                if flags & 1:
                    connection.sendall(packet(op, sequence, 132, len(assembled), b""))
                    continue
                request = pb.Request.FromString(assembled)
                field = request.DESCRIPTOR.fields_by_name[request.WhichOneof("command")]
                assert field.number == op + 1
                response = self.respond(request)
                if self.fault == 0xff07:
                    response.status = 27
                outgoing = response.SerializeToString()
                response_offset = 0
            fragment = outgoing[response_offset:response_offset + 192]
            more = response_offset + len(fragment) < len(outgoing)
            frame = packet(op, sequence, 129 if more else 128, response_offset, fragment)
            response_offset += len(fragment)
            fault = self.fault
            self.fault = 0
            if fault:
                print(json.dumps({"fault": hex(fault), "requests": self.requests}), flush=True)
            if fault == 0xff01:
                frame = frame[:-1] + bytes([frame[-1] ^ 1])
            elif fault == 0xff02:
                frame = packet(op, sequence + 1, 128, 0, fragment)
            elif fault == 0xff03:
                return
            elif fault == 0xff04:
                frame = packet(op, sequence, 128, 0, bytes(193))
            elif fault == 0xff05:
                continue
            # Deliberately split headers and payloads to exercise exact-read loops.
            connection.sendall(frame[:3])
            connection.sendall(frame[3:11])
            connection.sendall(frame[11:])
            if fault == 0xff06:
                return



class LegacyPeer(Peer):
    """Supply completion indications and per-channel queues for the legacy adapter."""

    def __init__(self):
        super().__init__()
        self.channels = {}

    def respond(self, request):
        name = request.WhichOneof("command")
        if name == "open":
            self.channels.clear()
        if name == "queue":
            message = request.queue.message
            response = pb.Response()
            marker = message.data[4] if len(message.data) > 4 else 0
            if marker == 0xe3:
                response.status = 17
                return response
            response.count = 1
            queue = self.channels.setdefault(request.queue.channel, deque())
            if marker != 0xe1:
                queue.append(pb.Message(
                    protocol=message.protocol,
                    handle=message.handle,
                    rx_status=512 if marker == 0xe2 else 8,
                    timestamp_us=123456,
                ))
            if marker not in (0xe1, 0xe2):
                echoed = pb.Message()
                echoed.CopyFrom(message)
                echoed.handle = 0
                echoed.rx_status = message.tx_flags & 0x180
                echoed.timestamp_us = 123457
                echoed.extra_data_index = len(message.data)
                queue.append(echoed)
            return response
        if name == "select":
            response = pb.Response()
            response.channels.extend(channel for channel in request.select.channels
                                     if self.channels.get(channel))
            return response
        if name == "read":
            response = pb.Response()
            queue = self.channels.setdefault(request.read.id, deque())
            if queue:
                response.message.CopyFrom(queue.popleft())
                response.count = 1
            else:
                response.status = 16
            return response
        if name == "ioctl" and request.ioctl.id == 8:
            self.channels.setdefault(request.ioctl.target, deque()).clear()
        return super().respond(request)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.56.1")
    parser.add_argument("--port", type=int, default=23210)
    parser.add_argument("--legacy", action="store_true")
    parser.add_argument("--startup-mode", choices=("elm", "binary", "drop-first"), default="elm")
    args = parser.parse_args()
    peer = LegacyPeer() if args.legacy else Peer()
    peer.startup_mode = args.startup_mode
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((args.host, args.port))
        listener.listen(4)
        print(f"Mock peer listening on {args.host}:{args.port}", flush=True)
        while True:
            connection, _ = listener.accept()
            with connection:
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                try:
                    peer.serve(connection)
                except (EOFError, ConnectionError, socket.timeout):
                    pass


if __name__ == "__main__":
    main()
