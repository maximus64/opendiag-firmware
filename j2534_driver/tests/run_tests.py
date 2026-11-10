#!/usr/bin/env python3
"""Build and run native host contract tests against isolated loopback peers."""
import argparse
import concurrent.futures
import ctypes
import importlib
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest


def add_library_arguments(parser):
    """The Makefile names the built libraries; the tests never guess them."""
    parser.add_argument("--native", type=Path, required=True)
    parser.add_argument("--legacy", type=Path, required=True)


def install_libraries(native, legacy, directory):
    """Copy both libraries side by side, where the 04.04 shim looks for the native one."""
    for library in (native, legacy):
        shutil.copy2(library, directory / library.name)
    return directory / native.name, directory / legacy.name


class Server:
    def __init__(self, peer_type, startup="elm"):
        self.peer_type = peer_type
        self.startup = startup
        self.errors = []
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen()
        self.listener.settimeout(0.1)
        self.port = self.listener.getsockname()[1]
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.serve, daemon=True)

    def serve(self):
        while not self.stop.is_set():
            try:
                connection, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with connection:
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                peer = self.peer_type()
                peer.startup_mode = self.startup
                try:
                    peer.serve(connection)
                except (EOFError, ConnectionResetError, BrokenPipeError):
                    pass
                except Exception as error:
                    self.errors.append(error)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *unused):
        self.stop.set()
        self.listener.close()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError("Mock peer did not stop")
        if self.errors:
            raise RuntimeError(f"Mock peer failed: {self.errors}")


def configure(directory, port, logging=0):
    (directory / "opendiag.ini").write_text(
        "[device]\nName=OpenDIAG\nTransport=tcp\nHost=127.0.0.1\n"
        f"Port={port}\nRpcTimeout=1000\n[logging]\nEnabled={logging}\nMaxFileKB=64\n")
    # Keep in-process ctypes tests independent of the machine's installed settings.
    os.environ.update(test_environment(directory))


def test_environment(directory):
    return {"OPENDIAG_DRIVER_CONFIG": str(directory / "opendiag.ini"),
            "HOME": str(directory / "user"),
            "XDG_STATE_HOME": str(directory / "state")}


def flatten(suite):
    for item in suite:
        if isinstance(item, unittest.TestSuite):
            yield from flatten(item)
        else:
            yield item


class ShardLoader(unittest.TestLoader):
    """Keep every total'th case, so sibling shards cover the selection exactly once.

    Filtering here rather than in the argument list leaves unittest.main in charge of
    its own options: -v, -q, -f, -b, -c and -k all keep working, including -k, whose
    pattern the base loader has already applied by the time we slice.
    """

    def __init__(self, index, total):
        super().__init__()
        self.index = index
        self.total = total

    def select(self, suite):
        cases = sorted(flatten(suite), key=lambda case: case.id())
        return unittest.TestSuite(case for position, case in enumerate(cases)
                                  if position % self.total == self.index)

    def loadTestsFromModule(self, *args, **kwargs):
        return self.select(super().loadTestsFromModule(*args, **kwargs))

    def loadTestsFromNames(self, *args, **kwargs):
        return self.select(super().loadTestsFromNames(*args, **kwargs))


class CountingRunner:
    """Collect the case count without running anything, honouring -k and test names."""

    def __init__(self):
        self.count = 0

    def run(self, suite):
        self.count = suite.countTestCases()
        return unittest.TestResult()


def count_cases(module, argv):
    runner = CountingRunner()
    unittest.main(module=module, argv=argv, exit=False, testRunner=runner,
                  testLoader=ShardLoader(0, 1))
    return runner.count


def shard_count(module, argv, requested):
    if requested:
        return max(1, requested)
    # Each case spends ~350 ms idle in the frontend handshake, so oversubscribing CPUs
    # still pays; measured flat past ~2x. The cap bounds memory at roughly 30 MB a shard.
    return max(1, min(2 * (os.cpu_count() or 4), 32, count_cases(module, argv)))


def run_shard(module, argv, index, total, verbosity):
    program = unittest.main(module=module, argv=argv, exit=False, verbosity=verbosity,
                            testLoader=ShardLoader(index, total))
    result = program.result
    if total > 1:
        print(f"SHARD ran={result.testsRun} "
              f"bad={len(result.failures) + len(result.errors)}", flush=True)
    return 0 if result.wasSuccessful() else 1


def spawn_shards(script, base, names, jobs):
    """One process per shard: the firmware fixture keeps global state inside its .so."""
    running = []
    for index in range(jobs):
        command = [sys.executable, script, *base,
                   "--shard", str(index), "--shards", str(jobs), *names]
        running.append(subprocess.Popen(command, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True))
    total = bad = status = 0
    for index, process in enumerate(running):
        output = process.communicate()[0]
        for line in output.splitlines():
            if line.startswith("SHARD ran="):
                fields = dict(field.split("=") for field in line.split()[1:])
                total += int(fields["ran"])
                bad += int(fields["bad"])
        if process.returncode:
            status = status or process.returncode
            print(f"--- shard {index + 1}/{jobs} failed ---\n{output}")
    label = Path(script).stem
    print(f"{'OK' if not status else 'FAILED'} {label}: {total} tests over {jobs} shards"
          + (f", {bad} failed" if bad else ""), flush=True)
    return status


def sharded_main(script, base, extra, args, verbosity=1, module=None):
    """Serial when --jobs 1, a single shard when re-executed, else fan out.

    `extra` is whatever argparse left over: unittest's own options and test names,
    passed through untouched for unittest.main to parse.
    """
    module = module or sys.modules["__main__"]
    argv = [script, *extra]
    if args.shard is not None:
        return run_shard(module, argv, args.shard, args.shards, verbosity)
    if {"-f", "--failfast"} & set(extra):
        # Shards cannot stop one another, so honour the flag by not splitting at all.
        return run_shard(module, argv, 0, 1, verbosity)
    jobs = shard_count(module, argv, args.jobs)
    if jobs == 1:
        return run_shard(module, argv, 0, 1, verbosity)
    return spawn_shards(script, base, extra, jobs)


def add_shard_arguments(parser):
    parser.add_argument("--jobs", type=int, default=0, help="0 selects one shard per CPU")
    parser.add_argument("--shard", type=int, help=argparse.SUPPRESS)
    parser.add_argument("--shards", type=int, help=argparse.SUPPRESS)


def test_ownership(native_path, legacy_path, peer):
    entered = threading.Event()
    release = threading.Event()

    class BlockingPeer(peer.Peer):
        def respond(self, request):
            if request.WhichOneof("command") == "version":
                entered.set()
                if not release.wait(3):
                    raise RuntimeError("Concurrent API test did not release the response")
            return super().respond(request)

    with tempfile.TemporaryDirectory(prefix="opendiag-ownership-") as temporary:
        directory = Path(temporary)
        native_path, legacy_path = install_libraries(native_path, legacy_path, directory)
        native = ctypes.CDLL(str(native_path))
        legacy = ctypes.CDLL(str(legacy_path))
        for library in (native, legacy):
            library.PassThruOpen.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_uint32)]
            library.PassThruOpen.restype = ctypes.c_int32
            library.PassThruClose.argtypes = [ctypes.c_uint32]
            library.PassThruClose.restype = ctypes.c_int32
        native.PassThruReadVersion.argtypes = [ctypes.c_uint32, ctypes.c_char_p,
                                               ctypes.c_char_p, ctypes.c_char_p]
        native.PassThruReadVersion.restype = ctypes.c_int32
        identifier = ctypes.c_uint32()
        unused = ctypes.c_uint32(0x12345678)
        with Server(BlockingPeer) as server:
            configure(directory, server.port, logging=1)
            assert native.PassThruOpen(b"J2534-1:OpenDIAG", ctypes.byref(identifier)) == 0
            assert legacy.PassThruOpen(None, ctypes.byref(unused)) == 0x0e
            assert unused.value == 0x12345678
            assert native.PassThruClose(identifier) == 0
            assert legacy.PassThruOpen(None, ctypes.byref(identifier)) == 0
            assert native.PassThruOpen(b"J2534-1:OpenDIAG", ctypes.byref(unused)) == 0x0e
            assert legacy.PassThruClose(identifier) == 0
            assert native.PassThruOpen(b"J2534-1:OpenDIAG", ctypes.byref(identifier)) == 0
            buffers = [ctypes.create_string_buffer(80) for _ in range(3)]
            statuses = []
            worker = threading.Thread(target=lambda: statuses.append(
                native.PassThruReadVersion(identifier, *buffers)))
            worker.start()
            try:
                assert entered.wait(2), "Version call did not enter the transport"
                assert native.PassThruClose(identifier) == 0x26, "Overlapping API call was accepted"
            finally:
                release.set()
                worker.join(timeout=3)
                native.PassThruClose(identifier)
            assert not worker.is_alive() and statuses == [0]
    print("PASS cross-API ownership and deterministic concurrent-call rejection")


def test_config_override(native_path, legacy_path, peer):
    """Bad configs clear discovery results while open reports the reason."""
    class DeviceInfo(ctypes.Structure):
        _fields_ = [("DeviceName", ctypes.c_char * 80),
                    ("DeviceAvailable", ctypes.c_uint32),
                    ("DeviceDLLFWStatus", ctypes.c_uint32),
                    ("DeviceConnectMedia", ctypes.c_uint32),
                    ("DeviceConnectSpeed", ctypes.c_uint32),
                    ("DeviceSignalQuality", ctypes.c_uint32),
                    ("DeviceSignalStrength", ctypes.c_uint32)]

    with tempfile.TemporaryDirectory(prefix="opendiag-config-") as temporary:
        directory = Path(temporary)
        native_path, legacy_path = install_libraries(native_path, legacy_path, directory)
        native = ctypes.CDLL(str(native_path))
        legacy = ctypes.CDLL(str(legacy_path))
        for library in (native, legacy):
            library.PassThruOpen.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_uint32)]
            library.PassThruOpen.restype = ctypes.c_int32
            library.PassThruGetLastError.argtypes = [ctypes.c_char_p]
            library.PassThruGetLastError.restype = ctypes.c_int32
        native.PassThruScanForDevices.argtypes = [ctypes.POINTER(ctypes.c_uint32)]
        native.PassThruScanForDevices.restype = ctypes.c_int32
        native.PassThruGetNextDevice.argtypes = [ctypes.POINTER(DeviceInfo)]
        native.PassThruGetNextDevice.restype = ctypes.c_int32

        def scan(expected):
            count = ctypes.c_uint32(0x12345678)
            status = native.PassThruScanForDevices(ctypes.byref(count))
            assert status == 0, f"Scan returned {status:#x}, expected STATUS_NOERROR"
            assert count.value == expected, f"Scan found {count.value}, expected {expected}"

        invalid = directory / "invalid.ini"
        invalid.write_text("[device]\nTransport=invalid\n")
        cases = [(directory / "missing.ini", "OPENDIAG_DRIVER_CONFIG file not found:"),
                 (directory, "OPENDIAG_DRIVER_CONFIG file not found:"),
                 (invalid, "invalid opendiag.ini settings")]
        # Native open reports ERR_OPEN_FAILED, translated to
        # ERR_DEVICE_NOT_CONNECTED by the 04.04 shim.
        operations = [(native, native.PassThruOpen, [b"J2534-1:OpenDIAG"], 0x22),
                      (legacy, legacy.PassThruOpen, [None], 0x08)]
        with Server(peer.Peer) as server:
            configure(directory, server.port, logging=1)
            for path, reason in cases:
                # Clear both an unread result and an already consumed result.
                for consumed in (False, True):
                    os.environ["OPENDIAG_DRIVER_CONFIG"] = str(directory / "opendiag.ini")
                    scan(1)
                    info = DeviceInfo()
                    if consumed:
                        assert native.PassThruGetNextDevice(ctypes.byref(info)) == 0
                        assert info.DeviceName == b"OpenDIAG"
                    os.environ["OPENDIAG_DRIVER_CONFIG"] = str(path)
                    scan(0)
                    ctypes.memset(ctypes.byref(info), 0xa5, ctypes.sizeof(info))
                    unchanged = bytes(info)
                    assert native.PassThruGetNextDevice(ctypes.byref(info)) == 0x10
                    assert bytes(info) == unchanged

                for library, operation, arguments, expected in operations:
                    unchanged = ctypes.c_uint32(0x12345678)
                    actual = operation(*arguments, ctypes.byref(unchanged))
                    assert actual == expected, f"{operation.__name__}: got {actual:#x}, expected {expected:#x}"
                    assert unchanged.value == 0x12345678, operation.__name__
                    text = ctypes.create_string_buffer(80)
                    assert library.PassThruGetLastError(text) == 0
                    reported = text.value.decode()
                    assert reason in reported, reported

            os.environ["OPENDIAG_DRIVER_CONFIG"] = str(directory / "opendiag.ini")
            scan(1)
            assert native.PassThruScanForDevices(None) == 0x04
            info = DeviceInfo()
            assert native.PassThruGetNextDevice(ctypes.byref(info)) == 0
            assert info.DeviceName == b"OpenDIAG"
    print("PASS config errors clear scan results and remain available through both open APIs")


def test_contract_mismatch(native_path, legacy_path, peer):
    """A protocol mismatch must name the offending field instead of failing blind."""
    cases = [({"wire_version": 2}, "wire 2, driver needs 3"),
             ({"api_version": 0x404}, "API 0404, driver needs 0500"),
             ({"fragment_bytes": 32}, "fragment 32, driver needs 64 to 4608"),
             ({"rpc_bytes": 2048}, "RPC 2048, driver needs 4608"),
             (None, "firmware reported no capabilities")]

    with tempfile.TemporaryDirectory(prefix="opendiag-contract-") as temporary:
        directory = Path(temporary)
        native_path, legacy_path = install_libraries(native_path, legacy_path, directory)
        native = ctypes.CDLL(str(native_path))
        legacy = ctypes.CDLL(str(legacy_path))
        for library in (native, legacy):
            library.PassThruOpen.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_uint32)]
            library.PassThruOpen.restype = ctypes.c_int32
            library.PassThruGetLastError.argtypes = [ctypes.c_char_p]
            library.PassThruGetLastError.restype = ctypes.c_int32

        for overrides, expected in cases:
            class MismatchedPeer(peer.Peer):
                def respond(self, request, fields=overrides):
                    response = super().respond(request)
                    if request.WhichOneof("command") == "capabilities":
                        if fields is None:
                            response.ClearField("capabilities")
                        else:
                            for key, value in fields.items():
                                setattr(response.capabilities, key, value)
                    return response

            identifier = ctypes.c_uint32()
            text = ctypes.create_string_buffer(80)
            with Server(MismatchedPeer) as server:
                configure(directory, server.port, logging=1)
                assert native.PassThruOpen(b"J2534-1:OpenDIAG",
                                           ctypes.byref(identifier)) == 0x22, expected
                native.PassThruGetLastError(text)
                reported = text.value.decode()
                assert expected in reported, f"05.00 lost {expected!r}: {reported!r}"
                assert "update one of them" in reported, reported
                # 04.04 reports ERR_DEVICE_NOT_CONNECTED but must forward the reason.
                assert legacy.PassThruOpen(None, ctypes.byref(identifier)) == 0x08, expected
                legacy.PassThruGetLastError(text)
                reported = text.value.decode()
                assert expected in reported, f"04.04 lost {expected!r}: {reported!r}"
    print(f"PASS {len(cases)} protocol-mismatch reports reach both APIs")


def test_device_side_waits(native_path, legacy_path, peer):
    """A timed read or select is one request the device holds, never a polling loop."""
    class Packed(ctypes.Structure):
        _pack_ = 1
        _layout_ = "ms"

    class Message(Packed):
        _fields_ = [(name, ctypes.c_uint32) for name in (
            "ProtocolID", "MsgHandle", "RxStatus", "TxFlags", "Timestamp", "DataLength",
            "ExtraDataIndex")] + [("DataBuffer", ctypes.POINTER(ctypes.c_ubyte)),
                                  ("DataBufferSize", ctypes.c_uint32)]

    class Resource(Packed):
        _fields_ = [("Connector", ctypes.c_uint32), ("NumOfResources", ctypes.c_uint32),
                    ("ResourceListPtr", ctypes.POINTER(ctypes.c_uint32))]

    class ChannelSet(Packed):
        _fields_ = [("ChannelCount", ctypes.c_uint32), ("ChannelThreshold", ctypes.c_uint32),
                    ("ChannelList", ctypes.POINTER(ctypes.c_uint32))]

    waits = []

    class RecordingPeer(peer.Peer):
        def respond(self, request):
            name = request.WhichOneof("command")
            if name == "read":
                waits.append(("read", request.read.timeout_ms))
            elif name == "select":
                waits.append(("select", request.select.threshold, request.select.timeout_ms))
            return super().respond(request)

    def messages(count):
        storage = [(ctypes.c_ubyte * 64)() for _ in range(count)]
        array = (Message * count)()
        for item, buffer in zip(array, storage):
            item.DataBuffer = buffer
            item.DataBufferSize = 64
        return array, storage

    def timed(call):
        start = time.monotonic()
        status = call()
        return status, (time.monotonic() - start) * 1000

    with tempfile.TemporaryDirectory(prefix="opendiag-waits-") as temporary:
        directory = Path(temporary)
        native_path, _ = install_libraries(native_path, legacy_path, directory)
        api = ctypes.CDLL(str(native_path))
        with Server(RecordingPeer) as server:
            configure(directory, server.port)
            device, channel = ctypes.c_uint32(), ctypes.c_uint32()
            assert api.PassThruOpen(b"J2534-1:OpenDIAG", ctypes.byref(device)) == 0
            pins = (ctypes.c_uint32 * 2)(6, 14)
            assert api.PassThruConnect(device, 5, 0, 500000, Resource(1, 2, pins),
                                       ctypes.byref(channel)) == 0

            output, keep = messages(2)
            count = ctypes.c_uint32(1)
            status, waited = timed(lambda: api.PassThruReadMsgs(channel, output,
                                                                ctypes.byref(count), 60))
            assert (status, count.value) == (0x10, 0), (status, count.value)
            assert waited >= 55, waited
            assert waits == [("read", 60)], waits

            # One queued message is returned at once; the second read waits out the rest.
            del waits[:]
            data = (ctypes.c_ubyte * 5)(0, 0, 7, 0xe0, 0x3e)
            sent = Message(ProtocolID=5, DataLength=5, ExtraDataIndex=5, DataBufferSize=5)
            sent.DataBuffer = ctypes.cast(data, ctypes.POINTER(ctypes.c_ubyte))
            queued = ctypes.c_uint32(1)
            assert api.PassThruQueueMsgs(channel, ctypes.byref(sent), ctypes.byref(queued)) == 0
            count = ctypes.c_uint32(2)
            status, waited = timed(lambda: api.PassThruReadMsgs(channel, output,
                                                                ctypes.byref(count), 60))
            assert (status, count.value) == (0x09, 1), (status, count.value)
            assert [kind for kind, _ in waits] == ["read", "read"], waits
            assert waits[0][1] == 60 and 0 < waits[1][1] <= 60, waits

            del waits[:]
            listed = (ctypes.c_uint32 * 1)(channel.value)
            selection = ChannelSet(1, 1, listed)
            status, waited = timed(lambda: api.PassThruSelect(ctypes.byref(selection), 1, 60))
            assert status == 0x10, status
            assert waited >= 55, waited
            assert waits == [("select", 1, 60)], waits
            assert api.PassThruClose(device) == 0
            del keep
    print("PASS timed reads and selects wait on the device in one request each")


def test_fragment_size(native_path, legacy_path, peer):
    """The driver sends and accepts fragments as large as the firmware advertises."""
    class Packed(ctypes.Structure):
        _pack_ = 1
        _layout_ = "ms"

    class Message(Packed):
        _fields_ = [(name, ctypes.c_uint32) for name in (
            "ProtocolID", "MsgHandle", "RxStatus", "TxFlags", "Timestamp", "DataLength",
            "ExtraDataIndex")] + [("DataBuffer", ctypes.POINTER(ctypes.c_ubyte)),
                                  ("DataBufferSize", ctypes.c_uint32)]

    class Resource(Packed):
        _fields_ = [("Connector", ctypes.c_uint32), ("NumOfResources", ctypes.c_uint32),
                    ("ResourceListPtr", ctypes.POINTER(ctypes.c_uint32))]

    peers = []

    class LargeFragmentPeer(peer.Peer):
        def __init__(self):
            super().__init__()
            self.fragment_bytes = 1024
            peers.append(self)

    queue_op = peer.pb.Request.DESCRIPTOR.fields_by_name["queue"].number - 1
    read_op = peer.pb.Request.DESCRIPTOR.fields_by_name["read"].number - 1
    with tempfile.TemporaryDirectory(prefix="opendiag-fragment-") as temporary:
        directory = Path(temporary)
        native_path, _ = install_libraries(native_path, legacy_path, directory)
        api = ctypes.CDLL(str(native_path))
        with Server(LargeFragmentPeer) as server:
            configure(directory, server.port)
            device, channel = ctypes.c_uint32(), ctypes.c_uint32()
            assert api.PassThruOpen(b"J2534-1:OpenDIAG", ctypes.byref(device)) == 0
            pins = (ctypes.c_uint32 * 2)(6, 14)
            assert api.PassThruConnect(device, 5, 0, 500000, Resource(1, 2, pins),
                                       ctypes.byref(channel)) == 0
            # A 258-byte flash block behind its CAN identifier.
            data = (ctypes.c_ubyte * 262)(*(i & 0xff for i in range(262)))
            sent = Message(ProtocolID=5, DataLength=262, ExtraDataIndex=262, DataBufferSize=262)
            sent.DataBuffer = ctypes.cast(data, ctypes.POINTER(ctypes.c_ubyte))
            count = ctypes.c_uint32(1)
            assert api.PassThruQueueMsgs(channel, ctypes.byref(sent), ctypes.byref(count)) == 0
            buffer = (ctypes.c_ubyte * 300)()
            received = Message(DataBufferSize=300)
            received.DataBuffer = ctypes.cast(buffer, ctypes.POINTER(ctypes.c_ubyte))
            count = ctypes.c_uint32(1)
            assert api.PassThruReadMsgs(channel, ctypes.byref(received),
                                        ctypes.byref(count), 0) == 0
            assert received.DataLength == 262 and bytes(buffer[:262]) == bytes(data)
            assert api.PassThruClose(device) == 0
    frames = [frame for frame in peers[-1].frames if frame[0] in (queue_op, read_op)]
    assert [(op, flags) for op, flags, _ in frames] == [(queue_op, 0), (read_op, 0)], frames
    assert frames[0][2] > 262, frames
    print("PASS a flash block crosses the wire as one frame each way")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    add_library_arguments(parser)
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    subprocess.run([sys.executable, str(project / "tests/test_wincompat.py")], check=True)
    build = args.build.resolve()
    tests = build / "tests"
    generated = tests / "protocol"
    generated.mkdir(parents=True, exist_ok=True)
    protocol = project.parent / "protocol"
    try:
        import google.protobuf
    except ImportError as error:
        raise SystemExit("Install Python protobuf, or set PYTHON to the repository .venv/bin/python") from error
    subprocess.run(["protoc", f"-I{protocol}", f"--python_out={generated}",
                    str(protocol / "j2534.proto")], check=True)
    os.environ["OPENDIAG_TEST_PROTO_DIR"] = str(generated)
    peer = importlib.import_module("mock_peer")
    configure(tests, 1, logging=1)
    # The Makefile builds the test programs and both ABI headers' objects.
    subprocess.run([str(tests / "platform_test")], check=True, timeout=15)
    cases = [("0500", "selftest", "elm"), ("0500", "contract", "elm"),
             ("0404", "selftest", "elm"), ("0404", "contract", "elm"),
             ("0500", "selftest", "drop-first")]
    # Each case owns a temporary directory, an ephemeral port and its own endpoint lease,
    # so they only ever waited on each other's startup handshakes. Run them together.
    def run_case(case):
        api, mode, startup = case
        with tempfile.TemporaryDirectory(prefix="opendiag-host-") as temporary:
            directory = Path(temporary)
            native, legacy = install_libraries(args.native, args.legacy, directory)
            peer_type = peer.Peer if api == "0500" else peer.LegacyPeer
            with Server(peer_type, startup) as server:
                configure(directory, server.port, logging=1)
                library = native if api == "0500" else legacy
                result = subprocess.run([str(tests / f"api{api}"), str(library), mode],
                                        cwd="/", text=True, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, timeout=180,
                                        env={**os.environ, **test_environment(directory)})
                logs = list(directory.rglob("*.log"))
                logged = bool(logs) and any("END status=" in path.read_text() for path in logs)
        return result.returncode, result.stdout, logged

    def run_legacy():
        return subprocess.run([sys.executable, str(project / "tests/test_legacy_io.py"),
                               "--native", str(args.native), "--legacy", str(args.legacy)],
                              text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              timeout=300)

    with concurrent.futures.ThreadPoolExecutor(max_workers=len(cases) + 1) as pool:
        pending_legacy = pool.submit(run_legacy)
        outcomes = list(pool.map(run_case, cases))
        legacy_result = pending_legacy.result()

    for (api, mode, startup), (code, output, logged) in zip(cases, outcomes):
        print(f"RUN {api} {mode} startup={startup}", flush=True)
        if code:
            print(output)
            raise SystemExit(code)
        for line in output.splitlines():
            if line.startswith("PASS"):
                print(line)
        if not logged:
            raise RuntimeError("API logging produced no return records")
    test_ownership(args.native, args.legacy, peer)
    test_config_override(args.native, args.legacy, peer)
    test_contract_mismatch(args.native, args.legacy, peer)
    test_device_side_waits(args.native, args.legacy, peer)
    test_fragment_size(args.native, args.legacy, peer)
    if legacy_result.returncode:
        print(legacy_result.stdout)
        raise SystemExit(legacy_result.returncode)
    for line in legacy_result.stdout.splitlines():
        if line.startswith(("OK ", "FAILED ")):
            print(line)
    print(f"PASS {len(cases)} native API scenarios and both C headers")


if __name__ == "__main__":
    main()
