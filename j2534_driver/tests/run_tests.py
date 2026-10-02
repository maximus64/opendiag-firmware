#!/usr/bin/env python3
"""Build and run native host contract tests against isolated loopback peers."""
import argparse
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
    for api, mode, startup in cases:
        with tempfile.TemporaryDirectory(prefix="opendiag-host-") as temporary:
            directory = Path(temporary)
            native, legacy = install_libraries(args.native, args.legacy, directory)
            peer_type = peer.Peer if api == "0500" else peer.LegacyPeer
            with Server(peer_type, startup) as server:
                configure(directory, server.port, logging=1)
                library = native if api == "0500" else legacy
                print(f"RUN {api} {mode} startup={startup}", flush=True)
                result = subprocess.run([str(tests / f"api{api}"), str(library), mode],
                                        cwd="/", text=True, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, timeout=90)
                if result.returncode:
                    print(result.stdout)
                    raise SystemExit(result.returncode)
                for line in result.stdout.splitlines():
                    if line.startswith("PASS"):
                        print(line)
                logs = list(directory.glob("*.log"))
                if not logs or not any("END status=" in path.read_text() for path in logs):
                    raise RuntimeError("API logging produced no return records")
    test_ownership(args.native, args.legacy, peer)
    subprocess.run([sys.executable, str(project / "tests/test_legacy_io.py"),
                    "--native", str(args.native), "--legacy", str(args.legacy)],
                   check=True, timeout=90)
    print(f"PASS {len(cases)} native API scenarios and both C headers")


if __name__ == "__main__":
    main()
