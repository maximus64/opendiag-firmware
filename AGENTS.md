# AGENTS.md

Guidance for AI coding agents working in this repository.

## What this is

ESP32-S3 firmware (ESP-IDF v6.1, C11, FreeRTOS) for the OpenDIAG OBD-II adapter, plus a host-side
J2534 pass-thru driver (`j2534_driver/`, C++11) that talks to the firmware over the same data link.
User docs are MkDocs sources in `docs/` (`docs/at_commands.md` tracks which ELM327 commands are
implemented; `docs/manual.md` documents shell commands and features, so update it alongside
user-visible behaviour changes). Licence is GPL-3.0-only; source files carry an SPDX header.

## Commands

Submodules are required for everything (nanopb lives in `components/nanopb/upstream`):

```bash
git submodule update --init --recursive
```

### Firmware (needs `. $IDF_PATH/export.sh`)

```bash
idf.py build
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM1 monitor     # console/shell is the SECOND CDC port; the first is the data link
idf.py format                      # clang-format main/ and test/host/ in place (tools/format.sh)
idf.py format-check                # fail without modifying
idf.py firmware-package            # release ZIP via tools/package_firmware.py
```

Reproducible release build exactly as CI does it (runs host tests first, `--verify` builds twice and
compares ZIPs; needs a full clone with tags):

```bash
mkdir -p build/docker && docker compose run --rm --build firmware [--verify]
```

If ESP-IDF rewrites `dependencies.lock` during reconfigure the release build fails; commit the
updated lock file.

### Host unit tests (no ESP-IDF needed)

```bash
cmake -S test/host -B test/host/build
cmake --build test/host/build
ctest --test-dir test/host/build --output-on-failure
ctest --test-dir test/host/build -R test_slcan          # one suite
test/host/build/test_slcan --list                         # list cases in a suite
test/host/build/test_slcan <substring>                    # run cases whose name contains substring
```

Options: `-DHW_LS_OBD_9=ON` builds for boards with the pin 9 low-side rework (`OPENDIAG_HW_LS_OBD_9`);
`-DCOVERAGE=ON` then `cmake --build test/host/build --target coverage` reports line coverage of
`main/` only (llvm-cov or gcov).

### J2534 driver (from repo root)

```bash
make -C j2534_driver -j4                 # native .so/.dylib (CONFIG=debug for -Og -g)
make -C j2534_driver test                # unit + Python contract tests; also builds test/host's
                                         # j2534_contract_firmware library and runs it as the peer
make -C j2534_driver test COVERAGE=1     # pass COVERAGE on the make command line, not via env CFLAGS
docker compose run --rm --build j2534-driver   # Windows i686 MinGW DLLs + config utility + ZIP
```

Requires GNU Make ≥ 4.3, Python 3 with `protobuf`, and `protoc` (`PYTHON=` to pick the
interpreter). The driver's local build script enforces ≥ 95% line coverage over `driver/`,
`common/`, `legacy/`, `platform/`, `config/`. Builds use `-Werror`. Windows builds target XP SP3
(`_WIN32_WINNT=0x0501`), checked by `tools/check_wincompat.py` against `tools/xp-imports.json`.

### Protocol / generated code

`protocol/j2534.proto` (+ `j2534.options`) is the firmware↔driver RPC schema. `main/j2534.pb.{c,h}`
and `protocol/j2534_pb2.py` are generated and committed — regenerate with `protocol/generate.sh`
(expects nanopb's generator importable from `PYTHON`, default `.venv/bin/python`). The driver
generates its own copy under its build dir. Generated `*.pb.*` files are excluded from clang-format.

Changing the wire format or RPC semantics means bumping `J2534_PROTOCOL_REVISION` in `main/j2534.h`;
the driver accepts only an exact match.

## Firmware architecture

Layering, top to bottom (`app_main` in `main/main.c` wires it up):

1. **Transports** — `usb_cdc.c` (two TinyUSB CDC ACM interfaces) and `ble_uart.c` (NimBLE UART
   characteristics + a control characteristic). CDC 1 is the stdio console/`shell.c`, not a
   comm port.
2. **`comm_iface`** — registers CDC0 and BLE as ports with non-blocking RX ring buffers
   (`comm_port_ops_t`). Overflow drops whole chunks rather than blocking.
3. **The link (`vif.c`, `link.h`)** — there is exactly *one* data link, bound to whichever transport
   connected first (`link usb|ble|drop` in the shell overrides). On client disconnect the link
   reverts to its default grammar and releases every claim, so the next client never inherits
   state. It runs on its own task, the only thread allowed inside the link's drivers.
4. **Front-ends (`vif_frontend_t`)** — the grammar spoken on the link: `elm327` (`elm327_at.c`,
   the default), `slcan` (`slcan.c`), `j2534` (`j2534.c` + `j2534_scheduler.c` + `j2534_wire.c`
   framing with magic `J253` and CRC). Each registers itself; `vif_link_set_frontend()` swaps them
   in the link task and releases all claims. ELM327 enters the others via `AT VIFPASSTHRU` /
   `AT VIFSLCAN`; the control plane (`vif_ctrl.c`: `ID`, `MODE`, `MODE=<name>`, `BUS`, `RESET`) is
   out-of-band only — shell `mode` command on USB, control characteristic on BLE — never inside
   the data stream.
5. **vif bus/pin arbitration** — callers use `vif_bus_open/send/recv/close(owner, bus, ...)` with two
   fixed owners, `VIF_OWNER_LINK` and `VIF_OWNER_SHELL`. Claims are per bus group (CAN, K-Line, and
   J1850 PWM/VPW sharing one transceiver). Pin claims cover the high-/low-side drivers and the boost
   converter (`VIF_PIN_VOLTAGE`/`VIF_PIN_GROUND`). `vif.c` owns `board.c` exclusively after
   `vif_init()`.
6. **Bus drivers (`bus_ops_t` in `bus.h`)** — `can_bus.c` (TWAI), `kline.c`, `j1850_pwm.c`,
   `j1850_vpw.c`. All exchange `bus_msg_t` (CAN id rides in the message; byte buses leave it 0).
   Pure encode/decode logic is split into `*_codec.c` files so it's host-testable; `bus_tx.c` /
   `bus_tx_worker.c` provide cancellable, sequenced transmit admission; `bus_notify()` wakes the
   claim-holding task on RX/TX/link-down events (front-ends sleep on events via `next_wake`
   rather than polling). `isotp.c` and `can_xfer.h` layer CAN frames/ISO-TP on top of vif.

## Host test architecture (`test/host/`)

- A standalone CMake project (not reachable from the top-level ESP-IDF build) that compiles
  firmware sources straight from `main/`. **Nothing in `main/` is `#ifdef`'d for the host**; the only
  seam is header shadowing: `stubs/` replaces ESP-IDF/FreeRTOS headers (with `fake_clock.h` for
  time), `driver_stubs/` adds real-ish RTOS/driver behaviour for driver suites, `fakes/` replaces
  whole firmware modules (board, bus, BLE, LED, ports).
- `framework/td_test.h` is an in-house Unity-compatible framework: `TEST(name)` auto-registers;
  optional `td_setup()`/`td_teardown()` per suite; a failed assertion aborts only that case.
- Each suite in `SUITES` gets a `<suite>_FW` list of firmware `.c` files to link. **Suites that
  `#include "foo.c"` to reach statics (e.g. `elm327_harness.h` includes `elm327_at.c`; driver
  suites include `kline.c`, `button.c`, `vif.c`, ...) must not also list that file**, or symbols
  are duplicated. New suites must be added to both `SUITES` and given a `_FW` list.
- `*_cov.c` suites are extra coverage-targeted cases reusing the stock suite's source list.

## J2534 driver architecture (`j2534_driver/`)

- `driver/` — native J2534 05.00 API (`opendiag32.dll` / `libopendiag.so|dylib`); `link.cpp`
  frames protobuf RPCs (same `J253` wire framing as the firmware) over a COM port or TCP socket
  (`transport` setting; the tests use TCP).
- `legacy/` — 04.04 shim (`odg40432.dll`) that loads the native DLL from beside itself and uses a
  private bridge; ship both from the same build.
- `platform/` — serial/filesystem per OS (`windows.cpp`, `posix.cpp` + `linux.cpp`/`darwin.cpp`).
- `config/` — Win32 configuration GUI (`opendiag_config.exe`) and COM-port classification.
- `tests/` — C/C++ ABI and API tests plus Python (`run_tests.py`) contract tests against loopback
  peers: `mock_peer.py`, and `firmware_peer.py`, which loads the real firmware J2534 core built from
  `test/host` (`j2534_contract_firmware`) behind a socket.
- Config is read from `OPENDIAG_DRIVER_CONFIG` or the per-OS system `opendiag.ini` (see
  `j2534_driver/README.md`); the file must exist.

## Conventions

- clang-format (LLVM base, 4-space indent, include order: own header, `<system>`, `"freertos/…"`,
  `"esp_…"`, project). Run `idf.py format` or `tools/format.sh` before committing.
- Commit subjects are prefixed with the area: `slcan: …`, `kline: …`, `docs/manual.md: …`,
  `test(j2534): …`.
- Comments in this codebase explain *why* (invariants, ownership, threading); keep that style when
  touching neighbouring code.
- CI (`.github/workflows/firmware.yml`) runs both Docker builds on PRs to `main`/`dev`; `v*.*.*`
  tags publish a release, and the firmware `manifest.json` version must match the tag.
