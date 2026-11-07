# Developer guide

This page is for anyone who wants to change the firmware. It covers how it's put together, how to
build and test it, how to add a command, and how to send a change.

---

## Repository layout

| Path | What it holds |
| --- | --- |
| `main/` | The firmware (ESP-IDF component) |
| `test/host/` | Host unit tests: the firmware sources built for Linux against fakes |
| `j2534_driver/` | The Windows J2534 DLLs and their own tests ([README](https://github.com/maximus64/opendiag-firmware/tree/main/j2534_driver)) |
| `protocol/` | The protobuf schema between the J2534 DLL and the firmware, and its generator |
| `docs/` | This site (MkDocs). `mkdocs.yml` is the navigation |
| `tools/` | Docker build script, firmware packaging, `format.sh`, `serial_term.py` |
| `components/` | Pinned third-party code (submodules) |

---

## How the firmware is put together

```text
 USB CDC0 / BLE / WiFi        USB CDC1                BLE control
        │                        │                         │
   comm_iface ── link ──┐      shell ──── vif_ctrl ────────┘
                        │        │
                  front-end (one at a time on the link)
                  elm327_at  ·  slcan  ·  j2534
                        │        │
                        vif (claims, owners, pins)
                        │
         can_bus · kline · j1850_pwm / j1850_vpw      board (pins, boost, sense)
```

- **Transports** (`usb_cdc.c`, `ble_uart.c`, `wifi_link.c`) register with `comm_iface`. **`link`** decides
  which one carries the single data link. The first to connect wins, and `link usb|ble|drop`
  overrides that.
- **Front-ends** are the grammars the link speaks. Each one is a `vif_frontend_t` (`start`,
  `feed`, `poll`, `stop`) and registers itself at boot (`elm327_register()`, `slcan_register()`,
  `j2534_register()`). Only one runs at a time. Switching releases everything the link held.
- **`vif`** owns the hardware. A bus or a pin is *claimed* by an owner: `VIF_OWNER_LINK` (the
  front-end's task) or `VIF_OWNER_SHELL` (the debug console). Only the owner's own task may use
  or release it. This is what stops the shell tearing down a bus mid-exchange.
- **Bus drivers** (`can_bus.c`, `kline.c`, `j1850_*.c`) sit behind `bus.h`: `open` with a
  `bus_cfg_t`, `send`/`recv` of `bus_msg_t`, and `set_param`/`ioctl`. `can_xfer.h` turns
  messages into CAN frames for the front-ends.
- **`isotp.c`** is ISO 15765-2, shared by the ELM327 and J2534 front-ends. Behaviour differences
  are flags in `isotp_config_t`, not separate copies.
- **Power** (`power.c`, decisions in `power_policy.c`) puts the adapter into deep sleep when the
  vehicle is parked and nobody uses it. It counts CAN edges with the pulse counter, beside the
  TWAI controller, and holds every output at its idle level while asleep.
- **Control plane** (`vif_ctrl.c`): `ID`, `MODE`, `BUS`, `RESET`. It is reachable from the shell
  and over BLE, and by design carries nothing that drives pins.

### Rules that keep a vehicle safe

Keep to these in any change:

- **Never drive a CAN bus you haven't checked.** The ELM327 front-end opens CAN with
  `listen_first`, so a bus at another bit rate is refused instead of flooded. New CAN open paths
  must do the same, or be listen-only.
- **The AUX CAN port is always listen-only.** Don't add a way around it.
- **Nothing that energises a pin goes on the control plane or BLE.** Pin control stays in the
  shell (`hsset`, `lsset`) and in front-ends whose clients ask for it explicitly (J2534,
  `AT PROGV`).
- **Release on disconnect.** Whatever a link client claimed must be released when it goes.
  `vif` does this if you claim through it.
- **No new defaults that transmit.** A command that sends something the user didn't ask for (a
  probe, a keep-alive) must be off by default.

---

## Building

With Docker, exactly as CI does it:

```bash
git submodule update --init --recursive
mkdir -p build/docker
docker compose run --rm --build firmware          # tests + firmware ZIP in build/docker
docker compose run --rm --build firmware --verify # also checks the build is reproducible
```

With ESP-IDF v6.1 installed:

```bash
git submodule update --init --recursive
. $IDF_PATH/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM1 monitor   # the console is the second port
```

The J2534 driver builds separately: `docker compose run --rm --build j2534-driver`, or see
its README.

---

## Host tests

```bash
cmake -S test/host -B test/host/build
cmake --build test/host/build
ctest --test-dir test/host/build --output-on-failure
ctest --test-dir test/host/build -R elm327_can     # one suite
```

Without a local compiler, use the image that `docker compose build firmware` creates. It is
named `opendiag-firmware-firmware` when the checkout folder is `opendiag-firmware`:

```bash
docker run --rm -v "$PWD":/project -w /project --entrypoint /opt/esp/entrypoint.sh \
  opendiag-firmware-firmware bash -c \
  "cmake -S test/host -B /tmp/hb && cmake --build /tmp/hb -j8 && ctest --test-dir /tmp/hb"
```

How they work:

- The sources in `main/` are compiled unchanged. `test/host/stubs/` shadows the ESP-IDF headers,
  and `test/host/fakes/` replaces the drivers: `fake_can_bus` records every open, frame and
  `bus_cfg_t`, and lets a test queue replies.
- The ELM327 suites include `elm327_harness.h`, which `#include`s `elm327_at.c`, so a test can
  reach its statics. A command travels the production path: port ring buffer → `feed` →
  parser → real `vif.c` → fakes.
- Each suite is its own executable, listed in `SUITES` in `test/host/CMakeLists.txt` with the
  firmware sources it links (`<suite>_FW`). A suite that includes a `.c` must not link it too.
- Helpers: `elm_ok("ATxx\r")` asserts an `OK`, `elm_ask("...")` returns the reply, and
  `elm_echo_off()` turns echo off. A test is `TEST(name) { ... }` with `TEST_ASSERT_*`
  macros (`test/host/framework/td_test.h`). It registers itself; there is no list to update.
- Coverage: `test/host/coverage.sh`.

Every change to `main/` should come with a test that fails without it.

---

## Adding an ELM327 AT command

Use `AT CS` (`main/elm327_at.c`) as the model:

1. **State.** If the command sets something, add a field to the settings struct and give it a
   default in `elm327_load_defaults()`. `AT D` and `AT Z` call it. A value that must survive
   `AT Z` lives in a file-scope static instead, as `SD` and `PP` do.
2. **Parse.** Add a branch to `elm327_at_extra_command()`. The command arrives upper-cased with
   spaces removed (`"CRA7C0"`). Return `true` once handled, whether it was valid or not, and
   answer `OK\r`, the value, or `?\r` for bad arguments.
3. **Behaviour.** Apply the setting where the bus work happens: `elm327_can_protocol_xfer()` for
   CAN, the K-Line and J1850 paths for the byte buses.
4. **Test.** Add a `TEST()` to the matching suite (`test_elm327_at.c`, `test_elm327_can.c`,
   `test_elm327_bytebus.c`). Check the reply text and what reached the fake bus.
5. **Document.** Tick the row in `docs/at_commands.md`, with a short note of any limit. If users
   will reach for it, also add it to the right section of `docs/manual.md`.

Answer `?` for anything not implemented, rather than `OK` with no effect. Apps use the answer to
decide what the adapter can do.

## Adding a shell command

Add a handler and a row to the command table at the end of `main/shell.c`:
`{"name", handler, "help text"}`. Hardware commands claim through `vif` as `VIF_OWNER_SHELL`.
Document the command in the shell tables of `docs/manual.md`.

---

## Formatting

```bash
tools/format.sh            # reformat main/ and test/host/
tools/format.sh --check    # report, change nothing
```

The style is `.clang-format`. Different clang-format releases wrap a few lines differently. If
`--check` flags lines your change didn't touch, leave them alone and format only your own code.

## Documentation

The site is built with MkDocs Material, exactly as the docs workflow does:

```bash
pip install "mkdocs<2" mkdocs-material mkdocs-github-admonitions-plugin
mkdocs build --strict      # broken links and anchors fail the build
mkdocs serve               # live preview on http://127.0.0.1:8000
```

| Page | Update it when |
| --- | --- |
| `docs/manual.md` | Anything a user sees: a command, a mode, a LED, a limit, a troubleshooting tip |
| `docs/at_commands.md` | An AT command is added, changed or gets a limit |
| `docs/diagnostics.md` | What can be done through which interface changes |
| `docs/developers.md` | The architecture, the tests or the workflow change |
| `README.md` | The *Current status* list changes |

---

## Sending a change

1. Fork, and branch from `main`: `git checkout -b pr/<short-topic>`.
2. **Keep it small:** one feature or one fix per pull request. Stack dependent PRs, each based
   on the branch it needs, and say so.
3. **Commits.** The subject is `area: what it does`, lower case and imperative. Examples:
   `elm327: add AT CRA reply filter`, `kline: wait P3min before the first request`. The body says
   why, and what was seen on a vehicle if that's where it came from. Put docs in their own commit
   (`docs/manual.md: document AT CRA`).
4. **Pull request.** The template asks for *Summary*, *Why*, *Details*, *Depends on*, *Docs* and
   *Testing*. Under *Testing*, separate what you ran (host tests, build, `mkdocs --strict`) from
   what still needs **confirming on hardware**, and name the vehicle if you tested on one.
5. **Before you push:** host tests pass, the firmware builds, `mkdocs build --strict` passes, and
   your code is formatted.

Bug reports are most useful with the firmware version (`version` in the shell), the vehicle and
the protocol, and a log of the exchange.

---

## Open work

These are not implemented yet. A pull request for any of them is welcome.

| Area | Item | Notes |
| --- | --- | --- |
| ELM327 | Interrupting a request with any byte (`STOPPED`) | Works in `AT MA` only. Apps use it to cancel |
| ELM327 | CAN requests longer than one frame | Needed for long `2E`/`31` over ELM327. Today it answers `?` |
| ELM327 | `CSM0`, monitor on K-Line and J1850, `MR`/`MT` on 11-bit | `MA` is CAN only and always silent |
| ELM327 | Error replies `CAN ERROR`, `BUS ERROR`, `DATA ERROR`, `BUFFER FULL` | Most failures answer `?` or `NO DATA` |
| ELM327 | `M1` and persistent settings (protocol, `PP`, `SD`) | Needs NVS. Everything is in RAM today |
| ELM327 | `CV` voltage calibration, `@2`/`@3`, `RTR`, `C0`/`C1`, `CTM`/`JTM`, `IFR4–6`, `PP 00–2B` | Smaller items |
| ELM327 | Protocol A (SAE J1939) | Trucks |
| ELM327 | STN/OBDLink extensions (`STI`, `STFAP`, `STCSEGT`…) | Check the STN reference manual for exact syntax |
| Firmware | Two CAN buses at once | One TWAI controller; the second transceiver serves AUX or pins 3/11 in turn |
| Firmware | The debug shell over WiFi | Only the data link is on TCP 35000 |
| Firmware | Joining an existing WiFi network (station mode) | Only the adapter's own access point today |
| Firmware | A persistent per-link default mode | A bench unit needs `mode usb slcan` after each power cycle |
| Docs | Vehicle notes: buses, addresses, quirks | Add what you find on your vehicle to the manual's troubleshooting section |
