# OpenDIAG - Open Source OBD2 Interface (Firmware)

Firmware for the OpenDIAG OBD-II diagnostic adapter — an ESP32-S3 based interface that presents an
ELM327-compatible AT command interface over USB and Bluetooth LE, and talks to most physical bus
found on an OBD-II connector.

> [!WARNING]
> **No warranty. Use at your own risk.** This software is provided as-is, and
> everything it does to a vehicle is your responsibility. See
> [Disclaimer](#disclaimer).

## Features

*   **ELM327 Compatible:** Provides an AT command set over USB and BLE, ensuring drop-in compatibility with existing ELM327-supported OBD-II software.
*   **Comprehensive OBD-II Physical Layers:** Supports CAN (ISO 15765-4), K-Line (ISO 9141-2), J1850 PWM, and J1850 VPW.
*   **SLCAN (LAWICEL) Protocol Support:** Provides raw, packet-level access to the CAN bus over a standard serial interface. This bypasses the ELM327 abstraction, allowing for advanced reverse engineering, custom UDS queries, and direct ECU flashing. Either link can be switched to it without disturbing a programming voltage already raised under ELM327.
*   **Advanced Pin Control:** Features programmable high-side and low-side drivers on the OBD connector pins, paired with a software-adjustable boost converter for manufacturer-specific wake-up and programming voltages.

## Building

Initialize the pinned upstream dependencies before firmware or host builds:

```bash
git submodule update --init --recursive
```

This is an [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) project, developed against
IDF v6.1. Install and activate the toolchain first:

```bash
. $IDF_PATH/export.sh
```

Then build and flash:

```bash
idf.py build
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM1 monitor   # the console is the SECOND port
```

The first port carries protocol traffic, not the console.

Run test suite:

```bash
cmake -S test/host -B test/host/build
cmake --build test/host/build
ctest --test-dir test/host/build
```

## Usage

Once flashed, the adapter enumerates as a USB device named **OpenDIAG** with two CDC ACM serial
ports, and advertises over BLE as **OpenDIAG**.

| | Purpose |
| --- | --- |
| USB CDC 0 | the data link — ELM327 by default, switchable to SLCAN |
| USB CDC 1 | console and debug shell |
| BLE UART characteristics | the data link — ELM327 by default, switchable to SLCAN |
| BLE control characteristic | the same control commands, for a BLE client |

Both data links start on **ELM327** and go back to it when their client disconnects, so any
existing ELM327 app works without being told anything.


## Current status

Implemented and working:

- ELM327 protocols 1 (J1850 PWM), 2 (J1850 VPW), 3 (ISO 9141-2), and 6–9 (CAN 11/29-bit at
  250k/500k)
- ELM327 over USB CDC port 0 and BLE, independently on each
- SLCAN (LAWICEL) on either link, for `slcand` and ECU reflashing
- Switching a link between the two without dropping its bus or programming voltage
- Board control: high-side/low-side drivers, boost converter, voltage sense, calibration

Not yet implemented:

- Protocols 10–12 (SAE J1939, USER1, USER2)
- The second CAN transceiver
- A persistent per-link default, so a bench unit has to be told `mode usb slcan` after each
  power cycle

Much of the ELM327 command set also remains unimplemented; `docs/at_commands.md` marks what is
supported today.

## Disclaimer

This software is provided **as-is, without warranty of any kind**, express or
implied, including but not limited to warranties of merchantability and fitness
for a particular purpose, as set out in sections 15 and 16 of the
[GNU General Public License](LICENSE). **You use it entirely at your own risk.**

- **No liability.** In no event shall the authors or contributors be liable for
  any claim, damage, or other loss arising from the use of this software — a
  bricked ECU, an immobilised vehicle, diagnostic or repair costs, lost data,
  personal injury, or anything else — whether in contract, tort, or otherwise.
- **You are responsible for what you send to a vehicle.** Nothing here can tell
  whether an image belongs in the module you are writing it to. Verify the
  target, the file, and the addresses yourself.
- **Only work on vehicles you own or are authorised to modify.** Programming an
  ECU may void warranties, and doing it to someone else's vehicle without
  permission may be unlawful.
- **Check your local law.** Modifying emissions-related calibrations is
  regulated in many jurisdictions and may make a vehicle illegal to drive on
  public roads. Handling manufacturer firmware may also carry its own
  restrictions. This project takes no position on either — that is yours to
  establish.
- **Safety.** A module that stops responding mid-flash can leave a vehicle
  undriveable without warning. Work on a stationary vehicle, in a place where it
  can stay put, on a stable supply.

## License

OpenDIAG is free software: you may redistribute it and modify it under the terms
of **version 3 of the GNU General Public License**, as published by the Free
Software Foundation. No later version applies. The full text is in
[LICENSE](LICENSE).

The GPL is copyleft: anything you distribute that is derived from this code must
be released under the same licence, with source. It carries no warranty — see
sections 15 and 16 of the licence, and [Disclaimer](#disclaimer).
