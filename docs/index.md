# OpenDIAG

OpenDIAG is an open-source OBD-II interface built around an ESP32-S3. It speaks the ELM327
command set most OBD-II apps expect. It is also an SAE J2534 pass-thru device for Windows
diagnostic and programming software. For developers, it gives raw CAN access over SLCAN and
direct control over the connector's pins.

> [!WARNING]
> **Use at your own risk.** OpenDIAG can reprogram vehicle modules and put up to 20 V on connector
> pins. Read the [safety notes](manual.md#safety) before connecting it to a car you depend on.

## What it does

| | |
| --- | --- |
| **OBD-II apps** | ELM327-compatible over USB or Bluetooth LE. Works with any app that supports a BLE or USB ELM327 adapter. |
| **J2534 pass-thru** | 32-bit Windows driver, API 05.00 and 04.04, for diagnostic and reprogramming software. |
| **SLCAN** | Raw CAN for SocketCAN (`slcand`), SavvyCAN, python-can and similar tools. |
| **Vehicle buses** | CAN (ISO 15765-4), K-Line and L-Line (ISO 9141-2, ISO 14230-4 KWP), SAE J1850 PWM and VPW. |
| **Pin control** | Programming voltage from 5 V to 20 V on OBD-II pins 6, 9, 11–14, and a low-side switch on pin 15. |

## Get started

1. Plug OpenDIAG into the vehicle's OBD-II port. The LED breathes cyan.
2. Connect from a computer over [USB](manual.md#connecting-over-usb), or
   [pair a phone](manual.md#pairing-a-phone) over Bluetooth LE.
3. Pick the mode for your software:
    - An OBD-II app: nothing to do. The adapter starts in ELM327 mode.
    - Windows J2534 software: [install the driver](manual.md#j2534-pass-thru-on-windows).
    - CAN tools: [switch to SLCAN](manual.md#slcan-mode-for-developers).

## Update the firmware

Flash the latest release from your browser on the
**[firmware update page](update/index.html)**. It works in desktop Chrome or Edge, with no
drivers or tools to install.

## Documentation

- [Firmware update](update/index.html): install the latest firmware from your browser.
- [User manual](manual.md): setup, modes, the debug shell, firmware updates, and troubleshooting.
- [ELM327 commands](at_commands.md): which AT commands are supported.

## Related projects

- [opendiag-firmware](https://github.com/maximus64/opendiag-firmware): this firmware.
  Releases, including the J2534 driver, are on its
  [releases page](https://github.com/maximus64/opendiag-firmware/releases).
- [opendiag-hardware](https://github.com/maximus64/opendiag-hardware): hardware design files.
- [opendiag-cli](https://github.com/maximus64/opendiag-cli): the companion command-line tool.

OpenDIAG firmware is free software under the GNU General Public License, version 3, and comes
with no warranty.
