# ELM327 AT command support

What this firmware answers to, against the ELM327 v2.3 datasheet. Anything
unticked is parsed far enough to be refused with `?` rather than silently
ignored - a client that asks for something and gets `OK` for it has been lied
to, which is worse than being told no.

`N/A` marks commands that describe hardware this adapter does not have: the
RS232 baud rate divisors, for instance, on a device whose links are USB and
Bluetooth.

### General Commands

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `<CR>` | repeat the last command | ✅ |
| `BRD hh` | try Baud Rate Divisor hh | ✅ handshake only, see below |
| `BRT hh` | set Baud Rate Timeout | ✅ accepted, no effect |
| `D` | set all to Defaults | ✅ |
| `E0, E1` | Echo off, or on | ✅ |
| `FE` | Forget Events | |
| `I` | print the version ID | ✅ |
| `L0, L1` | Linefeeds off, or on | ✅ |
| `LP` | go to Low Power mode | |
| `M0, M1` | Memory off, or on | ✅ |
| `RD` | Read the stored Data byte | ✅ |
| `SD hh` | Save Data byte hh | ✅ RAM only |
| `WS` | Warm Start (quick software reset) | ✅ |
| `Z` | reset all | ✅ |
| `@1` | display the device description | ✅ |
| `@2` | display the device identifier | |
| `@3 cccccccccccc` | store the @2 identifier | |

### Programmable Parameter Commands

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `PP xx OFF` | disable Prog Parameter xx | ✅ 2C-2F only |
| `PP FF OFF` | all Prog Parameters disabled | ✅ 2C-2F only |
| `PP xx ON` | enable Prog Parameter xx | ✅ 2C-2F only |
| `PP FF ON` | all Prog Parameters enabled | ✅ 2C-2F only |
| `PP xx SV yy` | for PP xx, Set the Value to yy | ✅ 2C-2F only |
| `PPS` | print a PP Summary | ✅ 2C-2F only |

### Voltage Reading Commands

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `CV dddd` | Calibrate the Voltage to dd.dd volts | |
| `CV 0000` | restore CV value to factory setting | |
| `RV` | Read the input Voltage | ✅ |

### Other

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `IGN` | read the IgnMon input level | N/A |

### OBD Commands

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `AL` | Allow Long (>7 byte) messages | ✅ |
| `AMC` | display Activity Monitor Count | |
| `AMT hh` | set the Activity Mon Timeout to hh | |
| `AR` | Automatically Receive | ✅ |
| `AT0, 1, 2` | Adaptive Timing off, auto1, auto2 | ✅ |
| `BD` | perform a Buffer Dump | |
| `BI` | Bypass the Initialization sequence | ✅ K-Line, CAN and J1850 |
| `DP` | Describe the current Protocol | ✅ |
| `DPN` | Describe the Protocol by Number | ✅ |
| `FT` | Filter for Transmitter off | ✅ |
| `FT hh` | Filter for Transmitter = hh | ✅ |
| `H0, H1` | Headers off, or on | ✅ |
| `IA` | Is the protocol Active? | ✅ |
| `MA` | Monitor All | ✅ CAN and K-Line |
| `MR hh` | Monitor for Receiver = hh | ✅ CAN 29 bit and K-Line |
| `MT hh` | Monitor for Transmitter = hh | ✅ CAN 29 bit and K-Line |
| `NL` | Normal Length messages | ✅ |
| `PC` | Protocol Close | ✅ |
| `R0, R1` | Responses off, or on | ✅ |
| `RA hh` | set the Receive Address to hh | ✅ |
| `S0, S1` | printing of Spaces off, or on | ✅ |
| `SH xyz` | Set Header to xyz | ✅ |
| `SH xxyyzz` | Set Header to xxyyzz | ✅ |
| `SH wwxxyyzz` | Set Header to wwxxyyzz | ✅ |
| `SP h` | Set Protocol to h and save it | ✅ |
| `SP Ah` | Set Protocol to Auto, h and save it | ✅ |
| `SP 00` | Erase stored protocol | ✅ |
| `SR hh` | Set the Receive address to hh | ✅ |
| `SS` | use Standard Search order (J1978) | ✅ |
| `ST hh` | Set Timeout to hh x 4 msec | ✅ |
| `TA hh` | set Tester Address to hh | ✅ |
| `TP h` | Try Protocol h | ✅ |
| `TP Ah` | Try Protocol h with Auto search | ✅ |

### J1850 Specific Commands (protocols 1 and 2)

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `IFR0, 1, 2` | IFRs off, auto, or on, if not monitoring | ✅ |
| `IFR4, 5, 6` | IFRs off, auto, or on, at all times | ✅ |
| `IFRH, S` | IFR value from Header or Source | ✅ |

### ISO Specific Commands (protocols 3 to 5)

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `FI` | perform a Fast Initiation | ✅ |
| `IB10` | set the ISO Baud rate to 10400 | ✅ |
| `IB12` | set the ISO Baud rate to 12500 | ✅ |
| `IB15` | set the ISO Baud rate to 15625 | ✅ |
| `IB48` | set the ISO Baud rate to 4800 | ✅ |
| `IB96` | set the ISO Baud rate to 9600 | ✅ |
| `IIA hh` | set ISO (slow) Init Address to hh | ✅ |
| `KW` | display the Key Words | ✅ |
| `KW0, KW1` | Key Word checking off, or on | ✅ |
| `SI` | perform a Slow (5 baud) Initiation | ✅ |
| `SW hh` | Set Wakeup interval to hh x 20 msec | ✅ |
| `SW 00` | Stop sending Wakeup messages | ✅ |
| `WM [1 - 6 bytes]` | set the Wakeup Message | ✅ |

On protocol 5 the fast initiation follows the header. By default, and with any
functional header (`AT SH C1 33 F1`, format byte `Cx`), it is the OBD request
`C1 33 F1 81`, and `AT IIA hh` changes its target. With a physical header
(format byte `8x`, for example `AT SH 81 20 F1`), the session is started on that
one ECU, `81 20 F1 81`, and is ended the same way. Body and chassis ECUs that
are not in the OBD group `$33`, such as the ABS at `$20` on Fiat/PSA vehicles,
answer nothing else:

```
AT SP 5
AT SH 81 20 F1
10 81            -> BUS INIT: OK, then 50 81 from the ABS
```

### CAN Specific Commands (protocols 6 to C)

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `C0, C1` | send Confirmation off, or on | |
| `CAF0, CAF1` | Automatic Formatting off, or on |  ✅ |
| `CEA` | CAN Extended Addressing off | ✅ |
| `CEA hh` | use CAN Extended Address hh | ✅ |
| `CER hh` | set CAN Extended Rx address to hh | ✅ |
| `CF hhh` | set the ID Filter to hhh | ✅ |
| `CF hhhhhhhh` | set the ID Filter to hhhhhhhh | ✅ |
| `CFC0, CFC1` | Flow Controls off, or on | ✅ |
| `CM hhh` | set the ID Mask to hhh | ✅ |
| `CM hhhhhhhh` | set the ID Mask to hhhhhhhh | ✅ |
| `CP hh` | set CAN Priority to hh (29 bit) | ✅ |
| `CRA` | reset the Receive Address filters | ✅ |
| `CRA hhh` | set CAN Receive Address to hhh | ✅ |
| `CRA hhhhhhhh` | set the Rx Address to hhhhhhhh | ✅ |
| `CS` | show CAN Status counts & freq | ✅ error counts only |
| `CSM0, CSM1` | Silent Monitoring off, or on | |
| `CTM1` | set Timer Multiplier to 1 | |
| `CTM5` | set Timer Multiplier to 5 | |
| `D0, D1` | display of the DLC off, or on | ✅ |
| `FC SM h` | Flow Control, Set the Mode to h | ✅ |
| `FC SH hhh` | FC, Set the Header to hhh | ✅ |
| `FC SH hhhhhhhh` | Set the Header to hhhhhhhh | ✅ |
| `FC SD [1 - 5 bytes]` | FC, Set Data to [...] | ✅ CTS only, see below |
| `PB xx yy` | Protocol B options and baud rate | ✅ no J1939 format |
| `RTR` | send an RTR message | |
| `V0, V1` | use of Variable DLC off, or on | ✅ |

### J1939 CAN Specific Commands (protocols A to C)

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `DM1` | monitor for DM1 messages | |
| `JE` | use J1939 Elm data format | |
| `JHF0, JHF1` | Header Formatting off, or on | |
| `JS` | use J1939 SAE data format | |
| `JTM1` | set Timer Multiplier to 1 | |
| `JTM5` | set Timer Multiplier to 5 | |
| `MP hhhh` | Monitor for PGN 0hhhh | |
| `MP hhhh n` | " and get n messages | |
| `MP hhhhhh` | Monitor for PGN hhhhhh | |
| `MP hhhhhh n` | " and get n messages | |

### Notes on the CAN additions

- **Protocols B and C** run on the CAN transceiver at 500 kbit/s divided by
  the `PB` divisor (or PP 2D / 2F). The driver takes 10, 20, 33.3, 50, 83.3,
  95.2, 100, 125, 250, 500, 800 and 1000 kbit/s; any other rate is refused
  when the bus opens. Like protocols 6
  to 9, they listen before driving the bus (`canguard`).
- **`CRA` / `CF` / `CM`** replace the ISO 15765-4 reply ranges (7E8-7EF,
  18DAF1xx). With a filter set, a flow control for a physical request goes
  out on the request's own identifier, unless `FC SM 1` names another.
- **`FC SD`** takes a clear-to-send flow control only: `30`, block size,
  STmin. Bytes after the third are sent as padding.
- **`CEA hh`** adds the extended address to every frame sent and strips it
  from replies in the formatted (`CAF1`, `H0`) view. `CER hh` accepts only
  replies that carry that address; without it, any address is accepted.
- **`MA` / `MR` / `MT`** reopen the bus listen-only, like the ELM327's
  default `CSM1`, and release it when stopped. Frames are taken from a
  32-frame queue every 50 ms, so a busy 500 kbit/s bus drops frames: use
  SLCAN for a full capture.
- **`MA` / `MR` / `MT` on K-Line** (protocols 3, 4 and 5) listen on pin 7 at
  10.4 kbaud without initialising, so the adapter never drives the line. Each
  message, as the gap between bytes delimits it, is printed as it was on the
  wire: header, data and checksum, also when the checksum is wrong. `MR hh` and
  `MT hh` keep messages whose target or source header byte is `hh`. Use it to
  watch what another scan tool asks an ECU.
- **`CS`** prints `T:hh R:hh`, the controller's transmit and receive error
  counters, read while the bus is open.
- **`BRD hh`**: USB and BLE have no baud rate, so only the handshake runs:
  `OK`, the ID string, then a bare carriage return from the client answers
  `OK` and a prompt.
- **`BI` on CAN and J1850** answers `OK` and sends nothing: these buses have
  no initiation to bypass, and a real ELM327 accepts it there too. Some tools
  (Multiecuscan) send `AT BI` after `AT SP C` as their "ELM327 1.3 or later"
  check and refuse every CAN module on `?`. With automatic search (`SP 0`) it
  is still refused.
- **`SD` / `RD`** and the PP values survive `AT Z` and a reconnect, but not
  a power cycle.
- Long replies whose first consecutive frame is numbered 0 instead of 1 are
  accepted.

### Error messages

What a request can answer besides its data, as the ELM327 words it:

| Message | When |
| --- | --- |
| `NO DATA` | Nothing answered within the timeout |
| `?` | The request itself is wrong: not hex, or too long for one CAN frame |
| `CAN ERROR` | The CAN bus could not be opened (for example, the listen-first check found another bit rate on it) or a frame or flow control could not be sent |
| `BUS ERROR` | A K-Line or J1850 transmission failed: a collision, a broken echo, the driver refused it |
| `BUS BUSY` | The K-Line or J1850 bus never went idle long enough to start |
| `DATA ERROR` | A multi-frame CAN reply arrived broken: out of sequence or longer than it said |
| `<DATA ERROR` | At the end of a K-Line or J1850 line: that message's checksum failed. The data is shown anyway |
| `BUFFER FULL` | Before a line: replies were lost because they arrived faster than they were read |
| `BUS INIT: ...ERROR` | The K-Line initialisation failed |

A failed automatic search still answers `NO DATA` or `?` as before.

### Custom Extensions (Not in ELM327 Spec)

| CMD | Description |
| :--- | :--- |
| `PROGV p vvvvvvvv` | drive an OBD-II pin `p`, or release it |
| `VIF PASSTHRU` | Select the J2534 binary frontend on the current data connection. |
| `VIF SLCAN` | Select the SLCAN frontend on the current data connection. |
