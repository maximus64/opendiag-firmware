### General Commands

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `<CR>` | repeat the last command | ✅ |
| `BRD hh` | try Baud Rate Divisor hh | N/A |
| `BRT hh` | set Baud Rate Timeout | N/A |
| `D` | set all to Defaults | ✅ |
| `E0, E1` | Echo off, or on | ✅ |
| `FE` | Forget Events | |
| `I` | print the version ID | ✅ |
| `L0, L1` | Linefeeds off, or on | ✅ |
| `LP` | go to Low Power mode | |
| `M0, M1` | Memory off, or on | |
| `RD` | Read the stored Data byte | |
| `SD hh` | Save Data byte hh | |
| `WS` | Warm Start (quick software reset) | |
| `Z` | reset all | ✅ |
| `@1` | display the device description | |
| `@2` | display the device identifier | |
| `@3 cccccccccccc` | store the @2 identifier | |

### Programmable Parameter Commands

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `PP xx OFF` | disable Prog Parameter xx | |
| `PP FF OFF` | all Prog Parameters disabled | |
| `PP xx ON` | enable Prog Parameter xx | |
| `PP FF ON` | all Prog Parameters enabled | |
| `PP xx SV yy` | for PP xx, Set the Value to yy | |
| `PPS` | print a PP Summary | |

### Voltage Reading Commands

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `CV dddd` | Calibrate the Voltage to dd.dd volts | |
| `CV 0000` | restore CV value to factory setting | |
| `RV` | Read the input Voltage | ✅ |

### Other

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `IGN` | read the IgnMon input level | |

### OBD Commands

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `AL` | Allow Long (>7 byte) messages | |
| `AMC` | display Activity Monitor Count | |
| `AMT hh` | set the Activity Mon Timeout to hh | |
| `AR` | Automatically Receive | |
| `AT0, 1, 2` | Adaptive Timing off, auto1, auto2 | |
| `BD` | perform a Buffer Dump | |
| `BI` | Bypass the Initialization sequence | |
| `DP` | Describe the current Protocol | |
| `DPN` | Describe the Protocol by Number | ✅ |
| `FT` | Filter for Transmitter off | |
| `FT hh` | Filter for Transmitter = hh | |
| `H0, H1` | Headers off, or on | ✅ |
| `IA` | Is the protocol Active? | |
| `MA` | Monitor All | |
| `MR hh` | Monitor for Receiver = hh | |
| `MT hh` | Monitor for Transmitter = hh | |
| `NL` | Normal Length messages | |
| `PC` | Protocol Close | |
| `R0, R1` | Responses off, or on | |
| `RA hh` | set the Receive Address to hh | |
| `S0, S1` | printing of Spaces off, or on | ✅ |
| `SH xyz` | Set Header to xyz | ✅ |
| `SH xxyyzz` | Set Header to xxyyzz | ✅ |
| `SH wwxxyyzz` | Set Header to wwxxyyzz | ✅ |
| `SP h` | Set Protocol to h and save it | ✅ |
| `SP Ah` | Set Protocol to Auto, h and save it | |
| `SP 00` | Erase stored protocol | |
| `SR hh` | Set the Receive address to hh | |
| `SS` | use Standard Search order (J1978) | |
| `ST hh` | Set Timeout to hh x 4 msec | ✅ |
| `TA hh` | set Tester Address to hh | |
| `TP h` | Try Protocol h | |
| `TP Ah` | Try Protocol h with Auto search | |

### J1850 Specific Commands (protocols 1 and 2)

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `IFR0, 1, 2` | IFRs off, auto, or on, if not monitoring | |
| `IFR4, 5, 6` | IFRs off, auto, or on, at all times | |
| `IFRH, S` | IFR value from Header or Source | |

### ISO Specific Commands (protocols 3 to 5)

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `FI` | perform a Fast Initiation | |
| `IB10` | set the ISO Baud rate to 10400 | |
| `IB12` | set the ISO Baud rate to 12500 | |
| `IB15` | set the ISO Baud rate to 15625 | |
| `IB48` | set the ISO Baud rate to 4800 | |
| `IB96` | set the ISO Baud rate to 9600 | |
| `IIA hh` | set ISO (slow) Init Address to hh | |
| `KW` | display the Key Words | |
| `KW0, KW1` | Key Word checking off, or on | |
| `SI` | perform a Slow (5 baud) Initiation | |
| `SW hh` | Set Wakeup interval to hh x 20 msec | |
| `SW 00` | Stop sending Wakeup messages | |
| `WM [1 - 6 bytes]` | set the Wakeup Message | |

### CAN Specific Commands (protocols 6 to C)

| CMD | Description | Supported |
| :--- | :--- | :--- |
| `C0, C1` | send Confirmation off, or on | |
| `CAF0, CAF1` | Automatic Formatting off, or on |  ✅ |
| `CEA` | CAN Extended Addressing off | |
| `CEA hh` | use CAN Extended Address hh | |
| `CER hh` | set CAN Extended Rx address to hh | |
| `CF hhh` | set the ID Filter to hhh | |
| `CF hhhhhhhh` | set the ID Filter to hhhhhhhh | |
| `CFC0, CFC1` | Flow Controls off, or on | |
| `CM hhh` | set the ID Mask to hhh | |
| `CM hhhhhhhh` | set the ID Mask to hhhhhhhh | |
| `CP hh` | set CAN Priority to hh (29 bit) | |
| `CRA` | reset the Receive Address filters | |
| `CRA hhh` | set CAN Receive Address to hhh | |
| `CRA hhhhhhhh` | set the Rx Address to hhhhhhhh | |
| `CS` | show CAN Status counts & freq | |
| `CSM0, CSM1` | Silent Monitoring off, or on | |
| `CTM1` | set Timer Multiplier to 1 | |
| `CTM5` | set Timer Multiplier to 5 | |
| `D0, D1` | display of the DLC off, or on | |
| `FC SM h` | Flow Control, Set the Mode to h | |
| `FC SH hhh` | FC, Set the Header to hhh | |
| `FC SH hhhhhhhh` | Set the Header to hhhhhhhh | |
| `FC SD [1 - 5 bytes]` | FC, Set Data to [...] | |
| `PB xx yy` | Protocol B options and baud rate | |
| `RTR` | send an RTR message | |
| `V0, V1` | use of Variable DLC off, or on | |

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
