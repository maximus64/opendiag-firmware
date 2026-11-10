# Diagnostic services

OpenDIAG doesn't run diagnostic services itself. It carries them: the app or tool on the other
end builds each request, and the adapter frames it for the bus, sends it and returns the reply.
What you can do to a module therefore depends on two things: what the module accepts, and which
of the adapter's three interfaces you reach it through.

This page lists the common services, what each one does to a vehicle, and which interface can
carry it.

> [!WARNING]
> Reading is safe. Clearing, writing, running routines and programming **change the vehicle**,
> and some can't be undone. Read [Before you change anything](#before-you-change-anything) first.

---

## Which interface for which job

| Interface | Best for | Limits |
| --- | --- | --- |
| **ELM327** (USB or Bluetooth, any OBD-II app) | Reading, clearing codes, short writes | On CAN a request must fit one frame: 7 data bytes, 6 with `AT CEA`. A longer one answers `?`. No periodic tester present. |
| **J2534** (Windows, 32-bit apps) | Workshop software, coding, reprogramming | ISO 15765 messages up to 4095 bytes, up to 10 periodic messages per channel, filters and flow control set by the application. |
| **SLCAN** (raw CAN, `slcand`, python-can) | Development, reverse engineering, scripted sessions | Frames only. The host does ISO-TP (for example `can-isotp`, `udsoncan`, `isotpsend`). |

The ELM327 limit matters most for writing: a UDS `2E` write carries the service byte and a
2-byte identifier, so over ELM327 it fits at most 4 bytes of data (3 with extended addressing).
Anything longer needs J2534 or SLCAN.

---

## Services

The tables use the ISO service identifiers (SID). The positive reply is the SID plus `40`, for
example `22` → `62`. A negative reply is `7F <SID> <code>`; code `78` means *pending, wait*, and
the adapter keeps waiting for the real answer.

Risk: **read** changes nothing; **clear** discards stored information; **write** changes the
module's behaviour; **critical** can leave a module unusable if it goes wrong.

### OBD-II (SAE J1979), every protocol

| Mode | What it does | Risk |
| --- | --- | --- |
| `01` | Live data (PIDs): speed, rpm, temperatures, fuel trims | read |
| `02` | Freeze frame saved with a fault | read |
| `03` | Stored emissions trouble codes | read |
| `04` | **Clears** trouble codes, freeze frames *and readiness monitors* | clear |
| `06` | On-board monitor test results | read |
| `07` | Pending trouble codes | read |
| `08` | Control of an on-board system (rarely supported) | write |
| `09` | Vehicle information: VIN, calibration IDs | read |
| `0A` | Permanent trouble codes | read |

Mode `04` also resets the readiness monitors. A vehicle can then fail an emissions inspection
until it has been driven through its drive cycles again.

### UDS (ISO 14229), usually on CAN

| SID | Service | What it does | Risk |
| --- | --- | --- | --- |
| `10` | DiagnosticSessionControl | `01` default, `03` extended ("workshop"), `02` programming | read / see below |
| `11` | ECUReset | Restarts the module | write |
| `14` | ClearDiagnosticInformation | `14 FF FF FF` clears every code | clear |
| `19` | ReadDTCInformation | `19 02 FF` codes by status, `19 04` snapshots, `19 06` extended data | read |
| `22` | ReadDataByIdentifier | `22 F1 90` VIN, `22 F1 87` part number, `22 F1 8C` serial | read |
| `23` | ReadMemoryByAddress | Raw memory | read (can have side effects on some modules) |
| `27` | SecurityAccess | Unlocks protected services; seed and key are module-specific | write |
| `28` | CommunicationControl | Stops a module's normal messages | write |
| `2E` | WriteDataByIdentifier | Writes a value: coding, settings, adaptations | write |
| `2F` | InputOutputControlByIdentifier | Drives an actuator (lamp, fan, valve) | write |
| `31` | RoutineControl | Starts a routine: tests, calibrations, erase | write / critical |
| `34`–`37` | Request download/upload, transfer data | Reprogramming | critical |
| `3E` | TesterPresent | Keeps a non-default session open | read |
| `85` | ControlDTCSetting | Stops a module recording codes | write |

### KWP2000 (ISO 14230), K-Line or CAN

Older modules, and many on K-Line, use KWP2000. The common services differ from UDS:

| SID | Service | Risk |
| --- | --- | --- |
| `10` | StartDiagnosticSession (`81` default; others are manufacturer-specific) | read / see below |
| `14` | ClearDiagnosticInformation (`14 FF 00` all codes, on most modules) | clear |
| `17`, `18` | Read DTCs by status | read |
| `1A` | ReadEcuIdentification | read |
| `21`, `22` | Read data by local identifier / by common identifier | read |
| `2E`, `3B` | Write data by identifier / by local identifier | write |
| `30` | InputOutputControlByLocalIdentifier | write |
| `31` | StartRoutineByLocalIdentifier | write / critical |
| `3E` | TesterPresent | read |

---

## Sessions: default, workshop and programming

A module starts in its **default session**, where the reading services work. Writing, actuator
tests and routines usually need the **extended** ("workshop", "advanced") session, often followed
by `27` SecurityAccess. Reprogramming needs the **programming** session.

A non-default session ends by itself after a few seconds without traffic (S3 timeout, typically
5 s in UDS). Then the module goes back to default and refuses what the session allowed.

- **J2534:** the application sets up `3E 80` as a periodic message, and the adapter sends it on
  time.
- **ELM327 on K-Line:** the adapter repeats the wake-up message (`AT SW`, `AT WM`), which keeps
  the link alive.
- **ELM327 on CAN:** the adapter has no periodic message. The app must keep sending requests, or
  `3E 00`, within the timeout.
- **SLCAN:** the host sends `3E` itself.

---

## Reading through the ELM327 interface

These examples use headers off (`AT H0`), the default.

OBD-II needs no setup:

```text
AT SP 0      automatic protocol search
03           stored codes
09 02        VIN
```

A module outside the OBD-II address range needs its addresses. On CAN 11-bit, for a module that
answers a request on `7xx` with a reply on `7yy`:

```text
AT SP 6           500 kbit/s 11-bit (or AT PB / AT SP B for another rate)
AT SH 7xx         request identifier
AT CRA 7yy        accept replies only from the module
22 F1 90          read its VIN
19 02 FF          read its codes
```

For modules on a slower body bus, or that need extended addressing (`AT CEA`), see
[Modules outside the OBD-II address ranges](manual.md#modules-outside-the-obd-ii-address-ranges).
On K-Line, `AT SH` takes the format, target and source bytes, for example `AT SH 81 xx F1`, and
`AT FI` or `AT SI` starts the link.

---

## Before you change anything

These steps apply to clearing, writing, routines and programming, whichever interface you use.

1. **Stationary vehicle, ignition on, engine off,** unless the procedure says otherwise. Never
   while driving.
2. **A battery maintainer** for anything longer than a few requests. A voltage drop during a
   write or a flash can leave a module unusable.
3. **Back up first.** Read and save the module's identification (`1A`, or `22 F1 8x`) and every
   value you are about to change, with its exact reply. Without the original value there is
   nothing to restore.
4. **One change at a time,** then read it back and check that the module and the vehicle behave.
5. **Log the session.** Keep every request and reply. Most apps can export a log.
6. **Know the module.** Airbag, ABS, steering and immobiliser modules are the highest risk. A
   wrong write there can disable a safety system or stop the vehicle from starting.
7. **Clear codes after the repair, not before.** Clearing hides the evidence a diagnosis needs,
   and mode `04` resets the readiness monitors.

Nothing here can tell whether a value or an image belongs in the module you are sending it to.
That check is yours. See the [disclaimer](https://github.com/maximus64/opendiag-firmware#disclaimer).
