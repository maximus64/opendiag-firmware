# OpenDIAG J2534 driver

J2534 host driver for OpenDiag. This target the J2534 **05.00** API, and also provide **04.04** adapter for older programs.

Build from the repository root with GNU Make 4.3 or newer, a C/C++ compiler,
Python 3 with `protobuf`, and `protoc`:

```sh
git submodule update --init --recursive
make -C j2534_driver -j4
make -C j2534_driver test
```

Make uses the native compiler by default. Set `PYTHON` to a protobuf-enabled
interpreter if needed, for example `PYTHON=../.venv/bin/python`.

Build the Windows DLLs and configuration utility through Docker:

```sh
mkdir -p build/docker
docker compose run --rm --build j2534-driver
```

The container runs the host tests, then builds and checks the Windows package
once using its i686 MSVCRT MinGW cross-compiler. The resulting archive is
`build/docker/opendiag-j2534-driver.zip`.

Install both DLLs from the same package; the 04.04 shim uses a private bridge in
the native DLL to preserve its channel semantics.

The ZIP includes `opendiag-cdc-xp.inf` for the diagnostic data and debug console
USB CDC ports on Windows XP SP3 (x86), using Windows' built-in `usbser.sys`.
Follow the hardware wizard instructions in [packaging/README.txt](packaging/README.txt).
The INF source is [packaging/opendiag-cdc-xp.inf](packaging/opendiag-cdc-xp.inf).

## Configuration and logs

Default locations:

| Platform | System configuration | User log directory |
| --- | --- | --- |
| Windows Vista and later | `%ProgramData%\OpenDIAG\opendiag.ini` | `%LOCALAPPDATA%\OpenDIAG\logs\` |
| Windows XP SP3 | `%ALLUSERSPROFILE%\Application Data\OpenDIAG\opendiag.ini` | `%USERPROFILE%\Local Settings\Application Data\OpenDIAG\logs\` |
| Linux | `/etc/opendiag/opendiag.ini` | `$HOME/.local/state/opendiag/` |
| macOS | `/Library/Application Support/OpenDIAG/opendiag.ini` | `$HOME/Library/Logs/OpenDIAG/` |

On Linux, an absolute `XDG_STATE_HOME` changes the log directory to `$XDG_STATE_HOME/opendiag/`.

The driver reads `OPENDIAG_DRIVER_CONFIG` when set, otherwise the system file.
The selected file must exist. Configuration errors produce an empty scan; a failed
`PassThruOpen` gives the reason through `PassThruGetLastError`.
Use a full path and set it for both the configuration utility and diagnostic application.

The Windows utility saves the override or system file, starting with defaults for a new file.

Logging is off by default. Enable it in the Windows utility or set `Enabled=1` under `[logging]`.
If the user log directory cannot be created, logs go beside the library.
