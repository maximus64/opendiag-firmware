OpenDIAG J2534 driver - 32-bit Windows

Target baseline: Windows XP SP3 (x86). The same DLLs are intended for 32-bit
applications on Windows 7, 8/8.1, 10 and 11, including x64 Windows via WOW64.
Native 64-bit applications cannot load these DLLs.

Install
1. Close diagnostic applications and the configuration utility.
2. Extract the ZIP into a temporary folder, keeping its files together.
3. Run install.cmd from an administrator command prompt. It copies the DLLs,
   configuration utility, scripts and license files into C:\OpenDIAG, then
   registers that folder in the 32-bit J2534 registry view, including on x64
   Windows. An existing C:\OpenDIAG\opendiag.ini is preserved on upgrades.
4. Run C:\OpenDIAG\opendiag_config.exe. Select the USB data COM port or
   TCP endpoint, save, and use Save & Test before opening diagnostic software.

APIs
opendiag32.dll: J2534-1 05.00. Use include/j2534.h.
odg40432.dll: J2534-1 04.04 adapter. Use include/j2534_0404.h.
Each header requires j2534_types.h. Do not mix the two API headers in one
translation unit. This is the existing bounded firmware subset, not a claim of
complete J2534 conformance. SCI and unsupported firmware features remain absent.

Only one cooperating application can own an endpoint in a Windows session.
The driver selects AT VIF PASSTHRU over the data connection. It retries only
the startup capabilities probe; vehicle operations are not automatically replayed.

Uninstall
Close diagnostic applications and the configuration utility, then run
C:\OpenDIAG\uninstall.cmd from an administrator command prompt. The copy in
the extracted ZIP also works. It removes both registry entries and the installed
DLLs, configuration utility, scripts and support files from C:\OpenDIAG.
Your opendiag.ini, logs and unrelated files are preserved; the folder is removed
only if empty. Delete retained settings and logs manually for a complete reset.
Other vendors' registry entries and files are not modified.
