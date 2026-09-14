OpenDIAG J2534 driver - 32-bit Windows

Target baseline: Windows XP SP3 (x86). The same DLLs are intended for 32-bit
applications on Windows 7, 8/8.1, 10 and 11, including x64 Windows via WOW64.
Native 64-bit applications cannot load these DLLs.

Install
1. Close diagnostic applications and the configuration utility.
2. Extract the ZIP into a temporary folder, keeping its files together.
3. Run install.cmd from an administrator command prompt. It copies the DLLs,
   configuration utility, CDC INF, scripts and license files into C:\OpenDIAG, then
   registers that folder in the 32-bit J2534 registry view, including on x64
   Windows. An existing C:\OpenDIAG\opendiag.ini is preserved on upgrades.
4. For USB on Windows XP, complete the CDC setup below to create the COM ports.
   TCP connections do not require the CDC driver.
5. Run C:\OpenDIAG\opendiag_config.exe. Select the USB data COM port or
   TCP endpoint, save, and use Save & Test before opening diagnostic software.

Windows XP SP3 USB CDC setup (32-bit x86 only)
opendiag-cdc-xp.inf uses the Microsoft usbser.sys driver supplied with Windows.
It supports the two CDC functions exposed by normal OpenDIAG firmware:
  USB\VID_303A&PID_4002&MI_00 - OpenDIAG Diagnostic Data Port
  USB\VID_303A&PID_4002&MI_02 - OpenDIAG Debug Console Port

1. Connect the OpenDIAG USB port while logged in as an administrator.
2. In the Found New Hardware Wizard, select "No, not this time" for Windows
   Update, then "Install from a list or specific location (Advanced)".
3. Choose "Search for the best driver in these locations", enable "Include this
   location in the search", and browse to C:\OpenDIAG or the extracted ZIP folder.
   The wizard selects the matching port from opendiag-cdc-xp.inf. If the device
   was already detected, use Device Manager > Update Driver to open the wizard.
4. This INF is unsigned. Choose "Continue Anyway" if XP shows a Windows Logo
   testing warning. If Windows requests usbser.sys, use the Windows XP SP3
   installation source; the ZIP does not redistribute Windows system files.
5. Complete the wizard for both ports. Under Device Manager > Ports (COM & LPT),
   note the COM number for "OpenDIAG Diagnostic Data Port" and select that port
   in opendiag_config.exe. The debug console port is not the J2534 data link.

install.cmd copies the INF but does not run the hardware wizard. This INF is
for 32-bit XP; the J2534 DLLs' WOW64 support does not make it an x64 USB driver.
opendiag.ini contains J2534 connection settings and is not a USB installer.

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
