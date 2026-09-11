@echo off
setlocal EnableExtensions DisableDelayedExpansion
set "INSTALL_DIR=C:\OpenDIAG"
set "PACKAGE_FILES=opendiag32.dll odg40432.dll opendiag_config.exe install.cmd uninstall.cmd README.txt COPYING nanopb-LICENSE.txt"
set "REGEXE=%SystemRoot%\System32\reg.exe"
if exist "%SystemRoot%\SysWOW64\reg.exe" set "REGEXE=%SystemRoot%\SysWOW64\reg.exe"
for %%F in (%PACKAGE_FILES%) do (
    if exist "%INSTALL_DIR%\%%F\" (
        echo Expected an installed file, found a directory: "%INSTALL_DIR%\%%F"
        exit /b 1
    )
)
for %%V in (04.04 05.00) do (
    "%REGEXE%" query "HKLM\SOFTWARE\PassThruSupport.%%V\OpenDIAG - OpenDIAG" >nul 2>&1
    if not errorlevel 1 (
        "%REGEXE%" delete "HKLM\SOFTWARE\PassThruSupport.%%V\OpenDIAG - OpenDIAG" /f >nul
        if errorlevel 1 (
            goto failed
        )
    )
)
for %%F in (%PACKAGE_FILES%) do (
    if /I not "%%F"=="uninstall.cmd" (
        if exist "%INSTALL_DIR%\%%F" del /q "%INSTALL_DIR%\%%F"
        if exist "%INSTALL_DIR%\%%F" goto failed
    )
)
rem Keep the final block together so this script can remove its installed copy.
(
    if exist "%INSTALL_DIR%\uninstall.cmd" del /q "%INSTALL_DIR%\uninstall.cmd"
    if exist "%INSTALL_DIR%\uninstall.cmd" (
        echo Cannot remove %INSTALL_DIR%\uninstall.cmd. Run as administrator.
        exit /b 1
    )
    if exist "%INSTALL_DIR%\" rmdir "%INSTALL_DIR%" 2>nul
    echo OpenDIAG registrations and installed program files removed.
    echo Existing opendiag.ini, logs and other files in %INSTALL_DIR% were preserved.
    exit /b 0
)

:failed
echo Removal failed. Close diagnostic applications and the configuration utility,
echo then run this script again from an administrator command prompt.
exit /b 1
