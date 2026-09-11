@echo off
setlocal EnableExtensions DisableDelayedExpansion
set "INSTALL_DIR=C:\OpenDIAG"
set "PACKAGE_FILES=opendiag32.dll odg40432.dll opendiag_config.exe install.cmd uninstall.cmd README.txt COPYING nanopb-LICENSE.txt"
set "REGEXE=%SystemRoot%\System32\reg.exe"
if exist "%SystemRoot%\SysWOW64\reg.exe" set "REGEXE=%SystemRoot%\SysWOW64\reg.exe"
for %%F in (%PACKAGE_FILES% opendiag.ini) do (
    if exist "%~dp0%%F\" (
        echo Expected a package file, found a directory: "%~dp0%%F"
        exit /b 1
    )
    if not exist "%~dp0%%F" (
        echo Missing file: "%~dp0%%F"
        exit /b 1
    )
    if exist "%INSTALL_DIR%\%%F\" (
        echo Expected an installed file, found a directory: "%INSTALL_DIR%\%%F"
        exit /b 1
    )
)
if not exist "%INSTALL_DIR%\" (
    mkdir "%INSTALL_DIR%"
    if errorlevel 1 goto failed
)
for %%D in ("%~dp0.") do set "SOURCE_DIR=%%~fD"
if /I not "%SOURCE_DIR%"=="%INSTALL_DIR%" (
    for %%F in (%PACKAGE_FILES%) do (
        copy /b /y "%~dp0%%F" "%INSTALL_DIR%\%%F" >nul
        if errorlevel 1 goto failed
    )
    if not exist "%INSTALL_DIR%\opendiag.ini" (
        copy /b /y "%~dp0opendiag.ini" "%INSTALL_DIR%\opendiag.ini" >nul
        if errorlevel 1 goto failed
    )
)
call :register 05.00 opendiag32.dll
if errorlevel 1 goto failed
call :register 04.04 odg40432.dll
if errorlevel 1 goto failed
for %%P in (CAN ISO15765 J1850PWM J1850VPW ISO9141 ISO14230) do (
    "%REGEXE%" add "%KEY%" /v %%P /t REG_DWORD /d 1 /f >nul
    if errorlevel 1 goto failed
)
for %%P in (SCI_A_ENGINE SCI_A_TRANS SCI_B_ENGINE SCI_B_TRANS) do (
    "%REGEXE%" add "%KEY%" /v %%P /t REG_DWORD /d 0 /f >nul
    if errorlevel 1 goto failed
)
echo OpenDIAG installed in %INSTALL_DIR% and registered for 32-bit applications.
echo Existing opendiag.ini settings were preserved.
echo Configure the adapter with %INSTALL_DIR%\opendiag_config.exe.
echo Restart diagnostic applications to refresh their adapter list.
exit /b 0

:register
set "KEY=HKLM\SOFTWARE\PassThruSupport.%~1\OpenDIAG - OpenDIAG"
"%REGEXE%" add "%KEY%" /v Vendor /t REG_SZ /d "OpenDIAG" /f >nul
if errorlevel 1 exit /b 1
"%REGEXE%" add "%KEY%" /v Name /t REG_SZ /d "OpenDIAG" /f >nul
if errorlevel 1 exit /b 1
"%REGEXE%" add "%KEY%" /v FunctionLibrary /t REG_SZ /d "%INSTALL_DIR%\%~2" /f >nul
if errorlevel 1 exit /b 1
"%REGEXE%" add "%KEY%" /v ConfigApplication /t REG_SZ /d "%INSTALL_DIR%\opendiag_config.exe" /f >nul
exit /b %errorlevel%

:failed
echo Installation failed. Close diagnostic applications and the configuration utility,
echo then run this script again from an administrator command prompt.
exit /b 1
