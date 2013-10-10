@echo off
set flasher="%ProgramFiles%\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"
if exist %flasher% goto :flash
set flasher="%ProgramFiles(x86)%\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"

:flash
set filename=%1%
if "%filename%" == "" set filename=out\STM32Gimbal.bin

%flasher% -c SWD -P %filename% 0x08000000 -V -Rst
pause