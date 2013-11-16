@echo off
set filename=%1%
set FLASHTOOL=setup\dfu-util-static.exe

if "%filename%" == "" set filename=out\STM32Gimbal.USB.bin

echo taking board into boot mode ...
REM boot loader hack by ala42
REM let dfu-util talk to the USB VCP address. This is detected by the VCP handler
REM which takes the board into the boot loader
%FLASHTOOL% --device 0483:5740 -D dummyname 2>1 >nul
