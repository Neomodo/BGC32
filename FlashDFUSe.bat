@echo off
REM set filename=%1%
REM set FLASHTOOL=setup\dfu-util-static.exe
set FLASHTOOL="C:\Users\John Ihlein\Desktop\STM32workspace\BGC32\setup\dfu-util-static.exe"

arm-none-eabi-objcopy -O binary "C:\Users\John Ihlein\Desktop\STM32workspace\BGC32\Release\BGC32.elf" "C:\Users\John Ihlein\Desktop\STM32workspace\BGC32\Release\BGC32.bin"

REM if "%filename%" == "" set filename=out\BGC32Gimbal.USB.bin
set filename="C:\Users\John Ihlein\Desktop\STM32workspace\BGC32\Release\BGC32.bin"

echo taking board into boot mode ...
REM boot loader hack by ala42
REM let dfu-util talk to the USB VCP address. This is detected by the VCP handler
REM which takes the board into the boot loader
%FLASHTOOL% --device 0483:5740 -D dummyname 2>1 >nul
call :sleep 4

echo starting upload ...
echo.
:loop
  %FLASHTOOL% --reset --device 1eaf:0003 --alt 1 --download %filename% && goto :end
  call :sleep 2
  goto :loop

:sleep
  ping -n %1 localhost >nul
  goto :EOF

:end
  pause
