BGC32
=====

Open source Alternative firmware for EvvGC 3 Axis Gimbal controller

#### NOTICE: NOT FUNCTIONAL!!! - BUILDABLE, currently in TESTING, runs on both 1.2 and 1.3 hardware STM32F1 processor supported currently, F4 version is in the works

#### WARNING: you must *NEVER* flash new firmware over UART1 when you have any form of LIPO connected to the controller. Doing so will damage your controller!!!


#### NOTICE: This source is buildable either via Makefile included or and Eclipse project specifics on the eclipe project will be forthcoming

#### CLI instructions ####
The CLI is accessable over the USB port after you install the ST VCP driver
!!! UART4 IS NOT SUPPORTED at this time!!!

To access the CLI, you connect Putty or other equivalent terminal program to
the controller over the USB VCP port.

To use Putty, you need to configure a connection as follows.
1. start Putty
2. on the session startup screen that pops up
  1. select serial
  2. enter the serial port number
  3. baud doesn't matter
  4. click the "terminal" on the left tree
    1. click "Implicit CR in every LF"
    2. click "local echo "force on""
  5. click the keyboard under terminal
    1. change the backspace to the control-h
  6. go back to the session branch on the left and name and save

Some of the basic 'single letter' commands (these do not require a carriage return)
'Z' - toggles the CLI on and off.  This also toggles the state of the motors from off to whatever
they were prior (default is all off) - for not the motors have to be off while the CLI is active.

'?' - help for the CLI commands

lowercase letters - these are mostly reporting commmands, they do not set parameters...
uppercase letters - these are mostly change commands and they do set parameters.

The format for most parameter setting commands is <command letter><value><semi-colon>..
If there are more than one parameter that needs to be included - ALL parameters
need to be specified, even if they haven't changed.  Also a final <semi-colon> is required
with all parameter setting commands.

#### NOTICE:
BGC32 from FocusFlight, a new alternative firmware
for the EvvGC controller

Original work Copyright (c) 2013 John Ihlein
                                 Alan K. Adamson

This file is part of BGC32.

Includes code and/or ideas from:

  1. BaseFlight
  2. EvvGC
  3. S.O.H. Madgwick

BGC32 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

BGC32 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with EvvGC. If not, see <http://www.gnu.org/licenses/>.
