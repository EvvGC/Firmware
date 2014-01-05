# EvvGC - Edited by me to be more suitable for a potentiometer RC input #
# The pitch offset angle is now directly proportional to the RC input, instead of accumulating #

# EvvGC - Open Source 3 axis gimbal controller #

#### NOTICE: BUILDABLE, currently in TESTING, runs on both 1.2 and 1.3 hardware

Port from initial Keil source to Eclipse of firmware for EvvGC 3/2axis Brushless Gimbal Controller

## Master branch, firmware v0.4 requires the use of GUI 0.4 ##
See GUI directory for latest version

#### GUI instructions ####
In order to convert from the 0.3x firmware to the 0.4 firmware some eeprom values
have to be changed to support this version and you need to accomplish the following

- disconnect motors
- Load the new firmware via UART1 (remember NO LIPO power!!!)
- disconnect UART1 and connect your usb/serial adapter to UART4
- with motors disconnected, load the new GUI 0.4 via the GUI directory above
- click the config on button
- then read the values from the eeprom
- write these old values down incase you need to revert back!!!
- you'll need to adjust only the "P" values.  Take what is read and multiplying each by 10
  e.g. .12 would be 1.20
- lastly adjust the bottom right value in the gui that sets any roll offset to 0.0
- write the new parameters
- click read and verify that everything was written correctly
- disconnect your usb/serial adapter from UART4
- connect motors and power up, and use as you wish

## Setting Up Windows Build Environment ##

You may setup your development environment using Cygwin (terminal emulator for windows) by follow the
tutorial Documentation/devEnvSetupEclipse.md

#### OR ####

You may setup your development environment in a more simple manner, not needing the full Cygwin by
following the tutorial Documentation/devEnvSetupNoCygwin.md

Original work Copyright (c) 2012 [Evaldis - RCG user name]
Modified work Copyright 2012 Alan K. Adamson

This file is part of EvvGC.

EvvGC is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

EvvGC is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with EvvGC.  If not, see <http://www.gnu.org/licenses/>.