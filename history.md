# History #

20130629
    - initial commit (derived from FW03preB)

    - to build under eclipse, you'll need the gnu toolchain from - https://launchpad.net/gcc-arm-embedded
    - I borrowed a setup documnent from the AQ32PLUS software, you'll have to translate a little bit from it.
      it's in the Documentation folder
    - while the eclipsesetup document in the Documents directory talks about installing the Code Sourcery
      toolchain, ignore that and use the above.  Follow the rest of the document however.
    - where it talks about importing a .cproject file, use the one in the setup folder called .cproject.orig

20130629
    - added USB libaries and stubbed in the VCP code (NOT FUNCTIONAL)

20130629
    - added newlib_stubs.c - these are the stub routines that should allow the uarts to function like
      consoles with limited input/output features like printf, etc.

20130711
    - updated the CMSIS library to the latest version from ARM (3.2)
      updated setup/.cproject.orig, confirmed build

20130712
    - updated with latest sources from evvaldis, builds ok, still untested

20130712
    - added the original sources for 0.3b and 0.3e (latest as of this date)

20130716
    - updated with new sprintf_ that is much smaller than original, changed Release build to optimize for size
      added the USB FS Driver (none of the USB drivers are currently used

20130716
    - updated the import .cproject.orig in the setup directory to remove Release optimizations

20130718
    - updated with the EvvGC 0.3g sources, built but not tested

20130721
    - added a minimal set of bin utililies in the setup/minutils.zip file.
      copy these files (rm.exe, cs-rm.exe, make.exe and cs-make.exe to the same location
      as the bin utilities for the compiler that you installed.)

20130801
    - replaced system_stmf10x.c with 3.6.1 version

20130808 major code rework by ala42
  - added Makefile, modified a Makefile that was posted previously in a source code package
  - added message printouts during startup, so you can see what is going on
  - made variable 'stop' volatile
    Previously while (stop == 0) {} was an endless loop due to compiler optimisation
  - added interrupt driven serial I/O using ring buffer

  AUX3/4 handling
  - changed resolution of timer3 from 10탎 to 1탎
  - made variables used for reading AUX3/AUX4 static instead of global
  - added access functions GetAUX3()/GetAUX4() returning pulse length from 0-999탎
  - disabled JTAG to make PB3 connected to AUX3 usable at all, AUX3 is not used in the code, yet
  - fixed variable types of counter helper variables like rc3a from int to unsigned short
  - changed max counter value from 60000 to 65535, as arithmetics in the arithmetic ring
    of the 16-bit counter does not work otherwise.

  - made several variables in engine.c local/static
  - moved several global variable declarations to the files where they belong to
  - added a constrain function to make the code more readable
  - fixed the usage of the AUX4 value so the sinus array is accessed correctly
  - fixed gyro resolution from 8000 to 8192 and made some cleanup in gyro.c

  - changed main loop timing from timer2 interrupt flag to direct timing reads

  - moved command handling from interrupt handler to main loop
  - cleaned up command handling
  - added '+', '-' commands for testing PWM outputs
  - added 'd' command to enable debug print messages
  - added 'G' command to print config values

  - removed motor off during config loop as config is atomic now
  - removed variable stop
  - renamed TimerOff to PWMOff

  - ReadFromEEPROM returns an error checked by configLoad() now, if eeprom is not available
  - MPU6050_Init returns an error now, added retries
  - relaxed watchdog timing, before the watchdog could trigger during gyro calibration

  - fixed several major problems in the dead time handling of TIM4 and TIM5
    - TIM5 active time was reduced by 70 18MHz ticks, had to be 80 to match the dead time of TIM1/8
    - TIM4 active time was not reduced at all by the dead time
    - delay between TIM4 and TIM5 was made with a delay loop when enabling the timer. Now the
      counter start value is set to the wanted delay. Old delay was 0.8탎 instead of 4.4탎.
    - inverted and swapped TIM4+5, as YawX and YawXN were swapped and did not match the Roll/Pitch waveform
    - PWM width is changed syncronously now in interrupt handlers
      This bug caused dead time violations/shortcuts on TIM4/5 before when PWM width change was large
    - activated compare preload mode, so PWM cycles are no longer interrupted when changing the PWM width
      This bug caused bad frequency and shortcuts on TIM4/5 before when PWM width change was large
  - changed BTDR initialisation order for TIM1/8 to remove startup glitch
  - simplyfied timer setup

  - moved rc code from main.c and timer.c to rc.c
  - moved PWM code from timer.c to pwm.c
  - moved and simplyfied PWM code from engine.c to pwm.c
  - removed timer.c

20130810 some fixes by ala42
  - fixed bug in ringbuffer handling
  - fixed problem in micros()
  - added 64MHz clock setup in case external oscilator fails
  - added callback for systick irq
  - removed TIM2 usage

  Performance
  - added performance messages, online selectable with 'p'
  - changed atan2(double) to atan2f(float), which saves ~60탎
  - changed SetPWM to use sinus table and fix point calculation, which saves ~550탎/loop
  - current loop time is ~660탎

20130813
  - increased the I2C delay to be on the safe side
  - revised gyro convertion to use gyroScaleFactor and corrected the math and value
    to match the actual gyro setup

20130819
  PWM handling
  - added profiling to measure when and how often the CCR registers are changed
    Log message is toggled with 'c'.
  - fixed bug causing yaw channel update every cycle without any need
    This bug just wasted time.
  - fixed bug that prevented CCR shadow register update at the planned moment
    The IRQ handler were added before to update the CCR registers early in the cycle.
    They were fired directly because of pending timer overruns, which caused non syncronous
    updates when the update occured at the end of the cycle.
  - added protection to prevent late CCR register updates in case the interrupt was blocked
  - increased priority of these interrupts
  - added phase delay between the three motor axis. Not sure if that helps, but it does not hurt

  - added idle time calculation, printed out in the performance log you get with 'p'
  - added USB-VCP support. Input and output runs in parallel to UART4.

  - changed start up messages, which can be seen on USB-VCP if you connect a terminal program fast enough
  - moved command handler to separate files

20130820
    - added built-versions directory with just the hex files of the various verions
      including the experimental version (both FW and GUI)
    - various usb VCP tweaks,
    - change to the command interpreter to resolve an error when writing values from the GUI

20130821
    - fixed array offset in engine.c for yaw power setting

20130901
  - added basic evvgc board support
  - added mechanism to reset CPU before starting main program to avoid USB setup problems
  - added shared memory communication with main program using backup ram area
  - added "fast reset mode" triggered by main program
  - reduced flash timeouts for 8x faster flashing
  - added some volatile definitions to use -O2 code
  - removed many compiler warnings

    - main program changes for boot loader support
  - added command 'r' to reset CPU
  - added command 'b' to reset CPU and stay in bootloader until program is uploaded

    - other changes
  - added printout of gcc version and firmware build date, looks like
          gcc version 4.7.4 20130613 (release) [ARM/embedded-4_7-branch revision 200083]
          EvvGC firmware V1.01, build date Sep  1 2013 01:08:53
    - added pin definitions for LED and I2C pins to pins.h, changed code to use them

20130903
    - updated the attitude estimation code to support external RX control of both Pitch
      and YAW... AUX3 - Pitch, AUX4 - YAW
    - added support for a roll offset that can be programmed via the GUI
    - added support for AutoPan on the YAW axis
    - other general formatting changes to the code

20130908
    - more cleanup in the attitude estimation engine
    - fixes for tuning the yaw motor

20130910
    - initial release of the 0.4 firmware

20130913
    - usb VCP fix for Windows XP
    - added version command to cli

20130917
    - incremental changes to the attitude engine and replacement of other *magic* numbers
    - release of 0.4.1 firmware

20130929
    - Fixed a stray interrupt problem that was occuring on 1.3 boards only due to the USB pin
      mis configuration
    - correct gyro configuration
    - release of 0.4.2 firmware

