# History #

20130629 - initial commit (derived from FW03preB)

    - to build under eclipse, you'll need the gnu toolchain from - https://launchpad.net/gcc-arm-embedded
    - I borrowed a setup documnent from the AQ32PLUS software, you'll have to translate a little bit from it.
      it's in the Documentation folder
    - while the eclipsesetup document in the Documents directory talks about installing the Code Sourcery
      toolchain, ignore that and use the above.  Follow the rest of the document however.
    - where it talks about importing a .cproject file, use the one in the setup folder called .cproject.orig
    
20130629 - added USB libaries and stubbed in the VCP code (NOT FUNCTIONAL)

20130629 - added newlib_stubs.c - these are the stub routines that should allow the uarts to function like
           consoles with limited input/output features like printf, etc.
           
20130711 - updated the CMSIS library to the latest version from ARM (3.2)
           updated setup/.cproject.orig, confirmed build
           
20130712 - updated with latest sources from evvaldis, builds ok, still untested

20130712 - added the original sources for 0.3b and 0.3e (latest as of this date)

20130716 - updated with new sprintf_ that is much smaller than original, changed Release build to optimize for size
           added the USB FS Driver (none of the USB drivers are currently used   
           
20130716 - updated the import .cproject.orig in the setup directory to remove Release optimizations

20130718 - updated with the EvvGC 0.3g sources, built but not tested