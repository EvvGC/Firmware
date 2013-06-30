EvvGC - Open Source 3 axis gimbal controller
======================
(THIS IS NOT FUNCTION AT THE MOMENT DUE TO SOME CALLS TO PRINTF IN THE SOURCE)

20130629 - initial commit (derived from FW03preB)

    - to build under eclipse, you'll need the gnu toolchain from - https://launchpad.net/gcc-arm-embedded
    - I borrowed a setup documnent from the AQ32PLUS software, you'll have to translate a little bit from it.
      it's in the Documentation folder
    - while the eclipsesetup document in the Documents directory talks about installing the Code Sourcery
      toolchain, ignore that and use the above.  Follow the rest of the document however.
    - where it talks about importing a .cproject file, use the one in the setup folder called .cproject.orig
    
20130629 - added USB libaries and stubbed in the VCP code (NOT FUNCTIONAL)
