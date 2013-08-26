# Setting Up Build Environment (Windows)

#### *Note: This tutorial was written using Windows 7, so there may be slight differences for your version*

1. 	Download [GNU Tools for ARM Embedded Processors](https://launchpad.net/gcc-arm-embedded) 
	1.	**Save Windows Zip Package (gcc-arm-non...4-win32.zip) to computer**
	1.	Unzip package
	1.	Copy "GNU Tools ARM Embedded" folder out of zip archive
		*	Assumed copy location for tutorial **C:\Program Files (x86)\GNU Tools ARM Embedded\**
2.	Install [GNU Make for Windows](http://gnuwin32.sourceforge.net/packages/make.htm)
	1.	Download "Complete package, except sources"
	2.	Run Installer
	3.	Select default install location **C:\Program Files (x86)\GnuWin32**
	4.	Under Select Components select either "Full Installation" or "Compact Installation" as both contain binaries needed
	5.	Select defaults for the rest of the installer and finish installation
3.	Install [GNU CoreUtils for Windows](http://gnuwin32.sourceforge.net/packages/coreutils.htm)
	1.	Download "Complete package, except sources"
	2.	Run Installer
	3.	Select default install location **C:\Program Files (x86)\GnuWin32**
	4.	Under Select Components select either "Full Installation" or "Compact Installation" as both contain binaries needed
	5.	Select defaults for the rest of the installer and finish installation
4.	Make copies of necessary .exe files for eclipse
	1.	Open **C:\Program Files (x86)\GnuWin32\bin** in Windows Explorer
	2.	Rename make.exe to cs-make.exe
	3.	Rename rm.exe to cs-rm.exe
	
	**NOTE: you may not see the .exe in your window if you have extensions hidden (default)**
5.	Update PATH Variable
	1. 	Select Start
	2.	Right Click on Computer
	3.	Select Properties
	4.	Select Advanced System Settings
	5.	Select Advanced Tab
	6.	Select Environment Variables...
	7.	In the System Variables pane scroll down and select PATH
	8.	Select Edit
	9.	Scroll to end of text
	10. Insert the follow at the END of the current path
	
		`;C:\Program Files (x86)\GnuWin32\bin;C:\Program Files (x86)\GNU Tools ARM Embedded\4.7 2013q2\bin`
		
		**Note:** *version of GNU Tools ARM may have changed for you (ie. 4.7 2013q2), please verify*
	11.	Click OK
5.	Get source code
	1.	Download [Firmware Source Code](https://github.com/EvvGC/Firmware/archive/master.zip)
	2.	Extract zip to Eclipse Workspace
		*	Directory Structure will look something like C:\Users\{Username}\workspace\Firmware-master once extracted
6.	Download [Eclipse Standard](http://www.eclipse.org/downloads/)
	*	32 and 64 bit version are available
	*	You may need to install a Java [JRE](http://www.oracle.com/technetwork/java/javase/downloads/index.html) (*make sure you select the JRE to download*)
	*	This does not need to be installed just run the exe (I prefer to extract to C:\Program Files\eclipse)
	*	**You must run eclipse now to setup workspace (take note of location) for later steps**
7.	Setup Eclipse
	1.	Move template .cproject file so Eclipse will see it for importing
		1.	Open Command Prompt and navigate to git repository directory from last step
			*	Click Start
			*	type "cmd" in search box to open DOS prompt
			*	cd to workspace directory
				
				`> cd workspace`
			*	cd into extracted source directory
			
				`> cd Firmware-master`
				
		1.	Move setup/.cproject.org to .cproject
		
			`> copy setup/.cproject.orig .cproject`
	2.	Open Eclipse
	3.	Close Welcome screen
	3.	Install GNU Arm Plugin
		*	See eclipseSetup.pdf steps 3 - 10
	4.	Import Project
		*	See eclipseSetup.pdf steps 12 - 14 Substituting 'Firmware-master' for 'aq32plus'
	5.	Configure Project
		1.	Ensure build variables are correct.
			1.  In the Project Explorer tab, right click on Firmware-master, select properties
            2.  Expand C/C++ Build in the left panel
			3.	Select Build Variables in left panel
			4.	Check to make sure GCCPATH and GCCVERSION is correct.
				*	Both these values can be figured by looking at directory structure from download of step 1
				*	GCCPATH will probably need '4.7 2013q2' updated to newer version as this was the version at time of writing
				*	GCCVERSION will probably need to be updated as 4.7.4 was current at time of writing
			7.	Close Properties Panel
8.	Clean Project
	1. 	Select Project from menu bar
	2.	Select Clean...
	3.	Select Clean All Projects and then OK
9.	Build Project
	1.	Select Project from the menu bar
	2.	Select Build All
	3.	Verify success by seeing "Finished building: EvvGC.hex" as the third to last line of the Console output on the lower right of eclipse.
	
	
## DONE

10.	If you are getting build error because of the USB source. Exclude unneeded source. For "Libraries>STM32\_USB\_Library", "Libraries>STM32\_OTG\_Driver"
	1.	Right click on the directory in the project explorer
	2.  Select Resource Configuration > Exclude from Build...
	3.  Select Debug and Release
	4.  Select OK
	5.  Right click on project in the project Explorer
	6.	Select Index > Rebuild
	
	
