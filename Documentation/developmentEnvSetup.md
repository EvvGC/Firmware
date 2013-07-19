# Setting Up Build Environment (Windows)

#### *Note: This tutorial was written using Windows 7, so there may be slight differences for your version*

1. 	Download [GNU Tools for ARM Embedded Processors](https://launchpad.net/gcc-arm-embedded) 
	1.	**Save Windows Zip Package (gcc-arm-non...4-win32.zip) to computer**
	1.	Unzip package
	1.	Copy "GNU Tools ARM Embedded" folder out of zip
		*	Assumed copy location for tutorial **C:\Program Files (x86)\GNU Tools ARM Embedded**
2. 	Download [Cygwin](http://cygwin.com/install.html)
	1.	Download setup.exe and run
		* 	Assuming install location of **C:\cygwin** for this tutorial
	2. 	When prompted to select packages make sure you select the devel package (click "Default" so it says "Install")
		*Note: The whole packages is not required but easiest for this writeup this will give you make and git (required), plus much more*
	3.	On next screen check box to "Select required packages"
	4.	Wait for install to finish (insert long coffee/beer break here!)
3.	Download [Eclipse Standard](http://www.eclipse.org/downloads/)
	*	32 and 64 bit version are available
	*	You may need to install a Java [JRE](http://www.oracle.com/technetwork/java/javase/downloads/index.html) (*make sure you select the JRE to download*)
	*	This does not need to be installed just run the exe (I prefer to extract to C:\Program Files\eclipse)
	*	**You must run application now to setup workspace (take note of location) for later steps**
4.	Make copies of necessary .exe files for eclipese
	1.	Open Cygwin
	2.	Navigate to **/cygdrive/c/cygwin/bin**

			$ cd /cygdrive/c/cygwin/bin
	3.	Copy rm to cs-rm
	
			$ cp ./rm ./cs-rm
	4.	Copy make to cs-make
		
			$ cp ./make ./cs-make
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
	
		`;C:\cygwin\bin\;C:\Program Files (x86)\GNU Tools ARM Embedded\4.7 2013q2\bin`
		
		**Note:** *version of GNU Tools ARM may have changed for you (ie. 4.7 2013q2), please verify*
	11.	Click OK
5.	Clone git repo
	1.	In cygwin navigate to your eclipse workspace (selected during Cygwin install)
	2.	Clone Repo
		
			$ git clone https://github.com/EvvGC/Firmware.git
6.	Setup Eclipse
	1.	Move template .cproject file so eclipse will see it for importing
		
			$ cd Firmware
			$ cp setup/.cproject.orig .cproject
	2.	Open Eclipse
	3.	Close Welcome screen
	3.	Install GNU Arm Plugin
		*	See eclipseSetup.pdf steps 3 - 10
	4.	Import Project
		*	See eclipseSetup.pdf steps 12 - 14 Substituting 'Firmware' for 'aq32plus'
	5.	Configure Project
		1.	Ensure build variables are correct.
			1.  In the Project Explorer tab, right click on Firmware, select properties
            2.  Expand C/C++ Build in the left panel
			3.	Select Build Variables in left panel
			4.	Check to make sure GCCPATH and GCCVERSION is correct.
				*	Both these values can be figured by looking at directory structure from download of step 1
				*	GCCPATH will probably need '4.7 2013q2' updated to newer version as this was the version at time of writing
				*	GCCVERSION will probably need to be updated as 4.7.4 was current at time of writing
			7.	Close Properties Panel
7.	Clean Project
	1. 	Select Project from menu bar
	2.	Select Clean...
	3.	Select Clean All Projects and then OK
8.	Build Project
	1.	Select Project
	2.	Select Build All
	
	
## DONE

9.	If you are getting build error because of the USB source. Exclude unneeded source. For "Libraries>STM32\_USB\_Library", "Libraries>STM32\_OTG\_Driver"
	1.	Right click on the directory in the project explorer
	2.  Select Resource Configuration > Exclude from Build...
	3.  Select Debug and Release
	4.  Select OK
	5.  Right click on project in the project Explorer
	6.	Select Index > Rebuild
	
	
