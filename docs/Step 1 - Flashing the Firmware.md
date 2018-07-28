# Flashing the Firmware

_Note: All drivers used are included in a standard Windows 10 installation._
1. Install the ST provided DfuSe utility from the bottom of [this page](http://www.st.com/en/development-tools/stsw-stm32080.html).
1. Run the installed DfuSe program.
1. Connect the board to your computer via USB and while doing so, keep the _boot_ button pressed. When the board is powered with the boot button pressed, the STM32 internal USB bootloader is started instead of the firmware.
1. As this activates the STM32F4 hardware bootloader and no firmware runs, only the Power LED should be on. If the Status LED does blink, the firmware is active. Try again above step.
1. Download the firmware from this project https://github.com/wernerdaehn/CC3D-CableCam-Controller/blob/master/bin/Debug/CableCamControllerF4.dfu
1. At the bottom of the utility, in the _Upgrade or Verify Action_ area, click on _Choose_ and select above's downloaded CableCamControllerF4.dfu file. 
<a href="https://raw.githubusercontent.com/wernerdaehn/CC3D-CableCam-Controller/master/_images/dfuse_choose.jpg"><img src="../_images/dfuse_choose.jpg" height="100px"/></a>
1. The DfuSe Utility should show in the top box the text _STM Device in DFU Mode_. This indicates the board's hardware bootloader is running. 
1. If it does, the _Upgrade_ button copies the firmware onto the board. 
<a href="https://raw.githubusercontent.com/wernerdaehn/CC3D-CableCam-Controller/master/_images/dfuse_upgrade.jpg"><img src="../_images/dfuse_upgrade.jpg" height="100px"/></a>
1. The next dialog(s) is to be confirmed with _yes_. We are certain the firmware is for the STM32F405RG chip.
1. To validate the flashing was successful, the _Verify_ action should be triggered, just to doublecheck.
1. At the top is the _Leave DFU mode_ button to restart the controller with the new firmware.
1. A first indication that everything is normal is when the Status LED on the board does flash with 1Hz.



Just to clarify. This is a new firmware. All the previous installed code is removed by that procedure from the board, no software bootloader, no previous firmware. So connecting this board to CC3D Revo software like the Ground Station will no longer work. You would need above method to reinstall the software bootloader of OpenPilot.
