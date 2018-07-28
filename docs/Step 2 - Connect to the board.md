# Connecting to the Controller

The CableCam Controller firmware was uploaded and either in the DFUSe tool the _Leave DFU Mode_ button was pressed or the board was connected without the boot button being pressed. 

1. As the board is connected to the computer via USB, a new COM port is available to interact with the CableCam Controller.
1. This can be validated by starting the Windows 10 Device Manager, it should have a new com port called _STMicroelectronics Virtual COM Port or _USB Serial Port_. The port name in brackets, here _COM3_ is the one to use in the terminal. 
<a href="https://raw.githubusercontent.com/wernerdaehn/CC3D-CableCam-Controller/master/_images/WindowsDeviceManager.jpg"><img src="../_images/WindowsDeviceManager.jpg" height="100px"/></a>
1. Open a serial terminal in order to talk to the board. The one I use is [putty](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html) as 64Bit install or just the 64Bit exe download.
1. In the terminal create a new serial connection to the port shown in the device manager. In putty that means clicking on _serial_ and entering the above portname like _COM3_. The settings for baud rate etc are irrelevant and can be left at their defaults.
1. To test if everything works properly, a first command can be sent by typing _$h_ for help and confirming the command with the return key.
1. This should print status information plus a help text in the console.
  <a href="https://raw.githubusercontent.com/wernerdaehn/CC3D-CableCam-Controller/master/_images/console_help.png"><img src="../_images/console_help.png" height="100px"/></a>



