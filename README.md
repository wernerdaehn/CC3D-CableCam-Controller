# CC3D-CableCam-Controller
CC3D and STM Cube based CableCam controller

In order to use the cablecam controller as is, simply download the 

```
  CC3D CableCam Controller\CC3D CableCam Controller\Debug\bin\CC3D CableCam Controller.hex 
```

file to the OpenPilot CC3D board. 

To flash the firmware there are two options, either using the SWD connector and a ST/Link2 programmer or use the hardware bootloader on UART1.

For the first method is quite convenient and recommended, the second requires some prep work
https://wiki.openpilot.org/display/WIKI/How+to+Flash+Bootloaders+with+an+FTDI+Cable

The source code itself contain all libraries required as a CooCox IDE project.
