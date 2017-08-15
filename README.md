# CC3D-CableCam-Controller
A STM32 based CableCam Controller
MCU used is [STM32F405RG](http://www.st.com/en/microcontrollers/stm32f405rg.html)
Board used: Any CC3D Revolution or clone, e.g. F4 Advanced Flight Controller by Bob Forooghi [example board](http://www.getfpv.com/f4-advanced-flight-controller.html)

## Quickstart
1. Connect the ESC to the Servo1 pins. This will later power the board as well. ESC should be turned off for now.
1. Connect the receiver (SumPPM or SBus) to the Main USART, the pin called SBus RX1 on above F4 controller
1. Provide power to the receiver by connecting its Vcc and Gnd to Servo5 Vcc and Gnd
1. Hall sensor board is powered by Servo6 Vcc and Gnd and its two input signals are connected to Servo5 and Servo6 signal pins
1. Install the ST provided DfuSe utility from the bottom of [this page](http://www.st.com/en/development-tools/stsw-stm32080.html)
1. Run the installed DfuSe program
1. Connect the board to your computer via USB and while doing so, keep the boot button pressed
1. As this activates the STM32F4 hardware bootloader and no firmware runs, only the orange power LED should be on. If the blue Status LED does blink, the firmware is active. Try again above step.
1. Download the firmware from this project https://github.com/wernerdaehn/CC3D-CableCam-Controller/blob/master/bin/Debug/CableCamControllerF4.dfu
1. Flash the firmware following this guide - starting with Step 4: http://kiss.flyduino.net/dwkb/flash-fc-pc/
1. ...

## Goals
The most simple way to control a cablecam is by connecting the RC receiver to the motor controller (=ESC, speed controller) and control it like a RC car.
But this has multiple limitations the CableCam Controller tries to solve:
1. The CableCam might crash into the start- or endpoint by accident. Would be much nicer if the CableCam calculates the required braking distance constantly and does engage the brake automatically. This way it stops automatically no matter of the user input.
1. A smooth acceleration/deceleration makes the videos look better. Even when the stick is pushed forward at once, the CableCam should accelerate slowly instead of the wheel slipping over the rope.
1. A speed limiter to protect the CableCam from going too fast and for constant speed travels during filming.
1. Direct positional control. Normally the stick controls the thrust. But the thrust might mean different things, e.g. a thrust of zero could make the CableCam accelerate downhill. The stick of the RC sender should rather control the speed - stick in neutral means the CableCam should stop.
1. Preprogram movement patterns and the Cablecam repeats them on request.

To achieve that the CableCam controller sits between the receiver and the motor controller and acts as a governour of the receiver input. If for example the user did push the stick forward from neutral to max within a second, the CableCam Controller rather increases the stick position slowly. For speed and positional input the controller is connected to two hall sensors on one of the running wheels.

<a href="https://raw.githubusercontent.com/wernerdaehn/CC3D-CableCam-Controller/master/_images/HallSensor.jpg">
  <img src="_images/HallSensor.jpg" width="50%"/>
</a>

## Getting started

### Hardware
Instead of using a custom build PCB, the project is based on easy to get hardware but flashed with a new firmware. 
This PCB should be small, have an onboard power regulator suited for RC-receiver typical voltages, servo connectors for power in and ESC output. It should also be based on a STM32F4 chip for easier flashing of the firmware - no bootloader needed - and include the needed hardware inverter for SBus signals. The Flight Controllers developed for Copters are an obvious choice.
In particular the [CC3D Revolution board](http://opwiki.readthedocs.io/en/latest/user_manual/revo/revo.html) is a perfect match. The only drawback of the CC3D Revo is that it contains additional hardware like a magnetometer, a barometric height sensor and an expensive RF link - nothing useful for a CableCam. But luckily there a clones of this board without these components, making the board cheaper as well.

_Note: The STM32F4 allows very flexible remapping of the pins to different functions, hence what is supposed to be used as output in the CC3D Revo firmware is rather used as input with the CableCam Controller firmware occasionally._

<a href="https://raw.githubusercontent.com/wernerdaehn/CC3D-CableCam-Controller/master/_images/CC3D_Revolution_Schematics.png">
  <img src="_images/CC3D_Revolution_Schematics.png" width="100%" />
</a>

As said, the board is put between the ESC and the receiver and gets the positional input from the hall sensors in addition.

<a href="https://raw.githubusercontent.com/wernerdaehn/CC3D-CableCam-Controller/master/_images/Hardware_Overview.jpg">
  <img src="_images/Hardware_Overview.jpg" width="50%"/>
</a>

The board can be powered via two methods, either via USB or via one of the Vcc_unreg pins on the Servo connectors. The USB power is applied to the board electronics only, hence is useful for flashing and configuration but not for powering the receiver. Also USB does not provide enough amps for the receiver. That is the reason the receiver and the hall sensors are not powered by the +5V pin but via the Servo out pins.
As the hall sensor is connected to Servo5 and Servo6 (order does not matter - can be swapped in the firmware) but needs Vcc and Gnd just once, the Reciver power cables are soldered to Servo5 Vcc and Gnd.
In other words, as in the normal case the ESC provides power to the receiver, just not directly but with the CC3D Revo board inbetween and hence powering it and all other external components as well.

_Note: All types of receivers are connected to the SBus/RX1 pin. The PPM pin is not used yet, not even for PPM receivers. The main reason is to make sure only one receiver can be connected._

<a href="https://raw.githubusercontent.com/wernerdaehn/CC3D-CableCam-Controller/master/_images/Board_with_Cables.jpg">
  <img src="_images/Board_with_Cables.jpg" width="25%"/>
</a>

#### Hall sensor
The hall sensor is very simple. The [Allegro Microsystems A1120](http://www.allegromicro.com/en/Products/Magnetic-Digital-Position-Sensor-ICs/Hall-Effect-Unipolar-Switches/A1120-1-2-5.aspx) sensor connects the output to Gnd if the magnetic field perpenticular to the chip surface exceeds a certain level and opens that switch if it falls below a level (=Open Drain). These two levels are different, hence avoiding noise in case the magnet field is exaclty at the switch level (=Hysteresis). As a result the sensor board can be very simple. Power between 3..24V is provided - hence perfectly suited for the Vcc_unreg provided by the ESC - and a capacitor nearby the chips is needed. The output signal of both is connected directly to the Servo5&6. A pullup resistor is not needed either, the STM32 MCU internal ones are turned on for those two pins.

<a href="https://raw.githubusercontent.com/wernerdaehn/CC3D-CableCam-Controller/master/_images/HallSensor_Board.png">
  <img src="_images/HallSensor_Board.png" width="50%"/>
</a>

The only important things to make sure are
1. The hall sensor does switch on and off reliably, meaning the magnetic field has to raise above and fall below the thresholds for sure.
1. The two hall sensors have a delay between switching on/off as this delay tells the rotation direction. If both would switch on exactly the same time, no rotation direction can be derived.

To achieve that one of the running wheels has 22 bores of d3x8mm (3mm drill and 8mm deep) and magnets are inserted alternating north/south. The hall sensor does switch on north magnetic fields only and therefore by adding a south orientated magnet between, it is an absolute certainty, the north field strength falls below the required level. 
In my case the bores are drilled into the skate wheel with a diameter of 60mm, hence the distance between each drill is 8.6mm. So as long as the distance between the two hall sensors is not 17.2mm or multiples thereof, they will not switch on/off at the same time. I aimed for a distance of 8.6mm+50% = 13mm roughly.

_Note: The sensor board PCB is currently redesigned using above guidelines. Maybe a LEDs are added as well as visual indication?_


### Flashing the firmware

### Initial Setup

### Use

## Detailed Usage

## Development

### Hardware Mapping

Connector Pin | Description | MCU Pin | MCU function
------------- | ----------- | ------- | ------------
Servo1 | Servo Output to the ESC; Connect the ESC to it in order to feed it with valid PPM servo signals | PB0 | TIM3_CH3
Servo2 | Servo Output (not used yet) | PB1 | TIM3_CH4 
Servo3 | ESC Output via UART | PA3 | USART2_RX
Servo4 | ESC Output via UART | PA2 | USART2_TX
Servo5 | 32Bit Quadruple Encoder used for Hall Sensor input | PA0 | TIM5_CH1
Servo6 | 32Bit Quadruple Encoder used for Hall Sensor input | PA1 | TIM5_CH2
LED Status | Status LED on the boards | PB5 (Low = On) | GPIO
LED Warn | Warn LED on the board | PB4 (Low = On) | GPIO
MainUSART | Receiver input; In SBus Mode | PA10 | USART1_RX
MainUSART | Receiver input; In SBus Mode | PA10 | TIM1_CH3
SPI1 | SPI for MPU-6000 IMU | PA4, PA5, PA6, PA7 | SPI1
SPI3 | SPI for Flash 16MBit and optional RF Module (not used) | PA15, PC10, PC11, PC12 | SPI3
