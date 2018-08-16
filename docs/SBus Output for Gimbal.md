# SBus Output

In order to control the gimbal there are multiple options

- A second remote like the one that comes with the Ronin itself.
- A second receiver in case the RC sender supports two receivers being bound to it simultaniously. FrSky Taranis does.
- Split the SBus signal from the receiver.
- CableCam Controller itself does generate the SBus signal for the Gimbal.

The latter is described here. The longterm goal is to enable the cablecam controller fully automatic movements including gimbal. Repeat travel patterns, keep the camera pointing to a POI regardeless of where it is at the moment and things like that.

At the moment however the SBus input channels are mapped to output channels and a SBus signal is created for that.

## Hardware

#### MCU Pins used

| Function                | MCU Pin | MCU Function | Connector Pin | Flip32 F4  |
| ----------------------- | ------- | ------------ | ------------- | ---------- |
| SBus Out (non-inverted) | PC7     | USART6_TX    | FlexIO Pin8   | CH6/TX6/J6 |
| GND                     |         |              | FlexIO Pin1   | GND/J2     |

Attention: **Do not connect the +5V line** with the board! Since both, gimbal and board do provide 5V power, it could cause damage. Would be like connecting two batteries with each other.

#### Pinout

![UART Pins](_images/Flip32_F4_Pins_SBus_Out.jpg)



Because the CC3D Revo does not have an output with a SBus inverter, this needs to be added externally. A search for "SBus Signal Inverter" shows many options, e.g. [this](http://flyingfolk.com/S-BUS-Receiver-Signal-Inverter-Converter-NAZE32-Flip32-SP3-Racing-Flight-Controller).

![UART Pins](_images/Flip32_F4_Pins_SBus_Out_SBus_Inverter.jpg)

## Associated commands

| Command                             | Allowed values         | Description                                                  |
| ----------------------------------- | ---------------------- | ------------------------------------------------------------ |
| \$G                                 |                        | Print the current mapping                                    |
| \$G \<channel 1\> ... \<channel 8\> | 1...16 or 256 to unmap | Set the channel mapping                                      |
| \$n                                 |                        | Note that the neutral range of the receiver input is used for the SBus output as well |
| \$D \<channel 1\> ... \<channel 8\> | integers               | Set the SBus out default values in case they are not mapped  |



## Settings

The Ronin-M expects a (almost) fixed channel assignment, as seen in the DJI Gimbal assistant.

Usually the mapping is:

| Channel | Function | Note                                  |
| ------- | -------- | ------------------------------------- |
| 1       | Tilt     | can be configured in the DJI app      |
| 2       | Roll     | can be configured in the DJI app      |
| 3       |          | not used by the Ronin-M               |
| 4       | Pan      | can be configured in the DJI app      |
| 5       |          | not used by the Ronin-M               |
| 6       | Function | left switch on the Ronin Sender       |
| 7       | Mode     | right hand switch on the Ronin Sender |
| 8       |          | not used by the Ronin-M               |

Note: In the DJI App the channel numbering starts with zero, not one.

Using the \$G command, the RC sender channels can be mapped to any of those for channels. For example the gimbal should be controlled with the left stick, channel 1 used for pan, channel 2 for tilt. The Function switch should be in the middle always ("Normal speed") and the Mode switch on upper ("Free, SmoothTrack Off").

In this case \$G 2 256 256 1 256 256 256 256 would be the correct setting.

| Ronin channel | Ronin | Input channel        | $G value |
| ------------- | ----- | -------------------- | -------- |
| 1             | Tilt  | 2 (stick up/down)    | 2        |
| 2             | Roll  |                      | 256      |
| 3             |       |                      | 256      |
| 4             | Pan   | 1 (stick left/right) | 1        |

As the Function and Mode is not assigned to a RC sender channel, it outputs the default values of neutral. For the Mode switch however it should be set to high. Hence the defaults are changed to

\$D 992 992 992 992 992 992 2000 992

If all needed channels are mapped, then the default settings are never used, e.g. \$G 2 8 256 1 256 6 7 256 would assign all five channels the Ronin-M does use to various channels of the RC sender.
