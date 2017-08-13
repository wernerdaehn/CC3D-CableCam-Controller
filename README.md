# CC3D-CableCam-Controller
CC3D and STM Cube based CableCam controller
MCU used is http://www.st.com/en/microcontrollers/stm32f405rg.html

## Goals

## Getting started

### Hardware

### Flashing the firmware

### Initial Setup

### Use

## Detailed Usage

## Development

### Hardware Mapping

* Servo1: Servo Output to the ESC; Connect the ESC to it in order to feed it with valid PPM servo signals; PB0 -> TIM3_CH3
* Servo2: PB1
* Servo3: ESC Output via UART; PA3 -> USART2_RX
* Servo4: ESC Output via UART; PA2 -> USART2_TX
* Servo5&6: 32Bit Quadruple Encoder used for Hall Sensor input; PA0, PA1 -> TIM5_CH1, TIM5_CH2
* LED Status: PB5 (Low = On)
* LED Warn: PB4 (Low = On)
* MainUSART: Receiver input; In SBus Mode: PA9, PA10 -> USART1_TX, USART1_RX; In SumPPM mode: PA10 -> TIM1_CH3

* SPI1: PA4, PA5, PA6, PA7 for MPU-6000 IMU
* SPI3: PA15, PC10, PC11, PC12 for Flash 16MBit and optional RF Module