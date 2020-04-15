# RFM69_RPi
RFM69 library in C for Raspberry Pi

## This library is currently untested

This is a port of the RFM69 library for arduino from https://github.com/LowPowerLab/RFM69 to C for raspberry pi.

## Description

This is a port of the RFM69 library for arduino from https://github.com/LowPowerLab/RFM69 to python for raspberry pi.

## Hardware setup

Attach the RFM69 as follows:

| RFM pin | Pi pin  
| ------- |-------
| 3v3     | 17  
| DIO0    | 18 (GPIO24)  
| MOSI    | 19  
| MISO    | 21  
| CLK     | 23  
| NSS     | 24  
| Ground  | 25  
| RESET   | 29

You can change the interrupt pin (GPIO24) and the reset pin in the initialisation.

Remember to choose correct frequency for your hardware (315, 433, 868 or 915 MHz).

## Prerequisites

wiringPi