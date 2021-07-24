# PinballXInput
Emulates an Xbox 360 Controller for PC-based Pinball Games.

Using an Arduino Leonardo (resp. Pro Micro clone) and a MPU6050 accelerometer:
* connected over USB to the PC and discovered as Xbox 360 controller
* the flipper left and right are mapped to buttons on GPIOs
* the accelerometer is used to nudge the table

Based on examples/libraries from:
* David Madison (github.com/dmadison/ArduinoXInput)
* Jeff Rowberg (https://github.com/jrowberg/i2cdevlib)
