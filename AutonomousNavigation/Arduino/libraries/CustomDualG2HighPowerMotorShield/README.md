# Modified Arduino library for the Pololu Dual G2 High Power Motor Driver Shields.
## Modified by Richard Cortez

## Summary

This is a library for the Arduino IDE that interfaces with the Pololu
[Dual G2 High Power Motor Driver Shield 18v18](https://www.pololu.com/catalog/product/2515),
[Dual G2 High Power Motor Driver Shield 24v14](https://www.pololu.com/catalog/product/2516),
[Dual G2 High Power Motor Driver Shield 18v22](https://www.pololu.com/catalog/product/2517),
and [Dual G2 High Power Motor Driver Shield 24v18](https://www.pololu.com/catalog/product/2518).
Modified to make it simple to drive four brushed, DC motors connected to two same motor driver shields.

## Getting started

### Hardware

The
[Dual G2 High Power Motor Driver Shields](https://www.pololu.com/category/218/pololu-dual-g2-high-power-motor-driver-shields)
can be purchased from Pololu's website.  Before continuing, careful
reading of the product page as well as the
[product user's guide](https://www.pololu.com/docs/0J72) is
recommended.

### Software Installation

1. Download the modified library from this Github.
2. Make sure the folder is named "CustomDualG2HighPowerMotorShield".
3. Drag the "CustomDualG2HighPowerMotorShield" folder into the "libraries"
   directory inside your Arduino sketchbook directory.  You can view
   your sketchbook location by opening the "File" menu and selecting
   "Preferences" in the Arduino IDE.  If there is not already a
   "libraries" folder in that location, you should make the folder
   yourself.
4. After installing the library, restart the Arduino IDE.

## Documentation

This library defines a `DualG2HighPowerMotorShield` base class that implements
commands common to all versions of the Dual G2 High Power Motor Driver Shields.
Four derived classes,`DualG2HighPowerMotorShield24v14`,
`DualG2HighPowerMotorShield18v18`, `DualG2HighPowerMotorShield24v18`,
and `DualG2HighPowerMotorShield18v22`, handle commands specific to each version.

### Library Reference

#### DualG2HighPowerMotorShield
Note: # signifies the motor numbers which the command will designate to. Can be values from 1-4.

- `DualG2HighPowerMotorShield()`<br> Default pins, selects the
  default pins as connected by the motor shield.
- `DualG2HighPowerMotorShield(unsigned char M1nSLEEP, unsigned char
  M1DIR, unsigned char M1PWM, unsigned char M1nFAULT, unsigned char
  M1CS, unsigned char M2nSLEEP, unsigned char M2DIR, unsigned char
  M2PWM, unsigned char M2nFAULT, unsigned char M2CS)` <br> Alternate
  constructor for shield connections remapped by user. If M1PWM and
  M2PWM are remapped, it will try to use analogWrite instead of
  timer1.
- `void init()` <br> Initialize pinModes and timer1.
- `void setM#Speed(int speed)` <br> Set speed and direction for motor 1-4.
  Speed should be between -400 and 400.  400 corresponds to motor
  current flowing from M1A to M1B.  -400 corresponds to motor current
  flowing from M1B to M1A.  0 corresponds to full coast.
- `unsigned char getM#Fault()` <br> Returns 1 if there is a fault on motor
  drivers, 0 if no fault.
- `void flipM#(bool flip)` <br> Flip the direction meaning of the speed
  passed to the setSpeeds function for motor 1-4. The default direction
  corresponds to flipM#(false) having been called.
- `void enableDrivers()` <br> Enables the MOSFET drivers for motor 1-4.
- `void disableDrivers()` <br> Puts the MOSFET drivers for motor 1-4 into low-power sleep mode.
- `unsigned int getM#CurrentReading()` <br> Returns current reading from
  motor 1-4.
- `void calibrateCurrentOffsets()` <br> Records the voltage offsets of the
  current readings from motor 1-4 when speeds are 0.
- `unsigned int getM#CurrentMilliamps(int gain)` <br> Returns current
  reading for motor 1-4 in milliamps.  Gain (mV/A) can be specified for a
  specific version of the shield.

#### DualG2HighPowerMotorShield24v14, DualG2HighPowerMotorShield18v18, DualG2HighPowerMotorShield24v18, DualG2HighPowerMotorShield18v22

### Current readings

The current readings returned by `getM#CurrentMilliamps` will be noisy and unreliable if you are using
a PWM frequency below about 5&nbsp;kHz.  We expect these readings to
work fine if you are using a board based on the ATmega168, ATmega328P,
or ATmega32U4, since this library uses 20&nbsp;kHz hardware PWM on
those boards.

On other boards, this library uses `analogWrite` to generate PWM
signals, which usually means that the PWM frequency will be too low to
get reliable current measurements.
