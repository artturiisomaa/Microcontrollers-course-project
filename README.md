# Microcontrollers course project

## Overview
The program is designed for the Atmega328P microcontroller.
The program implements the following functions:
- If the on/off switch is on, sound is on, and LED brightness is controlled via
  PWM.
- If the switch is off, the system is still powered on but stops operating and
  waits in power-down mode until the switch is again in the on position.
- The frequency of the sound is transmitted to the  external terminal every two
  seconds.
- The potentiometer controls the sound frequency. The frequency can also be
  tuned via an external terminal using characters '+' and '-' (e.g., '+'
  means adding 10 Hz to the frequency).

## Schematic for which the code was created.
![Schematic](images/Schematic.SVG)

