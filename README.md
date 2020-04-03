
# arduino_expander

Arduino as i2c (TWI) runtime configurable expander

Author: jkolczasty@gmail.com 

License: MIT

## Hardware

Tested on Orange Pi as I2C Master and I2C Slave:
  - Mini Pro (3.3V 8MHz)
  
## Implementation status

Implemented:
  - OUTPUT
  - INPUT
  - INPUT PULLUP
  - PWM

## TODO
  - analog inputs
  - persistent configuration (internal flash memory)
  - interrupts

## USAGE

Right now code supports 2 types of frames: configure and set

### 7 - Configure

To configure send i2c message with first byte as 7 and array of bytes like:

MODE, GPIO, VALUE_LOWBYTE, VALUE_HIBYTE

E.g.:

(7,1,7,1,0,2,8,0,0,5,9,50,0)

Means:
  - 7 - set configuration
  - 1,7,1,0 - set GPIO 7 in mode 1 (OUTPUT) and set initial value 1 - item index 0
  - 2,8,0,0 - set GPIO 8 in mode 2 (INPUT) - item index 1
  - 5,9,50,0 - set GPIO 9 in mode 5 (PWM) with initial duty cycle 50 (50/255) - item index 2
 
Order is important. TO set values you will use item index of configured GPIOs

### 8 - Set values

To set values on output and PWM send i2c message with first byte as 8 and array of bytes like:

ITEM_INDEX, VALUE_LOWBYTE, VALUE_HIBYTE

E.g.:

(8,0,1,0,2,255,0)

Means:
  - 8 - set values
  - 0,1,0 - set item 0 value 1 (item is an index of configured GPIOs, see configuration)
  - 2,255,0 - set item 2 value 255 (PWM, see *Configure*)
  
## Reading values

Just send read request and will respond with config frame with current values.

Example:

1,7,1,0,2,8,1,0,5,9,50,0

Means:
  - 1,7,1,0 - set GPIO 7 in mode 1 (OUTPUT) and current value 1
  - 2,8,1,0 - set GPIO 8 in mode 2 (INPUT) and current value 1
  - 5,9,100,0 - set GPIO 9 in mode 5 (PWM) and current duty cycle 100

