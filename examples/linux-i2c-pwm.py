#!/usr/bin/python3
#
# arduino_expander pwm example
#

import smbus
import time
import random
    
bus = smbus.SMBus(0)
address = 0x29

# config as follows:
# 7 as OUTPUT as 1 initial output
# 8 as INPUT
# 9 as PWM with 20/255 duty cycle

config = [1, 7, 1, 0, 2, 8, 0, 0, 5, 9, 20, 0]

bus.write_i2c_block_data(address, 7, config)

pwm = 0
pwm_max = 255
dc = 10
while 1:
    pwm += dc
    if pwm > pwm_max:
        pwm = pwm_max
        dc = -dc
    if pwm < 0:
        pwm = 0
        dc = -dc
    
    # set pwm
    data = [2, pwm % 256, int(pwm/256)]
    bus.write_i2c_block_data(address, 8, data)
    time.sleep(0.05)
