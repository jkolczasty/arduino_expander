#!/usr/bin/python3
#
# arduino_expander input/output example
#

import smbus
import time
import random
    
bus = smbus.SMBus(0)
address = 0x29

# config as follows:
# 7 as OUTPUT as 1 initial output
# 8 as INPUT

config = [1, 7, 1, 0, 2, 8, 0, 0]

def print_gpio(data):
    print("GPIOs:", end="")
    for i in range(0, int(len(data)/4)):
        
        print(" | ", data[i*4+1], "=", data[i*4+2] + data[i*4+3]*256, end="")
    print("")

bus.write_i2c_block_data(address, 7, config)

v = 1
while 1:
    time.sleep(0.5)

    v = 0 if v else 1

    data = [0, v, 0]
    # SET gpio item 0 to value
    bus.write_i2c_block_data(address, 8, data)
    
    # read values
    data = bus.read_i2c_block_data(address, 0)
    print_gpio(data)
