/**
 * arduino_expander
 * 
 *  i2c model
 * 
 * Author: jkolczasty@gmail.com 
 * License: MIT
 * 
 */

#ifdef USE_I2C

#ifndef EXPANDER_I2C_H_
#define EXPANDER_I2C_H_

#include <Arduino.h>
#include <Wire.h>

#include "expander_common.h"

#define I2C_SLAVE_ADDRESS 0x29

volatile uint8_t recvdata[DATA_SIZE]; // reqid, data...
volatile uint8_t recvdata_c = 0;
#define RECVDATA_CLEAR memset((void *)&recvdata, 0, sizeof(recvdata));

void i2c_request()
{
      Wire.write((const uint8_t *)&data, sizeof(data));
      //TODO: OUTPUT_INTERRUPT_CLEAR
}

void i2c_receive(int bc)
{
      uint8_t c = 0;
      uint8_t b;

      RECVDATA_CLEAR
      
      while ((c < bc) && (Wire.available() > 0))
      {
          b = Wire.read();
          if (c < sizeof(recvdata))
          {
            recvdata[c] = b;
          };
          c++;
      };

      if (c<2) {
        return;
      };

      // Configure, 4 bytes data per item
      if (recvdata[0] == 7)
      {
          DATA_CLEAR
          uint8_t count = (c-1)/DATA_ITEM_LENGTH;
          uint8_t idx;
          for (uint8_t i=0; i<count; i++)
          {
#ifdef LED_PIN
            LEDC += 1;
#endif
            idx = i*4;
            data[idx] = recvdata[idx+1];
            data[idx+1] = recvdata[idx+2];
            data[idx+2] = recvdata[idx+3];
            data[idx+3] = recvdata[idx+4];
          };
          configure();
          set();
          eeprom_save();
      };

      // Set values, 3 bytes data per item (item index, value low byte, value high byte)
      if (recvdata[0] == 8)
      {
          uint8_t count = (c-1)/3;
          uint8_t idx;
          uint8_t dataidx;
          for (uint8_t i=0; i<count; i++)
          {
            idx = i*3;
            dataidx = recvdata[idx+1];
            if (dataidx >= DATA_ITEMS)
            {
              continue;
            };
#ifdef LED_PIN
            LEDC += 1;
#endif
            // convert to real array index
            dataidx = dataidx * DATA_ITEM_LENGTH;
            data[dataidx+2] = recvdata[idx+2];
            data[dataidx+3] = recvdata[idx+3];
          };
          set();
      };
}


void setup_expander() {
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onRequest(i2c_request);
  Wire.onReceive(i2c_receive);
}

void loop_expander() {
}

#endif
#endif