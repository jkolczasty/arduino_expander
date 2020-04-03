/**
 * arduino_expander
 * 
 *  Simple usage of arduino boards (like Mini Pro 3.3V 8MHz) as runtime configurable i2c expander (output, input, input_pullup, PWM)
 * 
 *  Still TODO:
 *    - analog inputs
 *    - persistent configuration (internal flash memory)
 *  
 * Author: jkolczasty@gmail.com 
 * License: MIT
 * 
 */

#include <Arduino.h>
#include <Wire.h>

#define LED_PIN 13  // mini pro
#define I2C_SLAVE_ADDRESS 0x29

#define DATA_ITEMS 8
#define DATA_ITEM_LENGTH 4
#define DATA_SIZE DATA_ITEMS*DATA_ITEM_LENGTH

#define RECVDATA_CLEAR memset((void *)&recvdata, 0, sizeof(recvdata));

const uint16_t PWM_MAX = 1023;

uint8_t recvdata[DATA_SIZE];
uint8_t data[DATA_SIZE];  // mode, gpio, value1, value2

// TODO: test and implement interrupts - so far had no time to do so as not all GPIOs have (working) interrupts
//       and I don't really know much about them
//     
uint8_t PIN_INTERRUPTS[][2] = {
  {7, PCINT23},
  {8, PCINT0},
  {9, PCINT1}
};

const uint8_t PIN_INTERRUPTS_COUNT = sizeof(PIN_INTERRUPTS) / sizeof(PIN_INTERRUPTS[0]);

/*
  GPIO MODES (config):
    0 none
    1 output gpio
    2 input gpio
    3 input pullup gpio
    4 analog input gpio
    5 pwm
    255 change signal / interrupt
*/

#ifdef LED_PIN
uint8_t LEDC=0;
#endif

uint8_t getPinInterrupt(uint8_t pin)
{
  for (uint8_t i=0; i<PIN_INTERRUPTS_COUNT; i++)
  {
    if (PIN_INTERRUPTS[i][0] == pin)
    {
      return PIN_INTERRUPTS[i][1];
    };
  };
  return 0;
}

void digitalPinInterrupt()
{
  uint8_t v;
  uint8_t idx;

#ifdef LED_PIN
  LEDC++;
#endif

  for (uint8_t i=0; i<8; i++)
  {
    idx = i*4;
    if ((data[idx]==2) || (data[idx]==3))
    {
      v = digitalRead(data[idx+1]);
      if (v!= data[idx+2])
      {
        data[idx+2] = v;
      };
    };
  };
};

void configure()
{
  uint8_t idx;
  uint8_t mode;
  uint8_t gpio;
  uint8_t interrupt;
   
  for (uint8_t i=0; i<8; i++)
  {
    idx = i*4;
    mode = data[idx];

    if (mode == 0)
    {
      continue;
    };

    gpio =data[idx + 1];

    if (mode == 1)
    {
      pinMode(gpio, OUTPUT);
      continue;
    };

    if (mode == 2)
    {
      pinMode(gpio, INPUT);
      interrupt = getPinInterrupt(gpio);
      if (interrupt)
      {
        attachInterrupt(interrupt, digitalPinInterrupt, CHANGE);
      };
      continue;
    };

    if (mode == 3)
    {
      pinMode(gpio, INPUT_PULLUP);
      interrupt = getPinInterrupt(gpio);
      if (interrupt)
      {
        attachInterrupt(interrupt, digitalPinInterrupt, CHANGE);
      };
      continue;
    };
    
    if (mode == 4)
    {
      pinMode(gpio, INPUT);
      // attachInterrupt(digitalPinToInterrupt(gpio), digitalPinInterrupt, CHANGE);
      continue;
    };

    if (mode == 5)
    {
      pinMode(gpio, OUTPUT);
      
      interrupt = getPinInterrupt(gpio);
      if (interrupt)
      {
        attachInterrupt(interrupt, digitalPinInterrupt, CHANGE);
      };
      continue;
    };    
  };
};

void set()
{
  uint8_t idx;
  uint8_t mode;
  uint8_t gpio;
   
  for (uint8_t i=0; i<8; i++)
  {
    idx = i*4;
    mode = data[idx];

    if (mode == 0)
    {
      continue;
    };

    gpio =data[idx + 1];

    if (mode == 1)
    {
      digitalWrite(gpio, data[idx+2]);
      continue;
    };

    //WARNING: most arduino boards supports only 8bit PWM 0-255
    if (mode == 5)
    {
      uint16_t v = min(data[idx+2] + ((uint16_t)(data[idx+3]) << 8), PWM_MAX);

      analogWrite(gpio, (int)v);

      data[idx+2] = v & 0xFF;
      data[idx+3] = v >> 8;

      continue;
    };    
  };
};

void i2c_request()
{
      Wire.write((const uint8_t *)&data, sizeof(data));
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


void setup() {
#ifdef LED_PIN
  pinMode (LED_PIN, OUTPUT);
#endif
  // TODO: read from EEPROM

  configure();
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onRequest(i2c_request);
  Wire.onReceive(i2c_receive);
}

#ifdef LED_PIN
uint16_t led_div_count = 0;
uint16_t led = 0;
#define LED_DIV 10
#endif

void loop() {
  uint8_t idx;
  // TODO: implement analog read
  for (uint8_t i=0; i<DATA_ITEMS; i++)
  {
    idx = i*4;
    if ((data[idx]==2) || (data[idx]==3))
    {
      uint8_t v = digitalRead(data[idx+1]);
      if (v != data[idx+2])
      {
#ifdef LED_PIN
        LEDC++;
#endif
        data[idx+2] = v;
      };
    };
  };

#ifdef LED_PIN
  led_div_count++;
  if (led_div_count>=LED_DIV)
  {
    led_div_count = 0;

    if (led)
    {
      led = 0;
      digitalWrite(LED_PIN, 0);
    } else if (LEDC) {
      led = 1;
      LEDC = (LEDC - 1) % 5;
      digitalWrite(LED_PIN, 1);
    };
  };
#endif
}
