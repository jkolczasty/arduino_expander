/**
 * arduino_expander
 *  
 * Author: jkolczasty@gmail.com 
 * License: MIT
 * 
 */

#ifndef EXPANDER_COMMON_H_
#define EXPANDER_COMMON_H_

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SPI.h>


#define DATA_ITEMS 8
#define DATA_ITEM_LENGTH 4
#define DATA_SIZE DATA_ITEMS*DATA_ITEM_LENGTH

#define DATA_CLEAR memset((void *)&data, 0, sizeof(data));

const uint16_t PWM_MAX = 255;

uint8_t data[DATA_SIZE];  // mode, gpio, conf1, conf2
uint16_t values[DATA_ITEMS];

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

int8_t output_interrupt_pin = -1;


uint8_t getPinInterrupt(uint8_t pin)
{
  return 0;

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

void eeprom_load()
{
  EEPROM.get(0, data);
}

void eeprom_save()
{
  EEPROM.put(0, data);
}

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
#ifdef DEBUG      
      Serial.print("CONF: ");
      Serial.print(i);
      Serial.print(" gpio ");
      Serial.print(gpio);
      Serial.println(" OUTPUT ");
#endif
      pinMode(gpio, OUTPUT);
      continue;
    };

    if (mode == 2)
    {
#ifdef DEBUG      
      Serial.print("CONF: ");
      Serial.print(i);
      Serial.print(" gpio ");
      Serial.print(gpio);
      Serial.println(" INPUT ");
#endif
      pinMode(gpio, INPUT);
      // pinMode(gpio, INPUT_PULLUP);
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
#ifdef DEBUG      
      Serial.print("CONF: ");
      Serial.print(i);
      Serial.print(" gpio ");
      Serial.print(gpio);
      Serial.println(" PWM ");
#endif
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
#ifdef DEBUG2
      Serial.print("SET: ");
      Serial.print(i);
      Serial.print(" gpio ");
      Serial.print(gpio);
      Serial.print(" OUTPUT ");
      Serial.println(values[i]);
#endif
      digitalWrite(gpio, values[i]);
      continue;
    };

    //WARNING: most arduino boards supports only 8bit PWM 0-255
    if (mode == 5)
    {
      uint16_t v = min(values[i], PWM_MAX);
#ifdef DEBUG2
      Serial.print("SET: ");
      Serial.print(i);
      Serial.print(" gpio ");
      Serial.print(gpio);
      Serial.print(" PWM ");
      Serial.println((int)v);
#endif
      analogWrite(gpio, (int)v);

      data[idx+2] = v & 0xFF;
      data[idx+3] = v >> 8;

      continue;
    };    
  };
};

#ifdef LED_PIN
uint16_t led_div_count = 0;
uint16_t led = 0;
#define LED_DIV 1000
#endif

#endif