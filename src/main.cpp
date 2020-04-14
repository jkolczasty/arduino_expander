/**
 * arduino_expander
 * 
 *  Simple usage of arduino boards (like Mini Pro 3.3V 8MHz) as runtime configurable I2C/SPI/SERIAL expander (output, input, input_pullup, PWM)
 * 
 *  Still TODO:
 *    - SPI model - can't figure it out how to communicate with e.g. orange pi
 *    - analog inputs
 *    - persistent configuration (internal flash memory)
 *  
 * Author: jkolczasty@gmail.com 
 * License: MIT
 * 
 */

#include <Arduino.h>

// #define DEBUG 1
// #define DEBUG2 1

#define LED_PIN 13  // mini pro

#include "expander_common.h"

#define USE_SERIAL

#include "expander_i2c.h"
#include "expander_spi.h"
#include "expander_serial.h"


void setup() {
#ifdef DEBUG  
  Serial.begin(115200);
  delay(10);
  Serial.println("INIT");
#endif

#ifdef LED_PIN
  pinMode (LED_PIN, OUTPUT);
#endif

  eeprom_load();
  configure();

  //
  setup_expander();
#ifdef DEBUG
  Serial.println("ready");
#endif
}


void loop() {

  loop_expander();

  uint8_t idx;
  // TODO: implement analog read
  for (uint8_t i=0; i<DATA_ITEMS; i++)
  {
    idx = i*4;
    if ((data[idx]==2) || (data[idx]==3))
    {
      uint8_t v = digitalRead(data[idx+1]);
      if (v != values[i])
      {
#ifdef LED_PIN
        LEDC++;
#endif
        values[i] = v;
#ifdef EXPANDER_ON_CHANGE
        expander_on_change(i);
#endif
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
