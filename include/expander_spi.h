/**
 * arduino_expander
 *  
 *  spi model
 * 
 * Author: jkolczasty@gmail.com 
 * License: MIT
 * 
 */

#ifdef USE_SPI

#ifndef EXPANDER_SPI_H_
#define EXPANDER_SPI_H_

#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <expander_common.h>

#define RECVFRAME_CLEAR memset((void *)&recvframe, 0, sizeof(recvframe)); recvframe_c = 0; recvframe_ready = 0;
#define SENDFRAME_CLEAR memset((void *)&sendframe, 0, sizeof(sendframe)); sendframe_c = 0;
#define SENDFRAME_SERIAL Serial.print("SEND: "); Serial.print(sendframe[0]); Serial.print(sendframe[1]); Serial.print(sendframe[2]); Serial.print(sendframe[3]); Serial.println(sendframe[4]);

#define FRAME_SIZE 5

volatile uint8_t sendframe[FRAME_SIZE]; // msgid, data...
volatile uint8_t sendframe_c = 0;
volatile uint8_t recvframe[FRAME_SIZE]; // reqid, data...
volatile uint8_t recvframe_c = 0;
// volatile uint8_t recvframe_b = 0;
volatile unsigned long recvframe_t = 0;
volatile uint8_t recvframe_ready = 0;

void request_get_config()
{   
  SENDFRAME_CLEAR
  uint8_t id = recvframe[1];
  if (id < DATA_ITEMS)
  {
    uint8_t idx = id*4;
    Serial.println(idx);
    sendframe[0] = 1;
    sendframe[1] = data[idx]; // mode
    sendframe[2] = data[idx + 1]; // gpio
    sendframe[3] = data[idx + 2]; // value low byte
    sendframe[4] = data[idx + 3]; // value hi byte
  };

  // SPI.transfer((void *)&sendframe, sizeof(sendframe));
  SENDFRAME_SERIAL
};

void request_set_config()
{
  SENDFRAME_CLEAR
  uint8_t id = recvframe[1];
  if (id < DATA_ITEMS)
  {
    uint8_t idx = id*4;
    data[idx] = sendframe[idx + 1];
    data[idx + 1] = sendframe[idx + 2];
    data[idx + 2] = sendframe[idx + 3];
    data[idx + 3] = 0;
  };
  
  configure();
  set();
  eeprom_save();
}

void request_set_values()
{
  //DATA_CLEAR

  SENDFRAME_CLEAR
  uint8_t id = recvframe[1];
  if (id < DATA_ITEMS)
  {
    uint8_t idx = id*4;
    data[idx + 2] = sendframe[idx + 1];
    data[idx + 3] = sendframe[idx + 2];
    
    set();
#ifdef LED_PIN
        LEDC += 1;
#endif
  };
  
  //SPI.transfer((void *)&sendframe, sizeof(sendframe));
  SENDFRAME_SERIAL
}

ISR (SPI_STC_vect)
{
  byte c = SPDR;  // grab byte from SPI Data Register
  
  if (sendframe_c < FRAME_SIZE)
  {
    SPDR = sendframe[sendframe_c];
    sendframe_c++;
  } else {
    SPDR = 0;
  };

  // unsigned long t = micros();
  
  // if (recvframe_ready) { return ;};

  // if (recvframe_c>0)
  // {
  //   if (t<recvframe_t) {
  //     // mitigate overflow problem
  //     recvframe_t = t;
  //   } else if ((t - recvframe_t) > 2000)
  //   {
  //     //timeout old frame
  //     RECVFRAME_CLEAR
  //   };
  // };

  if (recvframe_c < FRAME_SIZE)
  {
    // recvframe_t = t;
    recvframe[recvframe_c]=c;
    recvframe_c++;

    if (recvframe_c == FRAME_SIZE)
    {
      recvframe_ready = 1;
    };
  };


  // Serial.print(recvframe[0]);
  // Serial.print(recvframe[1]);
  // Serial.print(recvframe[2]);
  // Serial.print(recvframe[3]);

  // Serial.println("");
};

void process_request()
{
  Serial.print("Received request: ");
  Serial.println(recvframe[0]);

  if (recvframe[0] == 1)
  {
    request_get_config();
    return;
  };

  if (recvframe[0] == 7)
  {
    request_set_config();
    return;
  };
};

void setup_expander() {
  RECVFRAME_CLEAR

  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SCK, INPUT);
  pinMode(SS, INPUT);

  SPCR |= _BV(SPE);
  // SPCR = (1 << SPE);

  SPI.attachInterrupt();
}

void loop_expander() {

  // Serial.print(recvdata_c);
  // Serial.print(" ");
  // Serial.print(recvdata[0]);
  // Serial.print(" ");
  // Serial.print(recvdata[1]);
  // Serial.print(" ");
  // Serial.println(recvdata_ready);

  if (recvframe_ready)
  {
    process_request();
    RECVFRAME_CLEAR
  };
}

#endif
#endif
