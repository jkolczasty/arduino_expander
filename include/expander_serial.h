/**
 * arduino_expander
 *  
 *  serial model
 * 
 * Author: jkolczasty@gmail.com 
 * License: MIT
 * 
 */


#ifdef USE_SERIAL

#ifndef EXPANDER_SERIAL_H_
#define EXPANDER_SERIAL_H_

#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <expander_common.h>
//#include <HardwareSerial.h>
#include <SoftwareSerial.h>

#define RECVFRAME_CLEAR memset((void *)&recvframe, 0, sizeof(recvframe)); recvframe_c = 0;
#define SENDFRAME_CLEAR memset((void *)&sendframe, 0, sizeof(sendframe)); sendframe_c = 0;

#define FRAME_SIZE 32+2

volatile uint8_t sendframe[FRAME_SIZE]; // len, msgid, data...
volatile uint8_t sendframe_c = 0;
volatile uint8_t recvframe[FRAME_SIZE]; // len, reqid, data...
volatile uint8_t recvframe_c = 0;
// volatile uint8_t recvframe_b = 0;
volatile unsigned long recvframe_t = 0;
volatile uint8_t recvframe_ready = 0;

SoftwareSerial _Serial(12, 11);  //rx, tx // MISO/MOSI


void send_response(uint8_t count)
{
  sendframe[0] = count + 1;
  _Serial.write((char*)& sendframe, count + 1);

#ifdef DEBUG
  Serial.print("SEND: ");
  for (uint8_t i=0; i<count + 1; i++) { Serial.print(sendframe[i]); Serial.print(" ");};
  Serial.println("");
#endif
}
#define EXPANDER_ON_CHANGE

void request_get_config()
{   
  SENDFRAME_CLEAR
  uint8_t id = recvframe[2];
  if (id < DATA_ITEMS)
  {
    uint8_t idx = id*4;
    sendframe[1] = 1;
    sendframe[2] = data[idx]; // mode
    sendframe[3] = data[idx + 1]; // gpio
    sendframe[4] = data[idx + 2]; // value low byte
    sendframe[5] = data[idx + 3]; // value hi byte
    send_response(5);
  };
};

void request_get_full_config()
{   
  SENDFRAME_CLEAR
  uint8_t c = 2; // len, cmd
  uint8_t idx;
  
  sendframe[1] = 1;

  for (uint8_t i=0; i<DATA_ITEMS; i++)
  {
    idx = i*4;
    sendframe[c] = data[idx]; // mode
    sendframe[c+1] = data[idx + 1]; // gpio
    sendframe[c+2] = data[idx + 2]; // value low byte
    sendframe[c+3] = data[idx + 3]; // value hi byte
    c = c + 4;
  };
  send_response(c-1);
};

void request_set_config()  // frame: len,7,[id,mode,gpio,vl,vh]+
{

  uint8_t idx;
  uint8_t idx2;
  uint8_t id;

  for (uint8_t i=0; i<(recvframe[0]-2); i++)
  {
    idx = 2 + i*5;
    id = recvframe[idx];

    if (id >= DATA_ITEMS)
    {
      continue;
    };
    idx2 = id * 4;

#ifdef DEBUG    
    Serial.print("SET CONFIG:");
    Serial.print(id);
    Serial.print(" ");
    Serial.print(recvframe[idx+1]);
    Serial.print(" ");
    Serial.print(recvframe[idx+2]);
    Serial.println("---");
#endif
    data[idx2] = recvframe[idx+1];
    data[idx2 + 1] = recvframe[idx+2];
    data[idx2 + 2] = recvframe[idx+3];
    data[idx2 + 3] = recvframe[idx+4];
  };
  
  eeprom_save();

  configure();
  set();
}

void request_set_value()
{
  //DATA_CLEAR

  SENDFRAME_CLEAR
  uint8_t id = recvframe[2];
  if (id < DATA_ITEMS)
  {
    uint16_t v = recvframe[3] + (recvframe[4] << 8);

#ifdef DEBUG2
    Serial.print("SET VALUE:");
    Serial.print(id);
    Serial.print(" gpio ");
    Serial.print(data[id*4 + 1]);
    Serial.print(" ");
    Serial.print(v);
    Serial.println("---");
#endif
    if (values[id] != v)
    {
#ifdef LED_PIN
      LEDC += 1;
#endif
    };
    values[id] = v;
    
    set();
  };
}

void request_get_values()
{   
  SENDFRAME_CLEAR
  uint8_t c = 2; // len, cmd
  
  sendframe[1] = 10;

  for (uint8_t i=0; i<DATA_ITEMS; i++)
  {
    sendframe[c] = values[i] & 0xff;
    sendframe[c+1] = values[i] >> 8;
    c = c + 2;
  };
  send_response(c-1);
};

void process_request()
{
#ifdef DEBUG
  Serial.print("RECV: ");
  for (uint8_t i=0; i<recvframe_c; i++) {Serial.print(recvframe[i]); Serial.print(" ");};
  Serial.println("");
#endif

  if (recvframe[1] == 1)
  {
    request_get_full_config();
    return;
  };

  if (recvframe[1] == 2)
  {
    request_get_config();
    return;
  };

  if (recvframe[1] == 7)
  {
    request_set_config();
    return;
  };

  if (recvframe[1] == 8)
  {
    request_set_value();
    return;
  };

  if (recvframe[1] == 10)
  {
    request_get_values();
    return;
  };  
};

void expander_on_change(uint8_t i)
{
#ifdef DEBUG  
    Serial.println("CHANGE");
#endif
    SENDFRAME_CLEAR
    sendframe[1] = 9; // value change
    sendframe[2] = i;
    sendframe[3] = values[i] & 0xff;
    sendframe[4] = values[i] >> 8;
    send_response(4);
}

void setup_expander() {
    RECVFRAME_CLEAR
    //Serial.begin(115200);
    _Serial.begin(38400);
#ifdef DEBUG    
    Serial.println("SERIAL EXPANDER READY");
#endif
    set();
}

void loop_expander() {
    
  if (_Serial.available())
  {
        unsigned long t = micros();
        if (recvframe_c>0)
        {
            if (t<recvframe_t) {
                // mitigate overflow problem
                recvframe_t = t;
            } else if ((t - recvframe_t) > 2000)
            {
                //timeout old frame
                RECVFRAME_CLEAR
#ifdef DEBUG
                Serial.println("RECV TIMEOUT");
                Serial.println(t - recvframe_t);
#endif
            };
        };
        if (recvframe_c<sizeof(recvframe))
        {   
            recvframe_t = t;
            recvframe[recvframe_c] = _Serial.read();
#ifdef DEBUG3
            Serial.println(recvframe[recvframe_c]);
#endif
            recvframe_c++;
        };
        if (recvframe_c == (recvframe[0] + 1))
        {
            process_request();
            RECVFRAME_CLEAR;
        };
  };
}

#endif
#endif
