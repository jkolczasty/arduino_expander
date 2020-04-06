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

#define FRAME_SIZE 32

volatile uint8_t sendframe[FRAME_SIZE]; // len, msgid, data...
volatile uint8_t sendframe_c = 0;
volatile uint8_t recvframe[FRAME_SIZE]; // len, reqid, data...
volatile uint8_t recvframe_c = 0;
// volatile uint8_t recvframe_b = 0;
volatile unsigned long recvframe_t = 0;
volatile uint8_t recvframe_ready = 0;

SoftwareSerial _Serial(12, 11);  //rx, tx // MISO/MOSI

void request_get_config()
{   
  SENDFRAME_CLEAR
  uint8_t id = recvframe[2];
  if (id < DATA_ITEMS)
  {
    uint8_t idx = id*4;
    Serial.println(idx);
    sendframe[0] = 5;
    sendframe[1] = 1;
    sendframe[2] = data[idx]; // mode
    sendframe[3] = data[idx + 1]; // gpio
    sendframe[4] = data[idx + 2]; // value low byte
    sendframe[5] = data[idx + 3]; // value hi byte
    _Serial.write((char*)& sendframe, 6);
  };

};

void request_set_config()  // frame: len,7,id,mode,gpio,vl,vh
{
  uint8_t id = recvframe[2];
  if (id < DATA_ITEMS)
  {
    uint8_t idx = id*4;
    Serial.print("SET CONFIG:");
    Serial.print(id);
    Serial.print(idx);
    Serial.print(recvframe[3]);
    Serial.println("---");
    data[idx] = recvframe[3];
    data[idx + 1] = recvframe[4];
    data[idx + 2] = recvframe[5];
    data[idx + 3] = recvframe[6];
  };
  
  configure();
  set();
  eeprom_save();
}

void request_set_values()
{
  //DATA_CLEAR

  SENDFRAME_CLEAR
  uint8_t id = recvframe[2];
  if (id < DATA_ITEMS)
  {
    uint8_t idx = id*4;

    Serial.print("SET VALUE:");
    Serial.print(id);
    Serial.print(" gpio ");
    Serial.print(data[idx+1]);
    Serial.print(" ");
    Serial.print(recvframe[3]);
    Serial.print(" ");
    Serial.print(recvframe[4]);
    Serial.println("---");

    data[idx + 2] = recvframe[3];
    data[idx + 3] = recvframe[4];
    
    set();
#ifdef LED_PIN
        LEDC += 1;
#endif
  };
  
}

void process_request()
{
  Serial.print("Received request: ");
  Serial.println(recvframe[1]);

  if (recvframe[1] == 1)
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
    request_set_values();
    return;
  };
};

#define EXPANDER_ON_CHANGE

void expander_on_change(uint8_t i)
{
    Serial.println("CHANGE");
    SENDFRAME_CLEAR
    uint8_t idx = i*4;
    sendframe[0] = 4;
    sendframe[1] = 9; // value change
    sendframe[2] = i;
    sendframe[3] = data[i*4+2]; //gpio
    sendframe[4] = data[i*4+3]; //gpio
    _Serial.write((char*)& sendframe, 5);
}

void setup_expander() {
    RECVFRAME_CLEAR
    //Serial.begin(115200);
    _Serial.begin(38400);
    Serial.println("SERIAL EXPANDER READY");
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
                Serial.println("RECV TIMEOUT");
                Serial.println(t - recvframe_t);
            };
        };
        if (recvframe_c<sizeof(recvframe))
        {   
            recvframe_t = t;
            recvframe[recvframe_c] = _Serial.read();
            Serial.println(recvframe[recvframe_c]);
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
