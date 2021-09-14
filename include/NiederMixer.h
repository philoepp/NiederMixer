#ifndef NIEDERMIXER_H
#define NIEDERMIXER_H

#include <Arduino.h>

union _Pumps
{
  uint16_t u16Raw;
  struct _Power
  {
    uint8_t Pump1   :1;
    uint8_t Pump2   :1;
    uint8_t Pump3   :1;
    uint8_t Pump4   :1;
    uint8_t Pump5   :1;
    uint8_t Pump6   :1;
    uint8_t Pump7   :1;
    uint8_t Pump8   :1;
    uint8_t Pump9   :1;
    uint8_t Pump10  :1;
    uint8_t Pump11  :1;
    uint8_t Pump12  :1;
    uint8_t Pump13  :1;
    uint8_t Pump14  :1;
    uint8_t Pump15  :1;
    uint8_t Pump16  :1;
  } Power; 
};

#endif