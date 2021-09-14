#ifndef NIEDERMIXER_H
#define NIEDERMIXER_H

#include <Arduino.h>

#define NUMBER_OF_PUMPS 16

typedef enum
{
  OFF,
  ON
} PUMP_STATE;

union PumpOutput
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

struct _Pump
{
  uint32_t  StartingTime;
  uint32_t  Duration;
  PUMP_STATE State;
};

struct _Pumps
{
  PumpOutput Outputs;
  _Pump Pump[NUMBER_OF_PUMPS];
};

#endif