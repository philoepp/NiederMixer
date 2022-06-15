#ifndef NIEDERMIXER_H
#define NIEDERMIXER_H

#include <Arduino.h>

/* -------------------------------------------------------------------------- 
* DEFINES
---------------------------------------------------------------------------- */
#define SCK             D6
#define LATCH           D7
#define DATA            D8
#define ENABLE          D3
#define LED_PIN         D2
#define LED_COUNT       2
#define NUMBER_OF_PUMPS 16
#define PUMP_SHIFT_OUT_DATA_CYCLE_TIME  50 // [ms] 50ms
#define LED_UPDATE_CYCLE_TIME           50 // [ms] 100ms should be enough
#define WEBSOCKET_JSON_UPDATE_INTERVAL  500 // [ms] A update to all WS clients each 500ms, if an update is needed
#define MAXIMUM_PUMP_RUNTIME            (2 * 60 * 1000) // [ms] 2min Maximum

/* -------------------------------------------------------------------------- 
* ENUMS, STRUCTS
---------------------------------------------------------------------------- */
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

typedef enum
{
  STATIC,
  RAINBOW,
  STROBE
} LED_MODE;

struct _LED
{
  LED_MODE Mode;
  uint8_t  Red;
  uint8_t  Green;
  uint8_t  Blue;
};

#endif