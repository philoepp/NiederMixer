#include <NiederMixer.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

/* -------------------------------------------------------------------------- 
* DEFINES
---------------------------------------------------------------------------- */
//#define ENABLE_WIFI

#define SCK     D6
#define LATCH   D7
#define DATA    D8
#define ENABLE  D3
#define LED_PIN D2

#ifdef ENABLE_WIFI
const char* ssid = "";
const char* password = "";
#endif

/* -------------------------------------------------------------------------- 
* STATIC FUNCTION PROTOTYPES
---------------------------------------------------------------------------- */
static void vInitIOs(void);
static void vInitPumpSetpoints(void);
static void vShiftPumpSetpointsOut(void);

AsyncWebServer server(80);
_Pumps Pumps = {0};

/* -------------------------------------------------------------------------- 
* FUNCTIONS
---------------------------------------------------------------------------- */
void setup(void) 
{
  Serial.begin(115200);

  #ifdef ENABLE_WIFI
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", ":(");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  #endif

  vInitIOs();
  vInitPumpSetpoints();
}

void loop(void) 
{
  Pumps.u16Raw = 0;
  Pumps.Power.Pump1 = 1;
  Pumps.Power.Pump6 = 1;
  Pumps.Power.Pump7 = 1;
  Pumps.Power.Pump16 = 1;
  vShiftPumpSetpointsOut();
  delay(1000);

  Pumps.u16Raw = 0;
  Pumps.Power.Pump8 = 1;
  Pumps.Power.Pump9 = 1;
  vShiftPumpSetpointsOut();
  delay(1000);
}

/* -------------------------------------------------------------------------- 
* STATIC FUNCTIONS
---------------------------------------------------------------------------- */
/**
* @brief Initializes the used IOs
* @return None
*/
static void vInitIOs(void)
{
  // WS2812 control output
  pinMode(LED_PIN, OUTPUT);

  // Shift register outputs
  pinMode(ENABLE, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(LATCH, OUTPUT);
  pinMode(DATA, OUTPUT);
}

/**
* @brief Initializes the pump setpoints
* @return None
*/
static void vInitPumpSetpoints(void)
{
  Pumps.u16Raw = 0;
  vShiftPumpSetpointsOut();
  digitalWrite(ENABLE, LOW);
}

/**
* @brief Shifts out the pump setpoints to the shift registers
* @return None
*/
static void vShiftPumpSetpointsOut(void)
{
  digitalWrite(LATCH, LOW);
  shiftOut(DATA, SCK, MSBFIRST, (uint8_t)((Pumps.u16Raw >> 8)& 0xFF));
  shiftOut(DATA, SCK, MSBFIRST, (uint8_t)(Pumps.u16Raw & 0xFF));
  digitalWrite(LATCH, HIGH);
}