#include <NiederMixer.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <NeoPixelBus.h>
#include <ESP8266mDNS.h>
#include <WebSocketsServer.h>

/* -------------------------------------------------------------------------- 
* DEFINES
---------------------------------------------------------------------------- */
#define SCK     D6
#define LATCH   D7
#define DATA    D8
#define ENABLE  D3
#define LED_PIN D2
#define LED_COUNT 2

const char* ssid = "NiederMixer";
const char* password = "12345678";
const char* hostname = "niedermixer";

IPAddress accessPointIp(192,168,1,1);
IPAddress subnet(255,255,255,0);

/* -------------------------------------------------------------------------- 
* STATIC FUNCTION PROTOTYPES
---------------------------------------------------------------------------- */
static void vInitIOs(void);
static void vInitPumpSetpoints(void);
static void vShiftPumpSetpointsOut(void);
static boolean fSetPumpTime(uint8_t pump, uint32_t duration);
static void vProcessPumps(void);
static void vWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

NeoPixelBus<NeoGrbFeature, NeoEsp8266Dma800KbpsMethod> LEDStrip(LED_COUNT, LED_PIN); // Note: Pin is ignored GPIO3 is used!
AsyncWebServer server(80);
WebSocketsServer webSocketServer = WebSocketsServer(81);
_Pumps Pumps = {0};

/* -------------------------------------------------------------------------- 
* FUNCTIONS
---------------------------------------------------------------------------- */
void setup(void) 
{
  Serial.begin(115200);

  // Set up WIFI AP
  WiFi.softAPConfig(accessPointIp, accessPointIp, subnet);
  WiFi.hostname(hostname);
  WiFi.softAP(ssid, password);

  MDNS.begin(hostname);
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", ":(");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA

  LEDStrip.Begin();
  LEDStrip.Show();

  webSocketServer.begin();
  webSocketServer.onEvent(vWebSocketEvent);

  vInitIOs();
  vInitPumpSetpoints();

// Start some pumps for test...
  (void)fSetPumpTime(0, 1000);
  (void)fSetPumpTime(1, 2000);
  (void)fSetPumpTime(2, 3000);
  (void)fSetPumpTime(3, 4000);
  (void)fSetPumpTime(9, 5000);
  (void)fSetPumpTime(15, 6000);
}

void loop(void) 
{
  LEDStrip.SetPixelColor(0, RgbColor(0, 0, 128));
  LEDStrip.SetPixelColor(1, RgbColor(0, 0, 128));
  LEDStrip.Show();

  webSocketServer.loop();
  
  vProcessPumps();
  vShiftPumpSetpointsOut(); // Update the outputs based on a fixed time base!
  delay(50); //TODO add a fixed time base
}

/* -------------------------------------------------------------------------- 
* STATIC FUNCTIONS
---------------------------------------------------------------------------- */
static void vWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) 
{
  switch(type) 
  {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
    {
      IPAddress ip = webSocketServer.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);

        // send message to client
      webSocketServer.sendTXT(num, "Connected");
      break;
    }
    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\n", num, payload);

      // send message to client
      // webSocket.sendTXT(num, "message here");

      // send data to all connected clients
      // webSocket.broadcastTXT("message here");
      break;
    case WStype_BIN:
      Serial.printf("[%u] get binary length: %u\n", num, length);
      hexdump(payload, length);

      // send message to client
      // webSocket.sendBIN(num, payload, length);
      break;

    default:
      break;
  }
}

/**
* @brief Initializes the used IOs
* @return None
*/
static void vInitIOs(void)
{
  // WS2812 control output
  pinMode(LED_PIN, OUTPUT); // Not needed.. GPIO3 is used via DMA

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
  Pumps.Outputs.u16Raw = 0;
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
  shiftOut(DATA, SCK, MSBFIRST, (uint8_t)((Pumps.Outputs.u16Raw >> 8)& 0xFF));
  shiftOut(DATA, SCK, MSBFIRST, (uint8_t)(Pumps.Outputs.u16Raw & 0xFF));
  digitalWrite(LATCH, HIGH);
}

/**
* @brief Sets the remaining time [ms] for a given pump
* @return True if used, false if pump was busy
*/
static boolean fSetPumpTime(uint8_t pump, uint32_t duration)
{
  boolean fReturn = false;

  // Check array limitation
  if(pump >= NUMBER_OF_PUMPS)
    return fReturn;

  // Check if pump isn't busy
  if(Pumps.Pump[pump].State == OFF)
  {
    Pumps.Pump[pump].StartingTime = millis();
    Pumps.Pump[pump].Duration = duration;
    fReturn = true;
  }
  else // Pump is still busy, don't accept new value
  {
    fReturn = false;
  }

  return fReturn;
}

/**
* @brief Process the pump outputs based on the given data
* @return None
*/
static void vProcessPumps(void)
{
  // Loop through all available pumps
  for(uint8_t i = 0; i < NUMBER_OF_PUMPS; i++)
  {
    // Check if the pump should be on
    if((millis() - Pumps.Pump[i].StartingTime) < Pumps.Pump[i].Duration)
    {
      // Switch the pump on
      Pumps.Pump[i].State = ON;
    }
    else
    {
      // Switch the pump off
      Pumps.Pump[i].State = OFF;
    }
  }

  // Reset outputs to defined state
  Pumps.Outputs.u16Raw = 0;

  // Apply the pump state to the desired outputs
  for(uint8_t i = 0; i < NUMBER_OF_PUMPS; i++)
  {
    Pumps.Outputs.u16Raw |= (uint16_t)Pumps.Pump[i].State << i;
  }
}