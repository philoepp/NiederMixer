#include <NiederMixer.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
//#include <ESPAsyncTCP.h>
//#include <ESPAsyncWebServer.h>
#include <ESP8266WebServer.h>
//#include <AsyncElegantOTA.h>
#include <NeoPixelBus.h>
#include <ESP8266mDNS.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

/* -------------------------------------------------------------------------- 
* DEFINES
---------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------- 
* GLOBAL VARIABLES
---------------------------------------------------------------------------- */
const char* ssid = "NiederMixer";
const char* password = "12345678";
const char* hostname = "niedermixer";

IPAddress accessPointIp(192,168,1,1);
IPAddress subnet(255,255,255,0);

NeoPixelBus<NeoGrbFeature, NeoEsp8266Dma800KbpsMethod> LEDStrip(LED_COUNT, LED_PIN); // Note: Pin is ignored GPIO3 is used!
//AsyncWebServer server(80);

ESP8266WebServer server(80);
WebSocketsServer webSocketServer = WebSocketsServer(81);
_Pumps Pumps = {0};
_LED Led = {};

// JSON communication stuff
StaticJsonDocument<768> doc;
JsonObject Pumps_0 = doc["Pumps"].createNestedObject();
JsonObject LED_0 = doc["LED"].createNestedObject();

/*
 Possible JSON Websocket Prototype

 {
  "Pumps": [
    {
      "Pump1": 0,
      "Pump2": 0,
      "Pump3": 0,
      "Pump4": 0,
      "Pump5": 0,
      "Pump6": 0,
      "Pump7": 0,
      "Pump8": 0,
      "Pump9": 0,
      "Pump10": 0,
      "Pump11": 0,
      "Pump12": 0,
      "Pump13": 0,
      "Pump14": 0,
      "Pump15": 0,
      "Pump16": 0
    }
  ],
  "LED": [
    {
      "Effect": "Single",
      "Red": 255,
      "Green": 255,
      "Blue": 255
    }
  ]
}

*/

/* -------------------------------------------------------------------------- 
* STATIC FUNCTION PROTOTYPES
---------------------------------------------------------------------------- */
static void vInitIOs(void);
static void vInitPumpSetpoints(void);
static void vShiftPumpSetpointsOut(void);
static void vUpdateLeds(void);
static boolean fSetPumpTime(uint8_t pump, uint32_t duration);
static void vProcessPumps(void);
static void vWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
static uint32_t getRemainingPumpRuntime(uint8_t u8Pump);
static void vCopyDataToJson(void);
static void vNotifyClientsIfNeeded(void);

/* -------------------------------------------------------------------------- 
* FUNCTIONS
---------------------------------------------------------------------------- */
void setup(void) 
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("Setup..");

  // Set up WIFI AP
  //WiFi.softAPConfig(accessPointIp, accessPointIp, subnet);
  //WiFi.hostname(hostname);
  //WiFi.softAP(ssid, password);
  
  WiFi.begin("SSID", "PWD");

  while(WiFi.status() != WL_CONNECTED) 
  {
    delay(100);
  }

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  MDNS.begin(hostname);
  MDNS.addService("http", "tcp", 80);
  MDNS.addService("ws", "tcp", 81);
  
  // handle index
  server.on("/", []() 
  {
      server.send(200, "text/html", "<html><head></head><body>Test<br/></body></html>");
  });
  
    //AsyncElegantOTA.begin(&server);    // Start ElegantOTA

  LEDStrip.Begin();
  LEDStrip.Show();

  webSocketServer.begin();
  webSocketServer.onEvent(vWebSocketEvent);

  vInitIOs();
  vInitPumpSetpoints();

  // Init JSON working structure with right values
  vCopyDataToJson();

// Start some pumps for test...
/*
  (void)fSetPumpTime(0, 1000);
  (void)fSetPumpTime(1, 2000);
  (void)fSetPumpTime(2, 3000);
  (void)fSetPumpTime(3, 4000);
  (void)fSetPumpTime(9, 5000);
  (void)fSetPumpTime(15, 6000);*/

// Do a Test of the led strip output
  LEDStrip.SetPixelColor(0, RgbColor(0, 0, 128));
  LEDStrip.SetPixelColor(1, RgbColor(0, 0, 128));

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("Setup..Done!");
}

void loop(void) 
{
  // Provide processing time to the different functions
  webSocketServer.loop();
  vProcessPumps();
  vShiftPumpSetpointsOut();
  vUpdateLeds();
  vNotifyClientsIfNeeded();
  server.handleClient();
}

/* -------------------------------------------------------------------------- 
* STATIC FUNCTIONS
---------------------------------------------------------------------------- */
/**
* @brief Handle the incomming websocket events
* @return None
*/
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
      {
      Serial.printf("[%u] get Text: %s\n", num, payload);

      DeserializationError error = deserializeJson(doc, payload, length);

      if (error) 
      {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        webSocketServer.sendTXT(num, "deserializeJson() failed!");
      }
      else
      {
        JsonObject Pumps_0 = doc["Pumps"][0];
        (void)fSetPumpTime(0, Pumps_0["Pump0"]);
        (void)fSetPumpTime(0, Pumps_0["Pump1"]);
        (void)fSetPumpTime(0, Pumps_0["Pump2"]);
        (void)fSetPumpTime(0, Pumps_0["Pump3"]);
        (void)fSetPumpTime(0, Pumps_0["Pump4"]);
        (void)fSetPumpTime(0, Pumps_0["Pump5"]);
        (void)fSetPumpTime(0, Pumps_0["Pump6"]);
        (void)fSetPumpTime(0, Pumps_0["Pump7"]);
        (void)fSetPumpTime(0, Pumps_0["Pump8"]);
        (void)fSetPumpTime(0, Pumps_0["Pump9"]);
        (void)fSetPumpTime(0, Pumps_0["Pump10"]);
        (void)fSetPumpTime(0, Pumps_0["Pump11"]);
        (void)fSetPumpTime(0, Pumps_0["Pump12"]);
        (void)fSetPumpTime(0, Pumps_0["Pump13"]);
        (void)fSetPumpTime(0, Pumps_0["Pump14"]);
        (void)fSetPumpTime(0, Pumps_0["Pump15"]);

        JsonObject LED_0 = doc["LED"][0];
        Led.Mode = LED_0["Mode"];
        Led.Red = LED_0["Red"];
        Led.Green = LED_0["Green"];
        Led.Blue = LED_0["Blue"];
      }
      // send message to client
      webSocketServer.sendTXT(num, "Request received!");

      // send data to all connected clients
      // webSocket.broadcastTXT("message here");
      break;
      }

    case WStype_BIN:
      Serial.printf("[%u] get binary length: %u\n", num, length);
      hexdump(payload, length);

      // send message to client
      webSocketServer.sendBIN(num, payload, length);
      break;

    default:
      break;
  }
}

/**
* @brief Publish the internal JSON data via websockes to the clients, if needed
* @return None
*/
static void vNotifyClientsIfNeeded(void) 
{
  String jsonMessage;
  static boolean oldPumpState = false;
  static uint32_t u32LastTime = millis();

  // Check if time has elapsed, only then send an update
  if((millis() - u32LastTime) > WEBSOCKET_JSON_UPDATE_INTERVAL)
  {
    // Update time keeper
    u32LastTime = millis();

    // But first check if the pumps are active, or were active
    if(   (Pumps.Outputs.u16Raw != 0)
        ||(oldPumpState))
    {
      // Then puplish the data
      (void)serializeJson(doc, jsonMessage);
      //webSocketServer.broadcastTXT(jsonMessage); // TODO: This somehow makes the ESP crash?!
    }

    // Save last pump state
    oldPumpState = (Pumps.Outputs.u16Raw != 0);
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

  // Init LED variables
  Led.Mode = STATIC;
  Led.Red = 0;
  Led.Green = 0;
  Led.Blue = 0;

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
  static uint32_t u32LastTime = millis();

  // Check if time has elapsed, then shift out the data again
  if((millis() - u32LastTime) > PUMP_SHIFT_OUT_DATA_CYCLE_TIME)
  {
    // Update time keeper
    u32LastTime = millis();

    // Shift out actual data and latch in
    digitalWrite(LATCH, LOW);
    shiftOut(DATA, SCK, MSBFIRST, (uint8_t)((Pumps.Outputs.u16Raw >> 8)& 0xFF));
    shiftOut(DATA, SCK, MSBFIRST, (uint8_t)(Pumps.Outputs.u16Raw & 0xFF));
    digitalWrite(LATCH, HIGH);
  }
}

/**
* @brief Updates the LEDs on a defined time base
* @return None
*/
static void vUpdateLeds(void)
{
  static uint32_t u32LastTime = millis();

  // Check if time has elapsed, then update the LEDs again
  if((millis() - u32LastTime) > LED_UPDATE_CYCLE_TIME)
  {
    // Update time keeper
    u32LastTime = millis();

    // Update LEDs
    LEDStrip.Show();
  }
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
  {
    return fReturn;
  }

  // Limit runtime to maximum time
  if(duration > MAXIMUM_PUMP_RUNTIME)
  {
    duration = MAXIMUM_PUMP_RUNTIME;
  }

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

  if(Pumps.Outputs.u16Raw != 0)
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
}


/**
* @brief Calculate remaining runtime of given pump
* @return Remaining runtime in ms
*/
static uint32_t getRemainingPumpRuntime(uint8_t u8Pump)
{
  uint32_t u32Return = 0;

  // Abort if given pump number isn't within available pumps
  if(u8Pump >= NUMBER_OF_PUMPS)
  {
    return 0;
  }

  // Check if the pump should be on
  if((millis() - Pumps.Pump[u8Pump].StartingTime) < Pumps.Pump[u8Pump].Duration)
  {
    // Pump should be on, calculate remaining runtime
    u32Return = Pumps.Pump[u8Pump].Duration - (millis() - Pumps.Pump[u8Pump].StartingTime);
  }
  else
  {
    // Pump should be off
    u32Return = 0;
  }

    return u32Return;
}


/**
* @brief Copy internal working variables to JSON variables for serialization
* @return None
*/
static void vCopyDataToJson(void)
{
  // Collect remaining runtimes of pumps
  Pumps_0["Pump0"] = getRemainingPumpRuntime(0);
  Pumps_0["Pump1"] = getRemainingPumpRuntime(1);
  Pumps_0["Pump2"] = getRemainingPumpRuntime(2);
  Pumps_0["Pump3"] = getRemainingPumpRuntime(3);
  Pumps_0["Pump4"] = getRemainingPumpRuntime(4);
  Pumps_0["Pump5"] = getRemainingPumpRuntime(5);
  Pumps_0["Pump6"] = getRemainingPumpRuntime(6);
  Pumps_0["Pump7"] = getRemainingPumpRuntime(7);
  Pumps_0["Pump8"] = getRemainingPumpRuntime(8);
  Pumps_0["Pump9"] = getRemainingPumpRuntime(9);
  Pumps_0["Pump10"] = getRemainingPumpRuntime(10);
  Pumps_0["Pump11"] = getRemainingPumpRuntime(11);
  Pumps_0["Pump12"] = getRemainingPumpRuntime(12);
  Pumps_0["Pump13"] = getRemainingPumpRuntime(13);
  Pumps_0["Pump14"] = getRemainingPumpRuntime(14);
  Pumps_0["Pump15"] = getRemainingPumpRuntime(15);
  // TODO Maybe use a loop and string manipulation..

  // Collect LED control status
  LED_0["Mode"]   = Led.Mode;
  LED_0["Red"]    = Led.Red;
  LED_0["Green"]  = Led.Green;
  LED_0["Blue"]   = Led.Blue;
}