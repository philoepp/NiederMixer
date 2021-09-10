#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#define SCK     D6
#define LATCH   D7
#define DATA    D8
#define ENABLE  D3
#define LED_PIN D2

const char* ssid = "";
const char* password = "";

AsyncWebServer server(80);

void setup(void) {
  Serial.begin(115200);
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

  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  pinMode(D8, OUTPUT);
}

void loop(void) 
{
  digitalWrite(ENABLE, LOW);

    for (int j = 0; j < 256; j++) {
	//Register pin grounden und low halten, solange übertragen wird
    digitalWrite(LATCH, LOW);
    shiftOut(DATA, SCK, MSBFIRST, j);
	//Register pin auf high setzen, um dem Chip zu signalisieren, dass
	//er nicht mehr länger Informationen lesen muss
    digitalWrite(LATCH, HIGH);
    delay(500);
  }
}