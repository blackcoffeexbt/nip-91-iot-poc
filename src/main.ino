#include <vector>
#include <utility>
#include <math.h>

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "time.h"
#include <NostrEvent.h>
#include <NostrRelayManager.h>
#include <NostrRequestOptions.h>
#include <Wire.h>
#include "Bitcoin.h"
#include "Hash.h"
#include <esp_random.h>
#include "QRCode.h"
#include <SPIFFS.h>
#include <ESP32Ping.h>
#include "wManager.h"

#include <ArduinoJson.h>

#include "sensor.h"

#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C


#define PARAM_FILE "/elements.json"


int triggerAp = false;

bool lastInternetConnectionState = true;

int socketDisconnectedCount = 0;

NostrEvent nostr;
NostrRelayManager nostrRelayManager;
NostrQueueProcessor nostrQueue;

String serialisedEventRequest;

extern bool hasInternetConnection;

NostrRequestOptions* eventRequestOptions;

bool hasSentEvent = false;

char const *nsecHex = "bdd19cecd942ed8964c2e0ddc92d5e09838d3a09ebb230d974868be00886704b"; // sender private key in hex e.g. bdd19cecdXXXXXXXXXXXXXXXXXXXXXXXXXX
char const *npubHex = "d0bfc94bd4324f7df2a7601c4177209828047c4d3904d64009a3c67fb5d5e7ca"; // sender public key in hex e.g. d0bfc94bd4324f7df2a7601c4177209828047c4d3904d64009a3c67fb5d5e7ca

extern char npubHexString[80];
extern char relayString[80];

fs::SPIFFSFS &FlashFS = SPIFFS;
#define FORMAT_ON_FAIL true

// define funcs
void configureAccessPoint();
void initWiFi();
bool whileCP(void);
unsigned long getUnixTimestamp();
// void noteEvent(const std::string& key, const char* payload);
void okEvent(const std::string& key, const char* payload);
void nip01Event(const std::string& key, const char* payload);
void relayConnectedEvent(const std::string& key, const std::string& message);
void relayDisonnectedEvent(const std::string& key, const std::string& message);
uint16_t getRandomNum(uint16_t min, uint16_t max);
void loadSettings();
void connectToNostrRelays();

// Define the WiFi event callback function
void WiFiEvent(WiFiEvent_t event) {
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("Connected to WiFi and got an IP");
      connectToNostrRelays();      
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi");
      // WiFi.begin(ssid, password); // Try to reconnect after getting disconnected
      break;
  }
}

/**
 * @brief Connect to the Nostr relays
 * 
 */
void connectToNostrRelays() {
  // first disconnect from all relays
  nostrRelayManager.disconnect();
  Serial.println("Requesting events");

  // split relayString by comma into vector
  std::vector<String> relays;
  // String relayStringCopy = String(relayString);
  // int commaIndex = relayStringCopy.indexOf(",");
  relays.push_back("nos.lol");

  // no need to convert to char* anymore
  nostr.setLogging(true);
  nostrRelayManager.setRelays(relays);
  nostrRelayManager.setMinRelaysAndTimeout(1,10000);

  // Set some event specific callbacks here
  Serial.println("Setting callbacks");
  nostrRelayManager.setEventCallback("ok", okEvent);
  nostrRelayManager.setEventCallback("connected", relayConnectedEvent);
  nostrRelayManager.setEventCallback("disconnected", relayDisonnectedEvent);
  // nostrRelayManager.setEventCallback(1, noteEvent);

  Serial.println("connecting");
  nostrRelayManager.connect();
}

// Function that gets current epoch time
unsigned long getUnixTimestamp() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

String lastPayload = "";

void relayConnectedEvent(const std::string& key, const std::string& message) {
  socketDisconnectedCount = 0;
  Serial.println("Relay connected: ");
  
  Serial.print(F("Requesting events:"));
  Serial.println(serialisedEventRequest);

  nostrRelayManager.broadcastEvent(serialisedEventRequest);
}

void relayDisonnectedEvent(const std::string& key, const std::string& message) {
  Serial.println("Relay disconnected: ");
  socketDisconnectedCount++;
  // reboot after 3 socketDisconnectedCount subsequenet messages
  if(socketDisconnectedCount >= 3) {
    Serial.println("Too many socket disconnections. Restarting");
    // restart device
    ESP.restart();
  }
}

void okEvent(const std::string& key, const char* payload) {
    if(lastPayload != payload) { // Prevent duplicate events from multiple relays triggering the same logic
      lastPayload = payload;
      Serial.println("payload is: ");
      Serial.println(payload);
    }
}

void nip01Event(const std::string& key, const char* payload) {
    if(lastPayload != payload) { // Prevent duplicate events from multiple relays triggering the same logic
      lastPayload = payload;
      // We can convert the payload to a StaticJsonDocument here and get the content
      StaticJsonDocument<1024> eventJson;
      deserializeJson(eventJson, payload);
      String pubkey = eventJson[2]["pubkey"].as<String>();
      String content = eventJson[2]["content"].as<String>();
      Serial.println(pubkey + ": " + content);
    }
}

uint16_t getRandomNum(uint16_t min, uint16_t max) {
  uint16_t rand  = (esp_random() % (max - min + 1)) + min;
  Serial.println("Random number: " + String(rand));
  return rand;
}

void setup() {
  Serial.begin(115200);
  Serial.println("boot");

  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
    if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  
  FlashFS.begin(FORMAT_ON_FAIL);
  // init spiffs
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  randomSeed(analogRead(0)); // Seed the random number generator

  // delay(500);
  // signalWithLightning(2,250);

  WiFi.onEvent(WiFiEvent);
  init_WifiManager();

  if(hasInternetConnection) {
  Serial.println("Has internet connection. Connectring to relays");
  connectToNostrRelays();
  }

  const char* ntpServer = "uk.pool.ntp.org";
  long gmtOffset_sec = 0;
  long daylightOffset_sec = 0;
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // get the sensor data, create the event and push it to the queue
  unsigned long timestamp = getUnixTimestamp();
  float temperature = bmp.readTemperature();
  Serial.println("Temperature: " + String(temperature) + "°C");

  StaticJsonDocument<512> tagsDoc;
  JsonArray tags = tagsDoc.to<JsonArray>();

  // Add device tag
  JsonArray deviceTag = tags.createNestedArray();
  deviceTag.add("device");
  deviceTag.add("1337107");

  // Add alt tag
  JsonArray altTag = tags.createNestedArray();
  altTag.add("alt");
  altTag.add("Temperature is " + String(temperature) + "°C");

  String noteString = nostr.getNote(nsecHex, npubHex, timestamp, String(temperature), 8003, tags);
  nostrRelayManager.enqueueMessage(noteString.c_str());

  float pressure = bmp.readPressure() / 100.0F;
  Serial.println("pressure: " + String(pressure) + "hPa");

  StaticJsonDocument<512> pressureTagsDoc;
  JsonArray pressureTags = pressureTagsDoc.to<JsonArray>();

  // Add device tag
  JsonArray pressureDeviceTag = pressureTags.createNestedArray();
  pressureDeviceTag.add("device");
  pressureDeviceTag.add("1337107");

  // Add alt tag
  JsonArray pressureAltTag = pressureTags.createNestedArray();
  pressureAltTag.add("alt");
  pressureAltTag.add("Pressure is " + String(pressure) + "hPa");

  noteString = nostr.getNote(nsecHex, npubHex, timestamp, String(pressure), 8004, pressureTags);
  nostrRelayManager.enqueueMessage(noteString.c_str());
}

long lastInternetConnectionCheckTime = 0;
long firstLoopTime = 0;

void loop() {
  if (millis() - lastInternetConnectionCheckTime > 10000) {
    if(WiFi.status() == WL_CONNECTED) {
      IPAddress ip(9,9,9,9);  // Quad9 DNS
      bool ret = Ping.ping(ip);
      if(ret) {
        if(!lastInternetConnectionState) {
          Serial.println("Internet connection has come back! :D");
          // reboot
          ESP.restart();
        }
        lastInternetConnectionState = true;
      } else {
        lastInternetConnectionState = false;
      }
    }
  }

  nostrRelayManager.loop();
  nostrRelayManager.broadcastEvents();

  if(firstLoopTime == 0) {
    firstLoopTime = millis();
  }
  // deep sleep for 30 minutes if more than 30 seconds has passed since the first loop
  if(millis() - firstLoopTime > millis() + 30000) {
    Serial.println("Going to sleep for 30 minutes");
    ESP.deepSleep(1800000000);
  }


}