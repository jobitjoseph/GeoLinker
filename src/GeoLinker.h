/* 
 * GeoLinker Library
 * Copyright (C) 2025 Jobit Joseph, Semicon Media Pvt Ltd (Circuit Digest)
 * Author: Jobit Joseph
 * Project: GeoLinker Cloud API Library
 *
 * Licensed under the MIT License
 * You may not use this file except in compliance with the License.
 * 
 * You may obtain a copy of the License at:
 * https://opensource.org/license/mit/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software (the "Software") and associated documentation files, to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, subject to the following additional conditions:

 * 1. All copies or substantial portions must retain:  
 *    - The original copyright notice  
 *    - A prominent statement crediting the original author/creator  

 * 2. Modified versions must:  
 *    - Clearly mark the changes as their own  
 *    - Preserve all original credit notices
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef GEOLINKER_H
#define GEOLINKER_H

#include <Arduino.h>
#include <vector>
#include <map>

#if defined(ESP32)
  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <HTTPClient.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h>
  #include <WiFiClientSecure.h>
#elif defined(ARDUINO_UNOWIFIR4)
  #include <WiFiS3.h>
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W) || defined(ARDUINO_RASPBERRY_PI_PICO_2W)
  #include <WiFi.h>
  #include <WiFiClientSecure.h>
  #include <HTTPClient.h>
#else
  #include <WiFiNINA.h>
  #include <WiFiClient.h>
  #include <ArduinoHttpClient.h>
#endif

#include <ArduinoJson.h>
#include <Stream.h>

#define DEBUG_NONE 0
#define DEBUG_BASIC 1
#define DEBUG_VERBOSE 2

  enum ParseResult {
  PARSE_SUCCESS = 0,
  PARSE_NO_DATA = 1,
  PARSE_INVALID_STATUS = 2,
  PARSE_INVALID_FORMAT = 3
};


class GeoLinker {
public:
  GeoLinker();
  bool begin(Stream& serial);
  void setApiKey(const String& key);
  void setDeviceID(const String& id);
  void setUpdateInterval_seconds(uint32_t interval_s);
  void setDebugLevel(uint8_t level);
  void enableOfflineStorage(bool enable);
  void enableAutoReconnect(bool enable);
  void setPayloads(const std::map<String, float>& payloads);
  void setBatteryLevel(uint8_t percent);
  void setTimeOffset(int hourOffset, int minuteOffset);
  void setWiFiCredentials(const char* wifi_ssid, const char* wifi_password);
  bool connectToWiFi();
  uint8_t loop();

private:
  Stream* gpsSerial;
  String apiKey;
  String deviceID;
  uint8_t debugLevel = DEBUG_NONE;
  uint32_t updateInterval = 60000;
  uint32_t lastUpdate = 0;
  bool offlineEnabled = false;
  bool reconnectEnabled = false;
  uint8_t batteryLevel = 255;
  std::map<String, float> payloadMap;
  const char* wifi_ssid = nullptr;
  const char* wifi_password = nullptr;

  String gpsBuffer;
  String lastValidGPRMC;
  bool newDataAvailable;
  unsigned long lastDataTime;

  struct GpsData {
    float lat;
    float lon;
    String timestamp;
  };

  std::vector<GpsData> offlineBuffer;
  const size_t offlineLimit = 100;
  int offsetHour = 0, offsetMin = 0;
  
  void handleSerialData();
  ParseResult parseNMEA(String& line, float& lat, float& lon, String& utcTime);
  bool getCoordinates(float& lat, float& lon, String& timestamp, ParseResult& parseResult);
  uint16_t sendData(float lat, float lon, String timestamp);
  void storeOfflineData(float lat, float lon, String timestamp);
  bool sendOfflineData();
  bool postToServer(const String& json);
  void debugPrint(const String& msg, uint8_t level);
  void debugPrintNoNewline(const String& msg, uint8_t level);
  bool isWiFiConnected();
  bool isValidGps(float lat, float lon);
  void handleWiFiReconnect();
};

#endif