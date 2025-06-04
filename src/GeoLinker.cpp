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

#include "GeoLinker.h"

GeoLinker::GeoLinker() : 
    gpsSerial(nullptr),
    newDataAvailable(false),
    lastDataTime(0) {
}

void GeoLinker::setApiKey(const String& key) { apiKey = key; }
void GeoLinker::setDeviceID(const String& id) { deviceID = id; }
void GeoLinker::setUpdateInterval_seconds(uint32_t interval_s) { updateInterval = interval_s * 1000; }
void GeoLinker::setDebugLevel(uint8_t level) { debugLevel = level; }
void GeoLinker::enableOfflineStorage(bool enable) { offlineEnabled = enable; }
void GeoLinker::enableAutoReconnect(bool enable) { reconnectEnabled = enable; }
void GeoLinker::setBatteryLevel(uint8_t percent) { batteryLevel = percent; }

bool GeoLinker::begin(Stream& serial) {
  gpsSerial = &serial;
  return true;
}

void GeoLinker::setWiFiCredentials(const char* wifi_ssid, const char* wifi_password) {
  this->wifi_ssid = wifi_ssid;
  this->wifi_password = wifi_password;
}

bool GeoLinker::connectToWiFi() {
  if (wifi_ssid == nullptr || wifi_password == nullptr) {
    debugPrint("WiFi credentials not set!", DEBUG_BASIC);
    return false;
  }

  debugPrint("Starting WiFi connection to: " + String(wifi_ssid), DEBUG_BASIC);
  
  WiFi.begin(wifi_ssid, wifi_password);
  
  unsigned long start = millis();
  int dotCount = 0;
  
  unsigned long previousPrint = 0;

  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    if (millis() - previousPrint >= 500) {
      previousPrint = millis();
      debugPrintNoNewline(".", DEBUG_BASIC);
      }

    yield(); // Important for ESP8266 to avoid watchdog reset
  }


  if (WiFi.status() == WL_CONNECTED) {
    debugPrint("", DEBUG_BASIC); // Add newline after dots
    debugPrint("WiFi Connected Successfully!", DEBUG_BASIC);
    #if defined(ARDUINO_UNOWIFIR4) && (debugLevel > DEBUG_BASIC)
      delay(1000);
    #endif
    debugPrint("IP Address: " + WiFi.localIP().toString(), DEBUG_BASIC);
    debugPrint("Signal Strength: " + String(WiFi.RSSI()) + " dBm", DEBUG_BASIC);
    return true;
  }
  
  debugPrint("WiFi Connection Failed - Timeout", DEBUG_BASIC);
  return false;
}

void GeoLinker::handleWiFiReconnect() {
  static unsigned long lastReconnectAttempt = 0;
  static bool wasConnected = true; // Assume initially connected
  const unsigned long reconnectInterval = 5000;
  
  bool currentlyConnected = isWiFiConnected();
  
  // Detect disconnection event
  if (wasConnected && !currentlyConnected) {
    debugPrint("WiFi Disconnected!", DEBUG_BASIC);
    wasConnected = false;
  }
  
  // Detect reconnection event  
  if (!wasConnected && currentlyConnected) {
    debugPrint("WiFi Reconnected Successfully!", DEBUG_BASIC);
    #if defined(ARDUINO_UNOWIFIR4) && (debugLevel > DEBUG_BASIC)
      delay(1000);
    #endif
    debugPrint("IP Address: " + WiFi.localIP().toString(), DEBUG_BASIC);
    wasConnected = true;
    return;
  }
  
  // Attempt reconnection if disconnected
  if (!currentlyConnected && millis() - lastReconnectAttempt >= reconnectInterval) {
    debugPrint("Attempting WiFi reconnection...", DEBUG_BASIC);
    lastReconnectAttempt = millis();
    
    #if defined(ESP8266)
      WiFi.reconnect();
    #elif defined(ARDUINO_RASPBERRY_PI_PICO_W) || defined(ARDUINO_RASPBERRY_PI_PICO_2W)
      if (wifi_ssid && wifi_password) {
        WiFi.begin(wifi_ssid, wifi_password);
      }
    #else
      WiFi.disconnect();
      WiFi.begin(wifi_ssid, wifi_password);
    #endif
  }
}

void GeoLinker::setPayloads(const std::map<String, float>& newPayloads) {
  payloadMap.clear();
  payloadMap.insert(newPayloads.begin(), newPayloads.end());
}

void GeoLinker::setTimeOffset(int hourOffset, int minuteOffset) {
  offsetHour = hourOffset;
  offsetMin = minuteOffset;
}

bool GeoLinker::isWiFiConnected() {
#if defined(ESP32) || defined(ESP8266)
  return WiFi.status() == WL_CONNECTED;
#else
  return WiFi.status() == WL_CONNECTED;
#endif
}

uint8_t GeoLinker::loop() {
  uint8_t status = 0;
  handleSerialData();
  
  // Offline data sending
  if (offlineEnabled && !offlineBuffer.empty() && isWiFiConnected()) {
    lastUpdate = millis();
    if (sendOfflineData()) {
      if (status != 4) status = 1;
    }
    else if (status == 0) {
      status = 3;
    }
  }

  // Main update cycle
  if (millis() - lastUpdate >= updateInterval) {
    float lat = 0.0, lon = 0.0;
    String time;
    ParseResult parseResult;
    
    if (getCoordinates(lat, lon, time, parseResult)) {
      int sendResult = sendData(lat, lon, time);
      if (sendResult == 200) {
        status = 1;
        offlineBuffer.clear();
      } else if (sendResult == 401) {
        status = 4;
        offlineBuffer.clear();
      } else {
        if (offlineEnabled) {
          storeOfflineData(lat, lon, time);
        }
        status = 3;
      }
    } else {
      // Handle different GPS failure types
      if (parseResult == PARSE_INVALID_STATUS || parseResult == PARSE_INVALID_FORMAT) {
        status = 5; // NMEA parse error
      } else {
        status = 2; // No GPS data or other GPS issues
      }
    }
    lastUpdate = millis();
  }

  // WiFi reconnection
  if (reconnectEnabled && !isWiFiConnected()) {
    handleWiFiReconnect();
  }

  return status;
}


void GeoLinker::handleSerialData() {
    while (gpsSerial->available()) {
        char c = gpsSerial->read();
        
        if (debugLevel >= DEBUG_VERBOSE) {
            Serial.write(c);  // Echo raw data in verbose mode
        }
        
        if (c == '\n') {
            // Process complete line
            if (gpsBuffer.startsWith("$GPRMC")) {
                lastValidGPRMC = gpsBuffer;
                newDataAvailable = true;
                lastDataTime = millis();
                
                if (debugLevel >= DEBUG_VERBOSE) {
                    debugPrint("Stored GPRMC: " + lastValidGPRMC, DEBUG_VERBOSE);
                }
            }
            gpsBuffer = "";
        } 
        else if (c != '\r') {  // Ignore carriage returns
            gpsBuffer += c;
        }
        
        // Prevent buffer overflow
        if (gpsBuffer.length() > 128) {
            gpsBuffer = "";
        }
    }
}

bool GeoLinker::getCoordinates(float& lat, float& lon, String& timestamp, ParseResult& parseResult) {
    // Check if we have valid data
    if (!newDataAvailable) {
        debugPrint("No new GPS data available", DEBUG_BASIC);
        parseResult = PARSE_NO_DATA;
        return false;
    }
    
    // Parse the stored message
    parseResult = parseNMEA(lastValidGPRMC, lat, lon, timestamp);
    newDataAvailable = false;
    
    return (parseResult == PARSE_SUCCESS);
}
ParseResult GeoLinker::parseNMEA(String& line, float& lat, float& lon, String& utcTime) {
  // Print the GPRMC sentence being parsed at verbose level
  debugPrint("Parsing GPRMC: " + line, DEBUG_VERBOSE);
  
  // Example: $GPRMC,hhmmss.000,A,latitude,N,longitude,E,...
  int idx = 0;
  String parts[13];
  String originalLine = line; // Keep original for parsing
  
  while (originalLine.length() && idx < 13) {
    int comma = originalLine.indexOf(',');
    parts[idx++] = originalLine.substring(0, comma);
    originalLine = (comma == -1) ? "" : originalLine.substring(comma + 1);
  }

  // Check if we have enough parts
  if (idx < 10) {
    debugPrint("GPS NMEA format invalid - insufficient data fields", DEBUG_BASIC);
    return PARSE_INVALID_FORMAT;
  }

  if (parts[2] != "A") {
    debugPrint("GPS data invalid (status not 'A')", DEBUG_VERBOSE);
    return PARSE_INVALID_STATUS;  // Invalid status
  }

  // Validate date format (ddmmyy)
  String date = parts[9];
  if (date.length() != 6) {
    debugPrint("Invalid date format, discarding data", DEBUG_VERBOSE);
    return PARSE_INVALID_FORMAT;
  }

  // Validate day (01-31)
  int day = date.substring(0, 2).toInt();
  if (day < 1 || day > 31) {
    debugPrint("Invalid day in date, discarding data", DEBUG_VERBOSE);
    return PARSE_INVALID_FORMAT;
  }

  // Validate month (01-12)
  int month = date.substring(2, 4).toInt();
  if (month < 1 || month > 12) {
    debugPrint("Invalid month in date, discarding data", DEBUG_VERBOSE);
    return PARSE_INVALID_FORMAT;
  }

  // Process coordinates
  float rawLat = parts[3].toFloat();
  float rawLon = parts[5].toFloat();

  debugPrint("Raw coordinates - Lat: " + String(rawLat, 6) + " Lon: " + String(rawLon, 6), DEBUG_VERBOSE);

  int latDeg = int(rawLat / 100);
  float latMin = rawLat - latDeg * 100;
  lat = latDeg + (latMin / 60.0);
  if (parts[4] == "S") lat = -lat;

  int lonDeg = int(rawLon / 100);
  float lonMin = rawLon - lonDeg * 100;
  lon = lonDeg + (lonMin / 60.0);
  if (parts[6] == "W") lon = -lon;

  debugPrint("Converted coordinates - Lat: " + String(lat, 6) + " Lon: " + String(lon, 6), DEBUG_VERBOSE);

  // Build timestamp
  utcTime = "20" + date.substring(4, 6) + "-" + date.substring(2, 4) + "-" + date.substring(0, 2);
  utcTime += " ";

  // Process time with offset
  String time = parts[1];  // hhmmss
  if (time.length() >= 6) {
    int hh = time.substring(0, 2).toInt() + offsetHour;
    int mm = time.substring(2, 4).toInt() + offsetMin;
    int ss = time.substring(4, 6).toInt();
    
    debugPrint("UTC Time: " + parts[1] + " Offset: " + String(offsetHour) + "h " + String(offsetMin) + "m", DEBUG_VERBOSE);
    
    // Handle overflow
    if (mm >= 60) { mm -= 60; hh += 1; }
    if (hh >= 24) hh -= 24;
    if (mm < 0) { mm += 60; hh -= 1; }
    if (hh < 0) hh += 24;

    char buff[20];
    sprintf(buff, "%02d:%02d:%02d", hh, mm, ss);
    utcTime += buff;
    
    debugPrint("Final timestamp: " + utcTime, DEBUG_VERBOSE);
  } else {
    debugPrint("Invalid time format, discarding data", DEBUG_BASIC);
    return PARSE_INVALID_FORMAT;
  }

  return PARSE_SUCCESS;
}

uint16_t GeoLinker::sendData(float lat, float lon, String timestamp) {
  if (!isWiFiConnected()) return false;
  
  // Use new ArduinoJson API
  JsonDocument doc;
  doc["device_id"] = deviceID;

  // Use new API instead of createNestedArray
  doc["timestamp"].to<JsonArray>().add(timestamp);
  doc["lat"].to<JsonArray>().add(lat);
  doc["long"].to<JsonArray>().add(lon);
  
  if (batteryLevel <= 100) {
    doc["battery"].to<JsonArray>().add(batteryLevel);
  }

  if (!payloadMap.empty()) {
    JsonArray p = doc["payload"].to<JsonArray>();
    JsonObject pl = p.add<JsonObject>();
    for (const auto& kv : payloadMap) {
      pl[kv.first] = kv.second;
    }
  }

  String out;
  serializeJson(doc, out);

#if defined(ESP32) || defined(ESP8266)
  // ESP32/ESP8266 implementation
  HTTPClient http;
  #if defined(ESP8266)
    // ESP8266 requires WiFiClient for HTTPS
    WiFiClientSecure client;
    client.setInsecure(); // Skip SSL verification
    http.begin(client, "https://www.circuitdigest.cloud/geolinker");
  #else
    // ESP32 can use either method
    http.begin("https://www.circuitdigest.cloud/geolinker");
  #endif
  
  http.addHeader("Authorization", apiKey);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.POST(out);
  debugPrint("Payload: " + out, DEBUG_VERBOSE);
  debugPrint("HTTP code: " + String(httpCode), DEBUG_BASIC);
  http.end();
  return httpCode;

#elif defined(ARDUINO_UNOWIFIR4)
  WiFiSSLClient client; // Use WiFiClient for HTTP instead of HTTPS

  // Connect to server
  if (!client.connect("www.circuitdigest.cloud", 443)) { // Use port 80 for HTTP
    debugPrint("Connection failed", DEBUG_BASIC);
    return -1;
  }

  // Build and send HTTP POST request
  client.println("POST /geolinker HTTP/1.1");
  client.println("Host: www.circuitdigest.cloud");
  client.println("Authorization: " + apiKey);
  client.println("Content-Type: application/json");
  client.println("Content-Length: " + String(out.length()));
  client.println("Connection: close");
  client.println();
  client.print(out);

  debugPrint("Payload: " + out, DEBUG_BASIC);

  // Wait for response
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      debugPrint("Client timeout", DEBUG_BASIC);
      client.stop();
      return -1;
    }
  }

  // Read status line and extract HTTP code
  int httpCode = -1;
  if (client.available()) {
    String statusLine = client.readStringUntil('\n');
    int firstSpace = statusLine.indexOf(' ');
    int secondSpace = statusLine.indexOf(' ', firstSpace + 1);
    if (firstSpace != -1 && secondSpace != -1) {
      String statusCode = statusLine.substring(firstSpace + 1, secondSpace);
      httpCode = statusCode.toInt();
    }
  }

  debugPrint("HTTP code: " + String(httpCode), DEBUG_BASIC);
  client.stop();
  return httpCode;
#else
  // Placeholder for unsupported boards
  debugPrint("Unsupported board for HTTPS requests", DEBUG_BASIC);
  return 0;
#endif
}

void GeoLinker::storeOfflineData(float lat, float lon, String timestamp) {
  if (offlineBuffer.size() >= offlineLimit) {
    offlineBuffer.erase(offlineBuffer.begin());  // FIFO rollover
  }
  offlineBuffer.push_back({lat, lon, timestamp});
  debugPrint("Stored offline data. Size: " + String(offlineBuffer.size()), DEBUG_BASIC);
}

bool GeoLinker::sendOfflineData() {
    if (offlineBuffer.empty()) return false;

    bool atLeastOneSuccess = false;
    
    for (auto it = offlineBuffer.begin(); it != offlineBuffer.end(); ) {
        // Validate data before sending - Fixed isEmpty() issue
        if (!isValidGps(it->lat, it->lon) || it->timestamp.length() == 0) {
            debugPrint("Skipping invalid offline data", DEBUG_BASIC);
            it = offlineBuffer.erase(it);  // Remove invalid entries
            continue;
        }
        int httpCode = sendData(it->lat, it->lon, it->timestamp);
        
        if (httpCode == 200) {
            debugPrint("Sent offline data", DEBUG_BASIC);
            it = offlineBuffer.erase(it);
            atLeastOneSuccess = true;
        } 
        else {
            debugPrint("Failed to send (Code: " + String(httpCode) + ")", DEBUG_BASIC);
            ++it;  // Keep for retry (network/timeout issues)
        }

        delay(100); // Brief pause between sends
    }

    return atLeastOneSuccess;
}

// Helper function (add to your class)
bool GeoLinker::isValidGps(float lat, float lon) {
    return !(lat == 0.0 && lon == 0.0);  // Example validation
}

void GeoLinker::debugPrint(const String& msg, uint8_t level) {
  if (debugLevel >= level) {
    Serial.println("[GeoLinker] " + msg);
  }
}
void GeoLinker::debugPrintNoNewline(const String& msg, uint8_t level) {
  if (debugLevel >= level) {
    Serial.print(msg);
  }
}