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

#include <GeoLinker.h>
GeoLinker geo; // Create GeoLinker instance

// ==================================================================
//                   HARDWARE CONFIGURATION
// ==================================================================

/* ----- Option 1: ESP32 with Custom Pins ----- */
HardwareSerial GPS(1);  // Using Serial1
#define GPS_RX 16       // GPIO16 for RX
#define GPS_TX 17       // GPIO17 for TX

/* ----- Option 2: Standard Hardware Serial (for Uno R4 WiFi, Pico W) ----- */
// HardwareSerial& GPS = Serial1;  // Uses default pins:
                                  // Uno R4 WiFi: RX= D0, TX= D1
                                  // Pico W/2W: TX= 0, RX= 1 

/* ----- Option 3: Software Serial (for ESP8266 etc) ----- */
// #include <SoftwareSerial.h>
// #define GPS_RX 14      // Custom RX pin
// #define GPS_TX 12      // Custom TX pin
// SoftwareSerial GPS(GPS_RX, GPS_TX);  // RX, TX pins (avoid conflict pins)

// Common GPS Settings
#define GPS_BAUD 9600   // Standard NMEA baud rate

// ==================================================================
//                   NETWORK CONFIGURATION
// ==================================================================
const char* ssid = "WiFi_SSID";       // Your network name
const char* password = "WiFi_Password"; // Your network password

// ==================================================================
//                   GeoLinker CONFIGURATION
// ==================================================================
const char* apiKey = "FhBxxxxxxxxx";  // Your GeoLinker API key
const char* deviceID = "weather_tracker"; // Unique device identifier
const uint16_t updateInterval = 10;   // Data upload interval (seconds)
const DebugLevel debugLevel = DEBUG_VERBOSE; //// Debug verbosity: DEBUG_NONE, DEBUG_BASIC, DEBUG_VERBOSE
const bool enableOfflineStorage = true; // Enable offline data storage
const bool enableAutoReconnect = true;  // Enable auto-reconnect
const int8_t timeOffsetHours = 5;      // Timezone hours (e.g., 5 for IST (5.30))
const int8_t timeOffsetMinutes = 30;   // Timezone minutes (e.g., 30 for IST (5.30))

// ==================================================================
//                   SETUP FUNCTION
// ==================================================================
void setup() {
  // initialise debug serial
  Serial.begin(115200);
  delay(1000);  // Give serial monitor time to connect

  // initialise GPS connection
  GPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX); // For custom hardware serial
  // GPS.begin(GPS_BAUD);  // For predefined hardware/software serial

  // initialise GeoLinker service
  geo.begin(GPS);

  /* Configure GeoLinker parameters */
  geo.setApiKey(apiKey);
  geo.setDeviceID(deviceID);
  geo.setUpdateInterval_seconds(updateInterval);
  geo.setDebugLevel(debugLevel);
  geo.enableOfflineStorage(enableOfflineStorage);
  geo.enableAutoReconnect(enableAutoReconnect);
  geo.setTimeOffset(timeOffsetHours, timeOffsetMinutes);
  geo.setWiFiCredentials(ssid, password);// initialise WiFi

  //  Establish WiFi connection
if (!geo.connectToWiFi()) {
Serial.println("Failed to connect to WiFi");
}

  Serial.println("GeoLinker setup complete");
}

// ==================================================================
//                   MAIN PROGRAM LOOP
// ==================================================================
void loop() {
  // Example sensor payloads (optional - up to 5 payloads)
  geo.setPayloads({
    {"temperature", 25.0},
    {"humidity", 60.0}
  });

  // Example battery level (optional)
  geo.setBatteryLevel(87);
  
  // ==========================================
  //         GEO LINKER OPERATION
  // ==========================================
  uint8_t status = geo.loop();
  
  if (status > 0) { 
    Serial.print("Status: ");
    switch(status) {
      case 1:
        Serial.println("Data sent successfully to GeoLinker!");
        break;
      case 2:
        Serial.println("GPS connection error!");
        break;
      case 3:
        Serial.println("Connection error - data saved to offline buffer!");
        break;
      case 4:
        Serial.println("Data format error!");
        break;
      case 5:
        Serial.println("Invalid GPS data (cold boot may need time)!");
        break;
      default:
        Serial.println("Unknown status code");
    }
  }
}
