# GeoLinker
GeoLinker Cloud API Library

A comprehensive Arduino library for GPS tracking and cloud data transmission, designed for IoT applications. The GeoLinker library enables seamless integration between GPS modules and GeoLinker cloud services with support for offline data storage, automatic reconnection, and multiple microcontroller platforms.

## 🚀 Features

- **Multi-Platform Support**: ESP32, ESP8266, Arduino Uno R4 WiFi, Raspberry Pi Pico W, and Raspberry Pi Pico 2 W
- **GPS Integration**: NMEA sentence parsing with coordinate conversion
- **Cloud Connectivity**: Seamless communication with GeoLinker Cloud API
- **Offline Storage**: Automatic data buffering when the network is unavailable (up to 100 entries)
- **Auto-Reconnection**: WiFi reconnection handling
- **Timezone Support**: Configurable time offset for local timezone
- **Custom Payloads**: Support for additional sensor data (up to 5 parameters)
- **Battery Monitoring**: Optional battery level reporting
- **Debug Levels**: Comprehensive debugging with multiple verbosity levels

## 📋 Requirements

### Hardware
- Compatible microcontroller (ESP32, ESP8266, Arduino Uno R4 WiFi, Raspberry Pi Pico W/2W, etc.)
- GPS module with NMEA output capability
- WiFi connectivity

### Software Dependencies
- **ArduinoJson** library (version 7.x or later)
- Platform-specific WiFi libraries (automatically included)

## 🔧 Installation

### Method 1: Arduino Library Manager (Recommended)
1. **Open Arduino IDE**
2. **Navigate to Library Manager**
   ```
   Tools → Manage Libraries... (or Ctrl+Shift+I)
   ```
3. **Search for GeoLinker**
   - Type "GeoLinker" in the search box
   - Click "Install" on the GeoLinker library by Jobit Joseph
4. **Install Dependencies**
   - The ArduinoJson dependency should install automatically
   - If not, search for "ArduinoJson" and install it manually (version 7.x or later)
5. **Include in Your Sketch**
   ```cpp
   #include <GeoLinker.h>
   ```

### Method 2: Manual Installation
1. **Download Library Files**
   - Download `GeoLinker.h` and `GeoLinker.cpp` from the repository
   - Create a folder named `GeoLinker` in your Arduino libraries directory

2. **Locate Arduino Libraries Folder**
   - **Windows**: `Documents\Arduino\libraries\`
   - **macOS**: `~/Documents/Arduino/libraries/`
   - **Linux**: `~/Arduino/libraries/`

3. **Install Library Files**
   - Copy `GeoLinker.h` and `GeoLinker.cpp` into `Arduino/libraries/GeoLinker/`
   - Your folder structure should look like:
     ```
     Arduino/libraries/GeoLinker/
     ├── GeoLinker.h
     └── GeoLinker.cpp
     ```

4. **Install Dependencies**
   ```
   Tools → Manage Libraries → Search for "ArduinoJson" → Install (version 7.x or later)
   ```

5. **Include in Your Sketch**
   ```cpp
   #include <GeoLinker.h>
   ```

## 🏗️ Hardware Setup

### Option 1: ESP32 with Custom Pins
```cpp
HardwareSerial GPS(1);  // Using Serial1
#define GPS_RX 16       // GPIO16 for RX
#define GPS_TX 17       // GPIO17 for TX

void setup() {
    GPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}
```

### Option 2: Standard Hardware Serial (Uno R4 WiFi, Pico W, Pico 2W)
```cpp
HardwareSerial& GPS = Serial1;  // Default pins:
                               // Uno R4 WiFi: RX=D0, TX=D1
                               // Pico W/2W: TX=0, RX=1

void setup() {
    GPS.begin(9600);
}
```

### Option 3: Software Serial (ESP8266)
```cpp
#include <SoftwareSerial.h>
#define GPS_RX 14
#define GPS_TX 12
SoftwareSerial GPS(GPS_RX, GPS_TX);

void setup() {
    GPS.begin(9600);
}
```

## 📝 Basic Usage

### Minimal Example
```cpp
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
const char* apiKey = "FhBOTY5zL7SE";  // Your GeoLinker API key
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
```

## 🔧 API Reference

### Core Methods

#### `bool begin(Stream& serial)`
Initialise the library with GPS serial connection.
- **Parameters**: `serial` - GPS serial interface
- **Returns**: `true` if successful

#### `void setApiKey(const String& key)`
Set your GeoLinker API key.
- **Parameters**: `key` - Your API key from GeoLinker dashboard

#### `void setDeviceID(const String& id)`
Set unique device identifier.
- **Parameters**: `id` - Unique device name/ID

#### `void setWiFiCredentials(const char* ssid, const char* password)`
Configure WiFi network credentials.
- **Parameters**: 
  - `ssid` - WiFi network name
  - `password` - WiFi password

#### `bool connectToWiFi()`
Establish WiFi connection.
- **Returns**: `true` if connected successfully

#### `uint8_t loop()`
Main library operation - call in your main loop.
- **Returns**: Status code (see Status Codes section)

### Configuration Methods

#### `void setUpdateInterval_seconds(uint32_t interval_s)`
Set data upload interval.
- **Parameters**: `interval_s` - Interval in seconds (default: 60)

#### `void setDebugLevel(uint8_t level)`
Set debug verbosity level.
- **Parameters**: `level` - `DEBUG_NONE` (0), `DEBUG_BASIC` (1), `DEBUG_VERBOSE` (2)

#### `void enableOfflineStorage(bool enable)`
Enable/disable offline data buffering.
- **Parameters**: `enable` - `true` to enable offline storage

#### `void enableAutoReconnect(bool enable)`
Enable/disable automatic WiFi reconnection.
- **Parameters**: `enable` - `true` to enable auto-reconnect

#### `void setTimeOffset(int hourOffset, int minuteOffset)`
Set timezone offset from UTC.
- **Parameters**: 
  - `hourOffset` - Hours offset (-12 to +12)
  - `minuteOffset` - Minutes offset (0, 15, 30, 45)

#### `void setPayloads(const std::map<String, float>& payloads)`
Set additional sensor data to transmit.
- **Parameters**: `payloads` - Map of sensor names and values (max 5 entries)

#### `void setBatteryLevel(uint8_t percent)`
Set battery level for reporting.
- **Parameters**: `percent` - Battery level (0-100%)

## 📊 Status Codes

The `loop()` method returns status codes indicating operation results:

| Code | Status | Description |
|------|--------|-------------|
| 0 | Normal | No action taken (waiting for next update) |
| 1 | Success | Data sent successfully to cloud |
| 2 | GPS Error | No GPS data or GPS connection issues |
| 3 | Network Error | Connection failed, data stored offline |
| 4 | Auth Error | API authentication failed |
| 5 | Parse Error | Invalid GPS data format |

## 🐛 Debugging

### Debug Levels

```cpp
geo.setDebugLevel(DEBUG_NONE);     // No debug output
geo.setDebugLevel(DEBUG_BASIC);    // Basic status messages
geo.setDebugLevel(DEBUG_VERBOSE);  // Detailed GPS and network info
```

### Common Issues

**GPS Not Working**
- Check wiring connections
- Ensure GPS module has clear sky view
- Wait for GPS cold start (may take several minutes)
- Enable verbose debugging to see raw NMEA data

**WiFi Connection Issues**
- Verify SSID and password
- Check signal strength
- Enable auto-reconnect for unstable connections

**Cloud Upload Failures**
- Verify API key is correct
- Check internet connectivity
- Enable offline storage for reliability

## 🌐 Cloud Integration

### Data Format
The library sends JSON data to the GeoLinker cloud service:

```json
{
  "device_id": "weather_tracker",
  "timestamp": ["2025-06-04 14:30:45"],
  "lat": [12.9716],
  "long": [77.5946],
  "battery": [87],
  "payload": [{
    "temperature": 25.5,
    "humidity": 60.0
  }]
}
```

### API Endpoint
- **URL**: `https://www.circuitdigest.cloud/geolinker`
- **Method**: POST
- **Headers**: 
  - `Authorization: YOUR_API_KEY`
  - `Content-Type: application/json`

## 🔒 License

This library is licensed under the MIT License. See the license header in source files for full details.

**Copyright (C) 2025 Jobit Joseph, Semicon Media Pvt Ltd (Circuit Digest)**

## 👨‍💻 Author

**Jobit Joseph**  
Semicon Media Pvt Ltd (Circuit Digest)



## 🔄 Version History

- **v1.0.0** - Initial release with multi-platform support
- Features: GPS tracking, cloud integration, offline storage, auto-reconnection

---
