// Communication with ESP32 will be over SoftwareSerial (Pins 8, 9)
// Hardware Serial (Pins 0, 1) will be used for Debugging
#include <Arduino.h>
#include <string.h>
#include <DHT.h>
#include <SoftwareSerial.h>

// RX=8, TX=9
SoftwareSerial espSerial(8, 9);

// DHT22 Sensor
#define DHTPIN 4          // DHT22 connected to pin 4
#define DHTTYPE DHT22     // DHT22 (AM2302)
DHT dht(DHTPIN, DHTTYPE);

// Moisture Sensor
const int MOISTURE_PIN = A3;
// Calibration: Measure raw value in air (dry) and in water/wet soil
// Your sensor reads ~1023 in dry air, typical capacitive sensors read ~400-500 in wet soil
const int AIR_VALUE = 1023;   // Calibration value for dry air (raw analog reading)
const int WATER_VALUE = 400;  // Calibration value for wet soil (adjust based on your soil)

// Actuator pin
const int pumpPin = 10; // Water pump connected to pin 10

void setup() {
  // Start the hardware serial for debugging
  Serial.begin(115200);

  // Start SoftwareSerial for ESP32 communication at 9600 baud (More reliable for SoftSerial)
  espSerial.begin(9600);

  // Initialize DHT22 sensor
  dht.begin();

  // Debug startup message
  Serial.println("ARDUINO: started, awaiting commands");

  // Set up the pump pin as an output
  pinMode(pumpPin, OUTPUT);
  digitalWrite(pumpPin, LOW); // Default to off
}

// Small, heap-free serial command parser
const unsigned long ALIVE_INTERVAL_MS = 5000;
const unsigned long SENSOR_INTERVAL_MS = 60000; // Read sensor every 60 seconds
static unsigned long lastAliveMillis = 0;
static unsigned long lastSensorMillis = 0;
char cmdBuf[64];
size_t cmdLen = 0;

void handleCommand(const char *cmd) {
  // Trim leading/trailing whitespace
  const char *s = cmd;
  while (*s == ' ' || *s == '\t') s++;
  size_t len = strlen(s);
  while (len > 0 && (s[len-1] == ' ' || s[len-1] == '\t' || s[len-1] == '\r' || s[len-1] == '\n')) len--;

  if (len == 0) return;

  // Copy to a local buffer to operate on
  char t[64];
  size_t copyLen = (len < sizeof(t)-1) ? len : sizeof(t)-1;
  memcpy(t, s, copyLen);
  t[copyLen] = '\0';

  Serial.print("CMD RX: "); Serial.println(t); // Debug to PC

  if (strcmp(t, "PUMP_ON") == 0) {
    digitalWrite(pumpPin, HIGH);
    Serial.println("ACTION: pump -> ON");
  } else if (strcmp(t, "PUMP_OFF") == 0) {
    digitalWrite(pumpPin, LOW);
    Serial.println("ACTION: pump -> OFF");
  } else if (strncmp(t, "WIFI_OK:", 8) == 0) {
    Serial.print("✅ ESP32 WiFi: ");
    Serial.println(t + 8);  // Print IP address
  } else if (strcmp(t, "WIFI_FAILED") == 0) {
    Serial.println("❌ ESP32 WiFi FAILED");
  } else if (strcmp(t, "MQTT_OK") == 0) {
    Serial.println("✅ ESP32 MQTT Connected");
  } else if (strcmp(t, "SYSTEM_READY") == 0) {
    Serial.println("✅ ESP32 System Ready!");
  }
}

void loop() {
  // Non-blocking read of serial data into cmdBuf
  while (espSerial.available()) {
    int c = espSerial.read();
    if (c < 0) break;
    if (c == '\r') continue; // ignore CR
    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      if (cmdLen > 0) handleCommand(cmdBuf);
      cmdLen = 0;
    } else {
      if (cmdLen < sizeof(cmdBuf)-1) {
        cmdBuf[cmdLen++] = (char)c;
      } else {
        // buffer overflow, reset
        cmdLen = 0;
      }
    }
  }

  // Read and send sensor data every 60 seconds
  unsigned long now = millis();
  if (now - lastSensorMillis >= SENSOR_INTERVAL_MS) {
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    
    // Read Moisture
    int moistureRaw = analogRead(MOISTURE_PIN);
    int moisturePercent = map(moistureRaw, AIR_VALUE, WATER_VALUE, 0, 100);
    moisturePercent = constrain(moisturePercent, 0, 100);

    // Check if readings are valid
    if (!isnan(humidity) && !isnan(temperature)) {
      // Send sensor data in format: SENSOR:temp,humidity,moisture
      espSerial.print("SENSOR:");
      espSerial.print(temperature, 1);
      espSerial.print(",");
      espSerial.print(humidity, 1);
      espSerial.print(",");
      espSerial.print(moisturePercent);
      espSerial.print(",");
      espSerial.println(moistureRaw);
      
      // Duplicate to USB for debugging
      Serial.print("DEBUG TX: SENSOR:");
      Serial.print(temperature, 1);
      Serial.print(",");
      Serial.print(humidity, 1);
      Serial.print(",");
      Serial.print(moisturePercent);
      Serial.print(",");
      Serial.println(moistureRaw);
    } else {
      espSerial.println("SENSOR:ERROR");
      Serial.println("DEBUG: Sensor read error");
    }
    lastSensorMillis = now;
  }
}
