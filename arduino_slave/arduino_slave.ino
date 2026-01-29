// Communication with ESP32 will be over the main hardware serial port (TX/RX pins 0, 1)
// const int rxPin = 8;
// const int txPin = 9;
// SoftwareSerial espSerial(rxPin, txPin); // RX, TX

// Sensor and actuator pins
const int dhtPin = 7; // Assuming a DHT11/22 is connected to pin 7
const int soilPin = A0; // Soil moisture sensor on A0
const int pumpPin = 4; // Water pump connected to pin 4

void setup() {
  // Start the hardware serial for communication with ESP32
  Serial.begin(9600);

  // Set up the pump pin as an output
  pinMode(pumpPin, OUTPUT);
  digitalWrite(pumpPin, LOW); // Default to off
}

void loop() {
  // Read sensor data (placeholders)
  float temperature = 25.0; // Placeholder
  float humidity = 60.0;    // Placeholder
  int soilMoisture = 500;   // Placeholder

  // Create a data string to send to the ESP32
  String dataString = "temp:" + String(temperature) + ",humidity:" + String(humidity) + ",soil:" + String(soilMoisture);
  
  // Send the data to the ESP32
  Serial.println(dataString);
  
  // Check for incoming commands from the ESP32
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "PUMP_ON") {
      digitalWrite(pumpPin, HIGH);
    } else if (command == "PUMP_OFF") {
      digitalWrite(pumpPin, LOW);
    }
  }
  
  delay(2000); // Send data every 2 seconds
}
