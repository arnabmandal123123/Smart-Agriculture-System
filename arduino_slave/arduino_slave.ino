// Communication with ESP32 will be over the main hardware serial port (TX/RX pins 0, 1)
// const int rxPin = 8;
// const int txPin = 9;
// SoftwareSerial espSerial(rxPin, txPin); // RX, TX

// Actuator pin
const int pumpPin = 10; // Water pump connected to pin 10

void setup() {
  // Start the hardware serial for communication with ESP32
  // Match baud with ESP32 firmware (115200)
  Serial.begin(115200);

  // Debug startup message
  Serial.println("ARDUINO: started, awaiting commands");

  // Set up the pump pin as an output
  pinMode(pumpPin, OUTPUT);
  digitalWrite(pumpPin, LOW); // Default to off
}

// Small, heap-free serial command parser
const unsigned long ALIVE_INTERVAL_MS = 5000;
static unsigned long lastAliveMillis = 0;
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

  Serial.print("CMD RX: "); Serial.println(t);

  if (strcmp(t, "PUMP_ON") == 0) {
    digitalWrite(pumpPin, HIGH);
    Serial.println("ACTION: pump -> ON");
  } else if (strcmp(t, "PUMP_OFF") == 0) {
    digitalWrite(pumpPin, LOW);
    Serial.println("ACTION: pump -> OFF");
  }
}

void loop() {
  // Non-blocking read of serial data into cmdBuf
  while (Serial.available()) {
    int c = Serial.read();
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

  // Periodic presence ping (rate-limited)
  unsigned long now = millis();
  if (now - lastAliveMillis >= ALIVE_INTERVAL_MS) {
    Serial.println("ARDUINO_ALIVE");
    lastAliveMillis = now;
  }
}
