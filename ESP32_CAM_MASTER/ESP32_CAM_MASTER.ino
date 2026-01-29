/*
/*
 * Smart Agricultural Disease Monitor - Optimized ESP32-CAM
 * Endpoints: /capture, /capture.jpg, /stream, /sensor
 * Includes: Preferences-based config UI, robust WiFi, reduced FB pressure
 */

// Ensure the ESP32 board package is installed and configured in the Arduino IDE.
// Added comments to guide the user on resolving the missing WiFi.h issue.
// No code changes are made here as the issue is related to the development environment setup.

#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Preferences.h>

// ========== MQTT Configuration ==========
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char* sensor_topic = "esp32/cam/sensors";
const char* status_topic = "esp32/cam/status";
const char* command_topic = "esp32/cam/command";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ============ CONFIGURATION ============
const char* ssid = "Arnabmandal";
const char* password = "hm403496";

// ============ GLOBALS ============
WebServer server(80);
camera_fb_t* capturedFrame = NULL;
Preferences prefs;

// Runtime network settings (may be persisted)
bool useStaticIP = false;
IPAddress staticIP;
IPAddress staticGateway;
IPAddress staticSubnet;
IPAddress staticDNS;

// AI Thinker ESP32-CAM pins
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define FLASH_LED_PIN      4
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

bool initCamera();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();

// ============ SETUP ============
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  // Serial now used for Arduino communication
  Serial.begin(115200);

  // Init Camera
  if (!initCamera()) {
    // Can't use Serial.print here anymore.
    // A more robust solution would be to blink an LED in an error pattern.
    ESP.restart();
  }

  // Read persisted network settings
  prefs.begin("cfg", false);
  String savedSsid = prefs.getString("ssid", "");
  String savedPass = prefs.getString("pass", "");
  useStaticIP = prefs.getBool("useStatic", false);
  if (useStaticIP) {
    String sip = prefs.getString("ip", "");
    String sg = prefs.getString("gw", "");
    String ss = prefs.getString("sn", "");
    String sd = prefs.getString("dns", "");
    if (sip.length()) staticIP.fromString(sip);
    if (sg.length()) staticGateway.fromString(sg);
    if (ss.length()) staticSubnet.fromString(ss);
    if (sd.length()) staticDNS.fromString(sd);
  }

  // WiFi Connection
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname("esp32-cam");
  WiFi.disconnect(true);
  delay(100);

  const char* wifi_ssid = (savedSsid.length() > 0) ? savedSsid.c_str() : ssid;
  const char* wifi_pass = (savedPass.length() > 0) ? savedPass.c_str() : password;

  if (useStaticIP && staticIP) {
    if (staticDNS) WiFi.config(staticIP, staticGateway, staticSubnet, staticDNS);
    else WiFi.config(staticIP, staticGateway, staticSubnet);
  }

  WiFi.begin(wifi_ssid, wifi_pass);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts++ < 60) { // Reduced timeout
    delay(500);
  }

  // Report WiFi connection status and provide a fallback AP for configuration
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed. Starting configuration AP 'ESP32-CAM-Setup'");
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("ESP32-CAM-Setup");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  }

  // MQTT Setup
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);

  // Web Server - Keep camera endpoints, remove sensor/pump
  server.on("/", [](){
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Location", "/stream");
    server.send(302, "text/plain", "Redirecting to stream");
  });
  server.on("/stream", handleStream);
  server.on("/capture", handleCapture);
  server.on("/capture.jpg", handleCaptureJPG);

  // Config endpoints are still useful
  server.on("/config", HTTP_GET, [](){
    String s_curSsid = prefs.getString("ssid", ssid);
    char curSsid[64] = {0};
    s_curSsid.toCharArray(curSsid, sizeof(curSsid));
    char page[512];
    snprintf(page, sizeof(page),
      "<html><body><h3>ESP32-CAM WiFi Config</h3>"
      "<form action='/saveconfig' method='POST'>"
      "SSID: <input name='ssid' value='%s'><br>"
      "Password: <input name='pass' value=''><br>"
      "<input type='submit' value='Save and Reboot'></form></body></html>",
      curSsid);
    server.send(200, "text/html", page);
  });

  server.on("/saveconfig", HTTP_POST, [](){
    prefs.putString("ssid", server.arg("ssid"));
    prefs.putString("pass", server.arg("pass"));
    server.send(200, "text/html", "Saved. Rebooting...");
    delay(500);
    ESP.restart();
  });
  
  server.on("/reboot", HTTP_GET, handleReboot);
  server.begin();
}

// ========== MQTT CALLBACK ==========
void callback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to a string
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(topic) == command_topic) {
    if (message == "PUMP_ON") {
      Serial.println("PUMP_ON");
    } else if (message == "PUMP_OFF") {
      Serial.println("PUMP_OFF");
    }
  }
}

// ========== MQTT RECONNECT ==========
void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    // Attempt to connect
    if (mqttClient.connect("ESP32-CAM-Client")) {
      mqttClient.publish(status_topic, "ESP32 Connected");
      // Subscribe to the command topic
      mqttClient.subscribe(command_topic);
      Serial.println("MQTT connected and subscribed to command topic");
    } else {
      // Wait 5 seconds before retrying
      Serial.println("MQTT connect failed, retrying in 5s");
      delay(5000);
    }
  }
}

// Helper: convert WiFi.status() code to human-readable text
const char* wifiStatusText(wl_status_t s) {
  switch (s) {
    case WL_NO_SHIELD: return "No shield";
    case WL_IDLE_STATUS: return "Idle";
    case WL_NO_SSID_AVAIL: return "No SSID available";
    case WL_SCAN_COMPLETED: return "Scan completed";
    case WL_CONNECTED: return "Connected";
    case WL_CONNECT_FAILED: return "Connect failed";
    case WL_CONNECTION_LOST: return "Connection lost";
    case WL_DISCONNECTED: return "Disconnected";
    default: return "Unknown";
  }
}

// Reboot endpoint
void handleReboot() {
  server.send(200, "text/plain", "Rebooting...");
  delay(200);
  ESP.restart();
}

// ============ MAIN LOOP ============
void loop() {
  server.handleClient();

  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      reconnect();
    }
    mqttClient.loop();

    // Check for data from Arduino
    if (Serial.available()) {
      String data = Serial.readStringUntil('\n');
      data.trim();
      
      // The data from Arduino is "temp:25.00,humidity:60.00,soil:500"
      // We will re-format it to a JSON string
      // {"temperature":25.00,"humidity":60.00,"soil":500,"arduinoConnected":true}
      
      int tempIndex = data.indexOf("temp:");
      int humidityIndex = data.indexOf(",humidity:");
      int soilIndex = data.indexOf(",soil:");
      
      if (tempIndex != -1 && humidityIndex != -1 && soilIndex != -1) {
        String tempStr = data.substring(tempIndex + 5, humidityIndex);
        String humidityStr = data.substring(humidityIndex + 10, soilIndex);
        String soilStr = data.substring(soilIndex + 6);
        
        char jsonBuffer[256];
        snprintf(jsonBuffer, sizeof(jsonBuffer), 
          "{\"temperature\":%s,\"humidity\":%s,\"soilMoisture\":%s,\"arduinoConnected\":true}", 
          tempStr.c_str(), humidityStr.c_str(), soilStr.c_str());
          
        mqttClient.publish(sensor_topic, jsonBuffer);
      }
    }
  }
  
  delay(10);
}

// ============ CAMERA INIT ============
bool initCamera() {
  Serial.println("Initializing camera...");
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    Serial.println("PSRAM found. Using optimized settings.");
    // Use SVGA for higher resolution when PSRAM is available and allow larger frame buffer
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 16; // balance quality vs size for better FPS
    config.fb_count = 3; // increase buffers to help maintain FPS during network I/O
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    Serial.println("PSRAM not found. Using fallback settings.");
    // Fallback: still request SVGA but keep conservative memory settings
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 18;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera initialization failed with error 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  // Ensure frame size is applied at sensor level
  s->set_framesize(s, FRAMESIZE_SVGA);
  s->set_vflip(s, 0);
  Serial.println("Camera initialized successfully.");
  return true;
}

// ============ STREAM HANDLER ============
void handleStream() {
  WiFiClient client = server.client();
  if (!client) return;
  client.setNoDelay(true);

  const char HEADER[] = "HTTP/1.1 200 OK\r\n"
                        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
                        "Access-Control-Allow-Origin: *\r\n"
                        "Cache-Control: no-cache\r\n"
                        "Connection: keep-alive\r\n\r\n";
  client.write(HEADER, sizeof(HEADER) - 1);

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera frame capture failed. Retrying...");
      delay(100);
      continue;
    }

    Serial.printf("Streaming frame len=%u\n", (unsigned)fb->len);

    if (fb->len > 0) {
      char lenBuf[64];
      int headerLen = snprintf(lenBuf, sizeof(lenBuf), "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", fb->len);
      int wroteHdr = client.write(lenBuf, headerLen);
      // Write image in chunks to avoid socket blocking on large single writes
      size_t wroteImg = 0;
      const uint8_t* data = (const uint8_t*)fb->buf;
      size_t remaining = fb->len;
      const size_t CHUNK = 1024;
      while (remaining && client.connected()) {
        size_t toWrite = (remaining > CHUNK) ? CHUNK : remaining;
        int r = client.write(data + wroteImg, toWrite);
        if (r > 0) { wroteImg += r; remaining -= r; }
        else break;
      }
      int wroteTail = client.write("\r\n", 2);
      Serial.printf("Wrote header=%d/%d img=%d/%u tail=%d/2\n", wroteHdr, headerLen, wroteImg, (unsigned)fb->len, wroteTail);
      if (wroteHdr != headerLen || wroteImg != fb->len || wroteTail != 2) {
        Serial.println("Client write failed, ending stream.");
        esp_camera_fb_return(fb);
        break;
      } else {
        Serial.println("Stream frame sent successfully.");
      }
    }

    esp_camera_fb_return(fb);
    yield();
  }

  Serial.println("Client disconnected.");
}

// ============ CAPTURE HANDLERS ============
void handleCapture() {
  Serial.println("HTTP /capture requested");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (capturedFrame) {
    esp_camera_fb_return(capturedFrame);
    capturedFrame = NULL;
  }
  capturedFrame = esp_camera_fb_get();
  if (capturedFrame) {
    server.send(200, "text/plain", "OK");
  } else {
    server.send(500, "text/plain", "Failed to capture frame");
  }
}

void handleCaptureJPG() {
  Serial.println("HTTP /capture.jpg requested");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  bool usedLocal = false;
  if (!capturedFrame) {
    // Capture on-demand so clients can request the image directly
    capturedFrame = esp_camera_fb_get();
    usedLocal = true;
  }
  if (!capturedFrame) {
    server.send(500, "text/plain", "Failed to capture frame");
    return;
  }
  WiFiClient client = server.client();
  if (!client) {
    if (usedLocal) { esp_camera_fb_return(capturedFrame); capturedFrame = NULL; }
    return;
  }
  client.setNoDelay(true);
  char hdr[128];
  int hdrLen = snprintf(hdr, sizeof(hdr), "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\nAccess-Control-Allow-Origin: *\r\n\r\n", (unsigned)capturedFrame->len);
      if (hdrLen > 0) {
        int hw = client.write(hdr, hdrLen);
        Serial.printf("Header write: %d/%d\n", hw, hdrLen);
      }
      // Send image in chunks to avoid large single socket writes
      size_t written = 0;
      const uint8_t* cbuf = (const uint8_t*)capturedFrame->buf;
      size_t remaining = capturedFrame->len;
      const size_t CHUNK = 1024;
      while (remaining && client.connected()) {
        size_t toWrite = (remaining > CHUNK) ? CHUNK : remaining;
        int r = client.write(cbuf + written, toWrite);
        if (r > 0) { written += r; remaining -= r; } else break;
      }
      Serial.printf("Image write: %u/%u\n", (unsigned)written, (unsigned)capturedFrame->len);
  if (usedLocal) {
    esp_camera_fb_return(capturedFrame);
    capturedFrame = NULL;
  } else {
    // If this image came from a prior /capture, return the FB to free memory
    esp_camera_fb_return(capturedFrame);
    capturedFrame = NULL;
  }
}
