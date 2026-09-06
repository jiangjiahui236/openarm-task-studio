#if !defined(ARDUINO_ARCH_ESP32)
#error "Wrong board/core: install 'esp32 by Espressif Systems' and select an ESP32-C3 board."
#else

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>

const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* TASK_STUDIO_SERVICE = "openarm-record";
const char* TASK_STUDIO_FALLBACK_IP = "";
const uint16_t TASK_STUDIO_PORT = 5011;
const int BUTTON_PIN = 4;
const unsigned long DEBOUNCE_MS = 20;
const unsigned long DISCOVERY_RETRY_MS = 5000;
const unsigned long SEND_RETRY_MS = 250;
const unsigned long ACK_TIMEOUT_MS = 5000;
const unsigned long HEARTBEAT_MS = 1000;
const unsigned long CONNECTION_TIMEOUT_MS = 5000;

WiFiUDP udp;
IPAddress taskStudioIp;
uint16_t taskStudioPort = TASK_STUDIO_PORT;
bool stableButtonState = HIGH;
bool lastReading = HIGH;
unsigned long lastChangeMs = 0;
unsigned long lastDiscoveryMs = 0;
unsigned long pendingSinceMs = 0;
unsigned long lastSendMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastPongMs = 0;
uint32_t nextSequence = 1;
uint32_t pendingSequence = 0;

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
  }
  MDNS.begin("openarm-remote-button");
  taskStudioIp = IPAddress();
  lastDiscoveryMs = 0;
}

bool findTaskStudio() {
  int services = MDNS.queryService(TASK_STUDIO_SERVICE, "udp");
  IPAddress localIp = WiFi.localIP();
  IPAddress subnetMask = WiFi.subnetMask();
  for (int index = 0; index < services; ++index) {
    IPAddress candidate = MDNS.IP(index);
    bool sameSubnet = true;
    for (int octet = 0; octet < 4; ++octet) {
      if ((candidate[octet] & subnetMask[octet]) != (localIp[octet] & subnetMask[octet])) {
        sameSubnet = false;
        break;
      }
    }
    Serial.print("mDNS candidate ");
    Serial.print(candidate);
    Serial.println(sameSubnet ? " is on the Wi-Fi subnet" : " ignored (wrong subnet)");
    if (!sameSubnet) {
      continue;
    }
    taskStudioIp = candidate;
    taskStudioPort = MDNS.port(index);
    lastPongMs = millis();
    Serial.print("Task Studio discovered at ");
    Serial.print(taskStudioIp);
    Serial.print(":");
    Serial.println(taskStudioPort);
    return true;
  }
  if (TASK_STUDIO_FALLBACK_IP[0] != '\0' && taskStudioIp.fromString(TASK_STUDIO_FALLBACK_IP)) {
    taskStudioPort = TASK_STUDIO_PORT;
    return true;
  }
  return false;
}

void sendRecordCommand() {
  if (taskStudioIp == IPAddress()) {
    return;
  }
  char command[32];
  snprintf(command, sizeof(command), "RECORD:%lu", static_cast<unsigned long>(pendingSequence));
  for (int attempt = 0; attempt < 3; ++attempt) {
    udp.beginPacket(taskStudioIp, taskStudioPort);
    udp.print(command);
    udp.endPacket();
    if (attempt < 2) {
      delay(8);
    }
  }
  lastSendMs = millis();
}

void beginRecordCommand() {
  pendingSequence = nextSequence++;
  if (nextSequence == 0) {
    nextSequence = 1;
  }
  pendingSinceMs = millis();
  lastSendMs = 0;
  sendRecordCommand();
}

void pollAcknowledgement() {
  int packetSize = udp.parsePacket();
  while (packetSize > 0) {
    char response[32] = {};
    int length = udp.read(response, sizeof(response) - 1);
    if (length > 0) {
      if (strcmp(response, "PONG") == 0) {
        lastPongMs = millis();
      }
      unsigned long acknowledged = 0;
      if (sscanf(response, "ACK:%lu", &acknowledged) == 1 && acknowledged == pendingSequence) {
        Serial.print("RECORD acknowledged: ");
        Serial.println(acknowledged);
        pendingSequence = 0;
      }
    }
    packetSize = udp.parsePacket();
  }
}

void sendHeartbeat() {
  if (taskStudioIp == IPAddress()) {
    return;
  }
  udp.beginPacket(taskStudioIp, taskStudioPort);
  udp.print(stableButtonState == LOW ? "PING:DOWN" : "PING:UP");
  udp.endPacket();
  lastHeartbeatMs = millis();
}

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  connectWifi();
  udp.begin(0);
  findTaskStudio();
  lastDiscoveryMs = millis();
  lastPongMs = millis();
  Serial.print("Remote record button ready, IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();
    findTaskStudio();
    lastDiscoveryMs = millis();
  }
  pollAcknowledgement();
  unsigned long now = millis();
  if (taskStudioIp == IPAddress() && now - lastDiscoveryMs >= DISCOVERY_RETRY_MS) {
    findTaskStudio();
    lastDiscoveryMs = millis();
    if (pendingSequence != 0 && taskStudioIp != IPAddress()) {
      sendRecordCommand();
    }
  }
  if (taskStudioIp != IPAddress() && now - lastHeartbeatMs >= HEARTBEAT_MS) {
    sendHeartbeat();
  }
  if (taskStudioIp != IPAddress() && now - lastPongMs >= CONNECTION_TIMEOUT_MS) {
    Serial.println("Heartbeat timeout; rediscovering Task Studio");
    taskStudioIp = IPAddress();
    lastDiscoveryMs = now - DISCOVERY_RETRY_MS;
    lastPongMs = now;
  }
  if (pendingSequence != 0 && taskStudioIp != IPAddress() && now - lastSendMs >= SEND_RETRY_MS) {
    sendRecordCommand();
  }
  if (pendingSequence != 0 && now - pendingSinceMs >= ACK_TIMEOUT_MS) {
    Serial.println("No ACK; rediscovering Task Studio");
    taskStudioIp = IPAddress();
    lastDiscoveryMs = now - DISCOVERY_RETRY_MS;
    pendingSinceMs = now;
  }
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastReading) {
    lastChangeMs = millis();
    lastReading = reading;
  }
  if (millis() - lastChangeMs >= DEBOUNCE_MS && reading != stableButtonState) {
    stableButtonState = reading;
    if (stableButtonState == LOW) {
      beginRecordCommand();
    } else {
      sendHeartbeat();
    }
  }
  delay(1);
}

#endif
