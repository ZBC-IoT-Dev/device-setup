#include <WiFiS3.h>
#include <PubSubClient.h>
#include <EEPROM.h>

// ---------- USER CONFIG ----------
const char* WIFI_SSID     = "SSID";
const char* WIFI_PASSWORD = "PASSWORD";
const char* MQTT_HOST     = "10.106.189.237";
const uint16_t MQTT_PORT  = 1883;
const char* DEVICE_TYPE   = "LightSwitch";

// ---------- LED/LIGHT CONFIG ----------
// D7 -> 220-330 ohm resistor -> LED anode (+), LED cathode (-) -> GND
const uint8_t LIGHT_PIN = 7;
// Breadboard LED is active HIGH (HIGH = ON).
const bool RELAY_ACTIVE_LOW = false;

// ---------- TIMING ----------
const unsigned long STATE_HEARTBEAT_MS = 30000;

// ---------- MQTT TOPICS ----------
const char* TOPIC_DISCOVERY    = "discovery/announce";
const char* TOPIC_HUB_ANNOUNCE = "hub/announce";

// ---------- EEPROM LAYOUT (UNO R4) ----------
const int HUB_ID_ADDR   = 0;
const int HUB_ID_MAXLEN = 64;

// ---------- GLOBALS ----------
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

String deviceId;
String rememberedHubId;
String topicSet;
String topicState;

unsigned long lastPublishMs   = 0;
unsigned long lastWifiRetryMs = 0;
unsigned long lastMqttRetryMs = 0;

unsigned long wifiRetryDelayMs = 1000;  // max 60s
unsigned long mqttRetryDelayMs = 1000;  // max 60s

bool lightOn = false;

// ---------- EEPROM HELPERS ----------
void eepromWriteString(int addr, const String& value, int maxLen) {
  int len = value.length();
  if (len > maxLen - 1) len = maxLen - 1;

  EEPROM.write(addr, (uint8_t)len);
  for (int i = 0; i < len; i++) {
    EEPROM.write(addr + 1 + i, (uint8_t)value[i]);
  }
  EEPROM.write(addr + 1 + len, 0);
}

String eepromReadString(int addr, int maxLen) {
  int len = EEPROM.read(addr);
  if (len < 0 || len > maxLen - 1) len = 0;

  char buf[HUB_ID_MAXLEN];
  for (int i = 0; i < len; i++) {
    buf[i] = (char)EEPROM.read(addr + 1 + i);
  }
  buf[len] = '\0';
  return String(buf);
}

void saveHubId(const String& hubId) {
  rememberedHubId = hubId;
  eepromWriteString(HUB_ID_ADDR, hubId, HUB_ID_MAXLEN);

  Serial.print("[Hub] Remembered new hubId: ");
  Serial.println(hubId);
}

// ---------- ID ----------
String buildStableDeviceId() {
  byte mac[6] = {0, 0, 0, 0, 0, 0};
  WiFi.macAddress(mac);

  char out[32];
  snprintf(
    out,
    sizeof(out),
    "uno-r4-%02X%02X%02X%02X%02X%02X",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
  );
  return String(out);
}

// ---------- LIGHT HELPERS ----------
void writeLightPin(bool on) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(LIGHT_PIN, on ? LOW : HIGH);
  } else {
    digitalWrite(LIGHT_PIN, on ? HIGH : LOW);
  }
}

void setLightState(bool on) {
  lightOn = on;
  writeLightPin(lightOn);
}

bool parseDesiredState(const String& body, bool& outState) {
  String v = body;
  v.trim();
  v.toLowerCase();

  if (v == "on" || v == "1" || v == "true") {
    outState = true;
    return true;
  }
  if (v == "off" || v == "0" || v == "false") {
    outState = false;
    return true;
  }

  int stateKey = v.indexOf("\"state\"");
  if (stateKey >= 0) {
    int colon = v.indexOf(':', stateKey);
    int q1 = v.indexOf('"', colon + 1);
    int q2 = v.indexOf('"', q1 + 1);
    if (colon >= 0 && q1 >= 0 && q2 > q1) {
      String stateText = v.substring(q1 + 1, q2);
      stateText.trim();
      if (stateText == "on") {
        outState = true;
        return true;
      }
      if (stateText == "off") {
        outState = false;
        return true;
      }
    }
  }

  int isOnKey = v.indexOf("\"ison\"");
  if (isOnKey >= 0) {
    int colon = v.indexOf(':', isOnKey);
    if (colon >= 0) {
      int one = v.indexOf('1', colon + 1);
      int zero = v.indexOf('0', colon + 1);
      if (one >= 0 && (zero < 0 || one < zero)) {
        outState = true;
        return true;
      }
      if (zero >= 0 && (one < 0 || zero < one)) {
        outState = false;
        return true;
      }
    }
  }

  return false;
}

// ---------- TELEMETRY ----------
String buildStatePayload() {
  String stateText = lightOn ? "ON" : "OFF";

  String payload = "{";
  payload += "\"id\":\"" + deviceId + "\",";
  payload += "\"type\":\"" + String(DEVICE_TYPE) + "\",";
  payload += "\"hubId\":\"" + rememberedHubId + "\",";
  payload += "\"state\":\"" + stateText + "\",";
  payload += "\"isOn\":" + String(lightOn ? 1 : 0) + ",";
  payload += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  payload += "\"uptimeMs\":" + String(millis());
  payload += "}";
  return payload;
}

void publishState() {
  String payload = buildStatePayload();
  bool okState = mqttClient.publish(topicState.c_str(), payload.c_str(), true);
  bool okDiscovery = mqttClient.publish(TOPIC_DISCOVERY, payload.c_str(), false);

  if (okState && okDiscovery) {
    Serial.print("[MQTT] State sent: ");
    Serial.println(payload);
  } else {
    Serial.println("[MQTT] State publish failed");
  }
}

// ---------- MQTT CALLBACK ----------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr(topic);

  String body;
  body.reserve(length);
  for (unsigned int i = 0; i < length; i++) {
    body += (char)payload[i];
  }

  if (topicStr == TOPIC_HUB_ANNOUNCE) {
    int keyIndex = body.indexOf("\"hubId\"");
    if (keyIndex < 0) return;

    int colon = body.indexOf(':', keyIndex);
    int q1 = body.indexOf('"', colon + 1);
    int q2 = body.indexOf('"', q1 + 1);
    if (q1 < 0 || q2 < 0) return;

    String hubId = body.substring(q1 + 1, q2);
    if (hubId.length() == 0) return;

    if (hubId != rememberedHubId) {
      saveHubId(hubId);
    }
    return;
  }

  if (topicStr == topicSet) {
    bool desired = false;
    if (!parseDesiredState(body, desired)) {
      Serial.print("[CMD] Unsupported payload: ");
      Serial.println(body);
      return;
    }

    if (desired != lightOn) {
      setLightState(desired);
      Serial.print("[CMD] Light -> ");
      Serial.println(lightOn ? "ON" : "OFF");
    } else {
      Serial.println("[CMD] Light already in requested state");
    }

    publishState();
  }
}

// ---------- CONNECTIVITY ----------
bool ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) return true;

  unsigned long now = millis();
  if (now - lastWifiRetryMs < wifiRetryDelayMs) return false;
  lastWifiRetryMs = now;

  Serial.print("[WiFi] Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 8000) {
    delay(200);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] Connected. IP=");
    Serial.println(WiFi.localIP());
    wifiRetryDelayMs = 1000;
    return true;
  }

  wifiRetryDelayMs = min((unsigned long)60000, wifiRetryDelayMs * 2);
  Serial.print("[WiFi] Retry in ms: ");
  Serial.println(wifiRetryDelayMs);
  return false;
}

bool ensureMqttConnected() {
  if (mqttClient.connected()) return true;
  if (WiFi.status() != WL_CONNECTED) return false;

  unsigned long now = millis();
  if (now - lastMqttRetryMs < mqttRetryDelayMs) return false;
  lastMqttRetryMs = now;

  String clientId = "device-" + deviceId;
  Serial.print("[MQTT] Connecting as ");
  Serial.println(clientId);

  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("[MQTT] Connected");
    mqttClient.subscribe(TOPIC_HUB_ANNOUNCE);
    mqttClient.subscribe(topicSet.c_str());
    mqttRetryDelayMs = 1000;
    publishState();
    return true;
  }

  mqttRetryDelayMs = min((unsigned long)60000, mqttRetryDelayMs * 2);
  Serial.print("[MQTT] Connect failed. state=");
  Serial.print(mqttClient.state());
  Serial.print(" retry(ms)=");
  Serial.println(mqttRetryDelayMs);
  return false;
}

// ---------- ARDUINO ----------
void setup() {
  Serial.begin(115200);
  delay(400);

  EEPROM.begin();
  rememberedHubId = eepromReadString(HUB_ID_ADDR, HUB_ID_MAXLEN);
  deviceId = buildStableDeviceId();

  topicSet = "devices/" + deviceId + "/set";
  topicState = "devices/" + deviceId + "/state";

  pinMode(LIGHT_PIN, OUTPUT);
  setLightState(false);  // safe default: light OFF at boot

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  Serial.println("==== UNO R4 Light Device Boot ====");
  Serial.print("Device ID: ");
  Serial.println(deviceId);
  Serial.print("MQTT Host: ");
  Serial.print(MQTT_HOST);
  Serial.print(":");
  Serial.println(MQTT_PORT);
  Serial.print("Light Pin: ");
  Serial.println(LIGHT_PIN);
  Serial.print("Command Topic: ");
  Serial.println(topicSet);
  Serial.print("State Topic: ");
  Serial.println(topicState);
  Serial.print("Remembered hubId: ");
  Serial.println(rememberedHubId);
}

void loop() {
  ensureWiFiConnected();
  ensureMqttConnected();

  if (mqttClient.connected()) {
    mqttClient.loop();

    unsigned long now = millis();
    if (now - lastPublishMs >= STATE_HEARTBEAT_MS) {
      lastPublishMs = now;
      publishState();
    }
  }

  delay(20);
}
