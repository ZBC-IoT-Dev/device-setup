#include <WiFiS3.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <DHT.h>

// ---------- USER CONFIG ----------
const char* WIFI_SSID     = "SSID";
const char* WIFI_PASSWORD = "PASSWORD";
const char* MQTT_HOST     = "10.106.189.237";
const uint16_t MQTT_PORT  = 1883;
const char* DEVICE_TYPE   = "ClimateSensor";

// ---------- DHT22 CONFIG ----------
// Wiring in your image matches data on D2.
const uint8_t DHT_PIN = 2;
const uint8_t DHT_TYPE = DHT22;

// ---------- TIMING ----------
const unsigned long SENSOR_READ_INTERVAL_MS = 2500;   // DHT22 needs >= 2s between reads
const unsigned long TELEMETRY_INTERVAL_MS   = 10000;  // publish every 10 seconds

// ---------- MQTT TOPICS ----------
const char* TOPIC_DISCOVERY    = "discovery/announce";
const char* TOPIC_HUB_ANNOUNCE = "hub/announce";

// ---------- EEPROM LAYOUT (UNO R4) ----------
const int HUB_ID_ADDR   = 0;
const int HUB_ID_MAXLEN = 64;

// ---------- GLOBALS ----------
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
DHT dht(DHT_PIN, DHT_TYPE);

String deviceId;
String rememberedHubId;

unsigned long lastPublishMs    = 0;
unsigned long lastSensorReadMs = 0;
unsigned long lastWifiRetryMs  = 0;
unsigned long lastMqttRetryMs  = 0;

unsigned long wifiRetryDelayMs = 1000;  // max 60s
unsigned long mqttRetryDelayMs = 1000;  // max 60s

float lastTempC = NAN;
float lastHumidity = NAN;
float lastHeatIndexC = NAN;
bool hasValidSensorReading = false;

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

// ---------- MQTT CALLBACK ----------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (String(topic) != TOPIC_HUB_ANNOUNCE) return;

  String body;
  body.reserve(length);
  for (unsigned int i = 0; i < length; i++) {
    body += (char)payload[i];
  }

  // Minimal JSON extraction: "hubId":"..."
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
    mqttRetryDelayMs = 1000;
    return true;
  }

  mqttRetryDelayMs = min((unsigned long)60000, mqttRetryDelayMs * 2);
  Serial.print("[MQTT] Connect failed. state=");
  Serial.print(mqttClient.state());
  Serial.print(" retry(ms)=");
  Serial.println(mqttRetryDelayMs);
  return false;
}

// ---------- SENSOR ----------
void readDhtIfDue() {
  unsigned long now = millis();
  if (now - lastSensorReadMs < SENSOR_READ_INTERVAL_MS) return;
  lastSensorReadMs = now;

  float humidity = dht.readHumidity();
  float tempC = dht.readTemperature();

  if (isnan(humidity) || isnan(tempC)) {
    hasValidSensorReading = false;
    Serial.println("[DHT22] Read failed");
    return;
  }

  lastHumidity = humidity;
  lastTempC = tempC;
  lastHeatIndexC = dht.computeHeatIndex(tempC, humidity, false);
  hasValidSensorReading = true;

  Serial.print("[DHT22] TempC=");
  Serial.print(lastTempC, 2);
  Serial.print(" Humidity=");
  Serial.print(lastHumidity, 2);
  Serial.print("%");
  Serial.print(" HeatIndexC=");
  Serial.println(lastHeatIndexC, 2);
}

// ---------- TELEMETRY ----------
String buildTelemetryPayload() {
  float tempF = lastTempC * 1.8f + 32.0f;

  String payload = "{";
  payload += "\"id\":\"" + deviceId + "\",";
  payload += "\"type\":\"" + String(DEVICE_TYPE) + "\",";
  payload += "\"hubId\":\"" + rememberedHubId + "\",";
  payload += "\"temp\":" + String(lastTempC, 2) + ",";
  payload += "\"tempC\":" + String(lastTempC, 2) + ",";
  payload += "\"tempF\":" + String(tempF, 2) + ",";
  payload += "\"humidity\":" + String(lastHumidity, 2) + ",";
  payload += "\"heatIndexC\":" + String(lastHeatIndexC, 2) + ",";
  payload += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  payload += "\"uptimeMs\":" + String(millis());
  payload += "}";
  return payload;
}

void publishTelemetryIfDue() {
  if (!hasValidSensorReading) return;

  unsigned long now = millis();
  if (now - lastPublishMs < TELEMETRY_INTERVAL_MS) return;
  lastPublishMs = now;

  String payload = buildTelemetryPayload();
  bool ok = mqttClient.publish(TOPIC_DISCOVERY, payload.c_str(), false);

  if (ok) {
    Serial.print("[MQTT] Sent: ");
    Serial.println(payload);
  } else {
    Serial.println("[MQTT] Publish failed");
  }
}

// ---------- ARDUINO ----------
void setup() {
  Serial.begin(115200);
  delay(500);

  EEPROM.begin();

  rememberedHubId = eepromReadString(HUB_ID_ADDR, HUB_ID_MAXLEN);
  deviceId = buildStableDeviceId();

  dht.begin();
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  Serial.println("==== UNO R4 DHT22 Device Boot ====");
  Serial.print("Device ID: ");
  Serial.println(deviceId);
  Serial.print("MQTT Host: ");
  Serial.print(MQTT_HOST);
  Serial.print(":");
  Serial.println(MQTT_PORT);
  Serial.print("DHT Pin: ");
  Serial.println(DHT_PIN);
  Serial.print("Remembered hubId: ");
  Serial.println(rememberedHubId);
}

void loop() {
  ensureWiFiConnected();
  ensureMqttConnected();
  readDhtIfDue();

  if (mqttClient.connected()) {
    mqttClient.loop();
    publishTelemetryIfDue();
  }

  delay(25);
}
