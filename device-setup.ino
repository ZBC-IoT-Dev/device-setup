#include <WiFi.h>
#include <PubSubClient.h>

/**
 * 🛰 Arduino Discovery Firmware
 * Part of the IoT Homekit project.
 * 
 * This sketch connects to WiFi and pulses its presence via MQTT
 * to the Raspberry Pi Gateway.
 */

// --- CONFIGURATION ---
const char* ssid     = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";
const char* mqtt_server = "YOUR_PI_IP_ADDRESS"; // Example: 192.168.1.50

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  
  // Connect to WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi Connected!");

  // Setup MQTT
  client.setServer(mqtt_server, 1883);
}

void connectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ArduinoClient-" + String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str())) {
      Serial.println("connected!");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" - trying again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();

  // Create the Discovery JSON
  // id: unique hardware ID, type: sensor category
  // data: actual sensor readings
  String jsonPayload = "{\"id\": \"arduino_stue_1\", \"type\": \"ClimateSensor\", \"temp\": 22.5, \"humidity\": 45}";

  Serial.println("Sending discovery pulse...");
  client.publish("discovery/announce", jsonPayload.c_str());

  // Wait 10 seconds before next shout
  delay(10000);
}
