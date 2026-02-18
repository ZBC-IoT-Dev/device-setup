# Arduino Device Setup (UNO R4 WiFi + MQTT)

This repository contains Arduino sketches for MQTT-based devices that integrate with the Raspberry Pi gateway.

Available sketches:

- `device-setup.ino`: Basic climate telemetry stub (`temp`, `humidity` placeholders)
- `device-setup-dht22.ino`: Real DHT22 climate sensor telemetry
- `device-setup-light.ino`: Light/switch actuator (receives commands)

## 1. Hardware Requirements

- Arduino UNO R4 WiFi
- Wi-Fi network reachable by gateway/MQTT broker
- MQTT broker (default: Mosquitto on gateway)
- Optional components:
  - DHT22 sensor for `device-setup-dht22.ino`
  - LED + 220-330 ohm resistor (or relay module) for `device-setup-light.ino`

## 2. Arduino IDE Libraries

Install via Library Manager:

- `PubSubClient`
- `DHT sensor library` (only for DHT22 sketch)

Core includes already used by UNO R4 WiFi:

- `WiFiS3`
- `EEPROM`

## 3. Configure Before Upload

In each sketch, update:

```cpp
const char* WIFI_SSID     = "SSID";
const char* WIFI_PASSWORD = "PASSWORD";
const char* MQTT_HOST     = "<gateway-or-broker-ip>";
const uint16_t MQTT_PORT  = 1883;
```

Then select board/port and upload.

## 4. MQTT Topics Used

Shared discovery topics:

- Publish telemetry: `discovery/announce`
- Subscribe hub announce: `hub/announce`

Per-device topics (light sketch):

- Subscribe command: `devices/<deviceId>/set`
- Publish retained state: `devices/<deviceId>/state`

`<deviceId>` is generated from Wi-Fi MAC, format:

- `uno-r4-<MACHEX>`

## 5. Sketch Behavior

### `device-setup.ino` (basic climate)

- Connects Wi-Fi + MQTT with exponential reconnect backoff
- Listens to `hub/announce` and stores `hubId` in EEPROM
- Every 10s publishes telemetry JSON to `discovery/announce`

Example payload:

```json
{
  "id": "uno-r4-ABCDEF123456",
  "type": "ClimateSensor",
  "hubId": "<rememberedHubId>",
  "temp": 22.5,
  "humidity": 45.0,
  "ts": 123456
}
```

### `device-setup-dht22.ino` (real sensor)

- Reads DHT22 every 2.5s
- Publishes valid reading every 10s
- Includes `tempC`, `tempF`, `humidity`, `heatIndexC`, `rssi`, `uptimeMs`

DHT22 wiring used by sketch:

- Data pin: `D2`

### `device-setup-light.ino` (actuator)

- Controls output pin `D7`
- Safe default at boot: OFF
- Parses commands from `devices/<deviceId>/set`
- Accepts payloads like:
  - `"on"`, `"off"`, `"1"`, `"0"`, `"true"`, `"false"`
  - `{ "state": "ON" }` / `{ "state": "OFF" }`
  - `{ "isOn": 1 }` / `{ "isOn": 0 }`
- Publishes current state to both:
  - `devices/<deviceId>/state` (retained)
  - `discovery/announce`

## 6. EEPROM Usage

All sketches persist last seen `hubId` so they keep context after reboot.

- EEPROM address: `0`
- Max stored length: `64`

## 7. Quick End-to-End Test

1. Start MQTT broker and gateway.
2. Upload one sketch with correct Wi-Fi/MQTT host.
3. Open Serial Monitor (`115200`) and confirm connection.
4. Verify telemetry arrives at broker:

```bash
mosquitto_sub -h <broker-ip> -t 'discovery/announce' -v
```

5. For light sketch, send command:

```bash
mosquitto_pub -h <broker-ip> -t 'devices/<deviceId>/set' -m '{"state":"ON"}'
```

## 8. Troubleshooting

- Wi-Fi never connects:
  1. verify SSID/password
  2. check network supports 2.4GHz for UNO R4 WiFi
- MQTT connect fails:
  1. verify broker IP/port
  2. ensure broker allows client connections from LAN
- Light not responding:
  1. verify command topic uses exact generated `deviceId`
  2. verify wiring polarity (`RELAY_ACTIVE_LOW`)
- No DHT22 values:
  1. check wiring to `D2`
  2. verify sensor power and minimum read interval (>=2s)
