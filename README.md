# RPints-and-MQTT-Integration
MQTT Integration of Flow Meters, Temperature Sensor, and RFID with RaspberryPints Digital Tap List

---

To set=up MQTT/RPints integration using an either the ESP8266 or ESP32 microcontroller,
Please reference:
https://github.com/rtlindne/RaspberryPints/blob/master/README.md

---

These sketches will allow a user to attach 2 YF-5201 Flow Meters, a DS18B20 OneWire Temperature Sensor, and a MFRC522 RFID reader to either a NodeMCU (ESP8266) or Lolin D32 (ESP32) microcontrollers and integrate with the RandR+ RaspberryPints branch via MQTT.

---

ESP8266_RPints_MQTT_FM_TR.ino: Connect 2 YF-S201 Flow Meters and 1 DS18B20 Temp Sensor
ESP8266_RPints_MQTT_AllInOne.ino: Connect 2 YF-S201 Flow Meters, 1 DS18B20 Temp Sensor, and 1 MFRC522 RFID Reader**
ESP32_RPints_MQTT_FM_TR.ino: Connect 2 YF-S201 Flow Meters and 1 DS18B20 Temp Sensor
ESP8232_RPints_MQTT_AllInOne.ino: Connect 2 YF-S201 Flow Meters, 1 DS18B20 Temp Sensor, and 1 MFRC522 RFID Reader

** The ESP8266_RPints_MQTT_AllInOne.ino is not stable!! The ESP8232_RPints_MQTT_AllInOne.ino is recommended for full integration.
