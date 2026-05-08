RPints and MQTT Integration
---
ESP8266 MQTT Integration of Flow Meters, Temperature Sensor, and RFID

---

To set-up MQTT/RPints integration using an either the ESP8266 or ESP32 microcontroller,
Please reference:
https://github.com/rtlindne/RaspberryPints/blob/master/README.md

---

These sketches will allow a user to attach 2 YF-5201 Flow Meters, a DS18B20 OneWire Temperature Sensor, and a MFRC522 RFID reader to a NodeMCU (ESP8266) microcontroller and integrate with the RandR+ RaspberryPints branch via MQTT.

**WARNING**ESP8266_RPints_MQTT_AllInOne.ino is not stable and may stop read/reporting RFID Cards. The addition of the RFID Reader to the NodeMCU may push beyond the limits of the microcontroller. The Lolin D32 (ESP32) is recommended if the All In One is needed.**WARNING**

---

-DS18B20 Sensor will broadcast temperature every 15 min via MQTT to MQTT_PW

-Sensor Payload that RaspberryPints expects:

-"T;probe;temp;unit;timestamp"

-Flow Meters will only broadcast when a pour is initiated

-Minimum of 100 pulses is needed to achieve actual pour (can be adjusted in the sketch)

-Flow Meter Payload that RaspberryPints expects:

-"P;tagIDStatus;tapnumber;pourpulses;RFIDTag"

-tagIDStatus is known as UserID in RaspberryPints

-If no RFID Card is used, tagIDStatus is sent as "-1" which in most cases will not correspond to a user.

-If an RFID Card is used, tagIDStatus is sent as "", which RaspberryPints will determine User ID based on the RFID Tag.

-RFID Information will only be broadcasted in the Flow Meter Payload

-45 seconds after scanning the card, RFID information in RFIDTag will be removed from memory
