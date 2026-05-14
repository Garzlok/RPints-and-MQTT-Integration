RPints and MQTT Integration
---
ESP8266 MQTT Integration of Flow Meters, Temperature Sensor, and RFID

---

To set-up MQTT/RPints integration using an either the ESP8266 or ESP32 microcontroller,
Please reference:
https://github.com/rtlindne/RaspberryPints/blob/master/README.md

---

These sketches will allow a user to attach 2 YF-5201 Flow Meters, a DS18B20 OneWire Temperature Sensor, and a MFRC522 RFID reader to a NodeMCU (ESP8266) microcontroller and integrate with the RandR+ RaspberryPints branch via MQTT.

**WARNING**==ESP8266_RPints_MQTT_AllInOne.ino has not been tested. The Lolin D32 (ESP32) is recommended if the All In One integration is needed.==**WARNING**

---

-DS18B20 Sensor will broadcast temperature every 15 min via MQTT to RaspberryPints

-Sensor Payload that RaspberryPints expects:

-"T;probe;temp;unit;timestamp"

-Flow Meters will only broadcast when a pour is initiated

-Minimum pulses is needed to achieve actual pour (can be adjusted in the sketch)

-Flow Meter Payload that RaspberryPints expects:

-"P;tagIDStatus;tapnumber;pourpulses;RFIDTag"

-tagIDStatus is known as UserID in RaspberryPints

-If no RFID Card is used, tagIDStatus is sent as "-1" which in most cases will not correspond to a user.

-If an RFID Card is used, tagIDStatus is sent as "", which RaspberryPints will determine User ID based on the RFID Tag.

-RFID Information will only be broadcasted in the Flow Meter Payload

-45 seconds after scanning the card, RFID information in RFIDTag will be removed from memory

```
_________________________________________________________________________________
NodeMCU ESP8266 Pin Assignment
_________________________________________________________________________________
Attaching Two(2) YF-S201 Flow Meters, One(1) DS18B20 Temperature Sensor
_________________________________________________________________________________
PIN----->DEVICE
A0
RS
RSV
SD3
SD2
SD1
CMD
SD0
CLK
GND
3V3
EN
RST
GND----->YF-S201 Ground (Black) Both Flow Meters
VIN----->YF-S201 5V (Red) Both Flow Meters

____________________________________________________________________________________

D0
D1
D2----->YF-S201 Data (yellow) Flow Meter 1
D3
D4
3V3
GND
D5------>YF-S201 Data (Yellow) Flow Meter 
D6
D7------>DS18B20 Data (Yellow)-----> 4.7K Ω <----- (Red) 3V3 DS18B20
D8
RX
TX
GND----->DS18B20 Ground (Black)
3V3----->DS18B20 3V3 (Red)-----> 4.7K Ω <-----(Yellow) Data DS18B20
```

```
___________________________________________________________________________________________
NodeMCU ESP8266 Pin Assignment
____________________________________________________________________________________________
Attaching Two(2) YF-S201 Flow Meters, One(1) DS18B20 Temperature Sensor, One(1) MFRC522 RFID
____________________________________________________________________________________________

PIN----->DEVICE
A0
RSV
RSV
SD3
SD2
SD1
CMD
SD0
CLK
GND
3V3
EN
RST
GND----->YF-S201 Ground (Black) Both Flow Meters
VIN----->YF-S201 3V3 (Red) Both Flow Meters

__________________________________________________________________________________________

D0----->DS18B20 Data (Yellow)-----> 4.7K Ω <----- (Red) 3V3 DS18B20
D1----->YF-S201 Data (yellow) Flow Meter 1
D2----->YF-S201 Data (Yellow) Flow Meter 2
D3
D4----->MFRC522 RST
3V3---->MMFRC522 VCC (Red)
GND---->MFRC522 GND (Black)
D5----->MOSI, MISO, or SCK
D6----->MOSI, MISO, or SCK
D7----->MOSI, MISO, or SCK
D8----->MFRC522 SDA (SS)
RX
TX
GND----->DS18B20 Ground (Black)
3V3----->DS18B20 3V3 (Red)-----> 4.7K Ω <-----(Yellow) Data DS18B20
```
