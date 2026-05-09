RPints and MQTT Integration
---
ESP32 MQTT Integration of Flow Meters, Temperature Sensor, and RFID

---

To set-up MQTT/RPints integration using an either the ESP8266 or ESP32 microcontroller,
Please reference:
https://github.com/rtlindne/RaspberryPints/blob/master/README.md

---

These sketches will allow a user to attach 2 YF-5201 Flow Meters, a DS18B20 OneWire Temperature Sensor, and a MFRC522 RFID reader to a Lolin D32 (ESP32) microcontroller and integrate with the RandR+ RaspberryPints branch via MQTT.

---

-DS18B20 Sensor will broadcast temperature every 15 min via MQTT to RaspberryPints

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

---
Lolin D32 ESP32 Pin Assignment
---

Attaching Two(2) YF-S201 Flow Meters, One(1) DS18B20 Temperature Sensor

PIN---------------->DEVICE

GND---------------->DS18B20 Ground (Black)

23 (MOSI)

22 (SCL)

TX0

RX0

21 (SDA)

19 (MISO)

18 (SCK)

5 (SS)

17

16

4

0

2

15

GND---------------->YF-S201 Ground (Black) Both Flow Meters


3V----------------->DS18B20 3V3 (Red)-----> 4.7K Ω <-----(Yellow) Data DS18B20

RST

VP

VN

A34

A32

A33

DAC1 25------------>YF-S201 Data (Yellow) Flow Meter 2

DAC2 26------------>YF-S201 Data (yellow) Flow Meter 1

27----------------->DS18B20 Data (Yellow)-----> 4.7K Ω <----- (Red) 3V3 DS18B20

14

12

13

EN

USB---------------->YF-S201 5V (Red) Both Flow Meters

BAT


---
Attaching Two(2) YF-S201 Flow Meters, One(1) DS18B20 Temperature Sensor, One(1) MFRC522 RFID

PIN---------------->DEVICE

GND---------------->DS18B20 Ground (Black)
     |------------->MFRC522 GND

23 (MOSI)---------->MFRC522 MOSI

22 (SCL)

TX0

RX0

21 (SDA)
19 (MISO)---------->MFRC522 MISO

18 (SCK)----------->MFRC522 SCK

5 (SS)------------->MFRC522 SS (SDA)

17

16

4

0------------------>MFRC522 RST

2

15

GND---------------->YF-S201 Ground (Black) Both Flow Meters


3V----------------->DS18B20 3V3 (Red)-----> 4.7K Ω <-----(Yellow) Data DS18B20
     |------------->MFRC522 VCC

RST

VP

VN

A34

A32

A33

DAC1 25------------>YF-S201 Data (Yellow) Flow Meter 2

DAC2 26------------>YF-S201 Data (yellow) Flow Meter 1

27----------------->DS18B20 Data (Yellow)-----> 4.7K Ω <----- (Red) 3V3 DS18B20

14

12

13

EN

USB---------------->YF-S201 5V (Red) Both Flow Meters

BAT
---
