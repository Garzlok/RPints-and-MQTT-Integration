/*
  =============================================================================
  MQTT integration with RaspberryPints - Lolin D32 (ESP32)
  =============================================================================
  Hardware:
    - Dual YF-S201 Style Flow Meters (GPIO 25, 26)
    - DS18B20 OneWire Temperature Sensor (GPIO 27)
    - MFRC522 RFID Reader (SPI: SCK=18, MISO=19, MOSI=23, SS=5, RST=0)

  Features:
    - MQTT integration with RaspberryPints
    - NTP time sync with local timezone (EST with Daylight Savings)
    - Non-blocking WiFi/MQTT reconnection
    - Pour noise filtering (min pulse threshold)
    - RFID tag auth with 45-second session timeout
    - Temperature reporting every 15 minutes with retry on failure

  Special Thanks to HBT Members RandR+ and Thorrak
  ===============================================================================
*/

#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MFRC522.h>
#include <SPI.h>
#include <WiFi.h>   
#include <time.h>

// Forward Declarations
void setup_wifi(); 
void IRAM_ATTR pulseCounter1();
void IRAM_ATTR pulseCounter2(); 
void callback(char* topic, byte* payload, unsigned int length); 
bool checkMQTTConnection(); 
void RFIDCardAction(char* RFIDTag);
void RFIDCheckFunction(); 
void sendTemp(float temp, const char* probe, const char* unit, const char* timestamp); 
char* getTimestamp(); 

// WiFi Settings
const char* ssid = "SSID"; 
const char* password = "SSID_PW";

// MQTT Settings
const char* mqtt_server = "raspberrypints.local";           // If your RaspberryPints has a static IP, you can use the IP address here.
const int mqtt_port = 1883;
const char* mqtt_user = "RaspberryPints";                   // If you change the MQTT Broker User name, make sure you add that name here.
const char* mqtt_pass = "MQTT_PW";                          // Your MQTT Broker PW.
const char* mqtt_topic = "rpints/pours"; 

// RFID Settings (Lolin D32 SPI: SCK=18, MISO=19, MOSI=23)
#define SS_PIN 5
#define RST_PIN 0
unsigned long lastRfidCheckTime = 0;
unsigned int rfidCheckDelay = 250;
unsigned long lastRfidReadTime;
unsigned long lastRfidActivity = 0;
alignas(4) char RFIDTag[16];
volatile bool tagIsActive = false;
volatile bool messagePrinted = false;
MFRC522 mfrc522(SS_PIN, RST_PIN);

// Flow Sensor Pins (Lolin D32)
const int flowPin1 = 25; 
const int tapNumber1 = 4;                                   // Change for each tap. If running an Arduino through Serial or USB in conjunction with MQTT, Do not make this a pin number already used.
volatile unsigned long pulseCount1 = 0;

const int flowPin2 = 26; 
const int tapNumber2 = 6;                                   // Change for each tap. If running an Arduino through Serial or USB in conjunction with MQTT, Do not make this a pin number already used.
volatile unsigned long pulseCount2 = 0;

// Pour tracking
const unsigned long POUR_TIMEOUT = 2000;                    // ms of no flow before pour is considered done
const unsigned long CHECK_INTERVAL = 100;                   // how often to check for flow activity
const unsigned long MIN_POUR_PULSES = 200;                  // minimum pulses to count as a real pour (noise filter)
bool pouring1 = false;
unsigned long pourPulses1 = 0;
unsigned long lastPulseTime1 = 0;
bool pouring2 = false;
unsigned long pourPulses2 = 0;
unsigned long lastPulseTime2 = 0;
unsigned long lastCheckTime = 0;

// OneWire Settings
#define SENSOR_PIN 27                                       // Safe GPIO for OneWire on D32
const char* TZstr = "EST+5EDT,M3.2.0/2,M11.1.0/2";          // TZ offset set for EST and Daylight SAvings (POSIX Timezone String)
OneWire oneWire(SENSOR_PIN);
DallasTemperature DS18B20(&oneWire);

float temperature_C;                                        // temperature in Celsius
float temperature_F;                                        // temperature in Fahrenheit
static unsigned long tempTime = 0;
char probeName[24] = "Garage";                              // Name your Temp Probe to your requirements

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);

  setup_wifi();
  Serial.println("=== Kegerator Boot ===");

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  delay(1000);
  SPI.begin();                                              // Uses default pins 18, 19, 23
  mfrc522.PCD_Init(); 
  Serial.println("RFID Reader Ready"); 

  DS18B20.begin();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");        // Standard ESP32 NTP config
  setenv("TZ", TZstr, 1);
  tzset();
  
  pinMode(flowPin1, INPUT_PULLUP);
  pinMode(flowPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowPin1), pulseCounter1, FALLING);
  attachInterrupt(digitalPinToInterrupt(flowPin2), pulseCounter2, FALLING);

  Serial.println("Setup complete.");
}

void loop() {
  // Non-blocking connection health check
  if (!client.connected()) {
    static unsigned long lastReconnectAttempt = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = currentMillis;
      if (checkMQTTConnection()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    client.loop();
  }

  unsigned long now = millis();

 // 1. RFID scan interval
  if ((now - lastRfidCheckTime) > rfidCheckDelay || lastRfidCheckTime == 0) {
    
    // Keepalive: reinit if reader has been idle too long
    if ((now - lastRfidActivity > 300000UL) && !pouring1 && !pouring2) {
        mfrc522.PCD_Init();
        lastRfidActivity = millis();
        Serial.println("RFID keepalive reinit");
    }

    RFIDCheckFunction();
    lastRfidCheckTime = now;
  }

  // 2. Process Tag Buffer & Timeout logic
  RFIDCardAction(RFIDTag);

  // 3. Flow activity & MQTT Reporting
  if (now - lastCheckTime > CHECK_INTERVAL) {
    noInterrupts();
    unsigned long count1 = pulseCount1;
    pulseCount1 = 0;
    unsigned long count2 = pulseCount2;
    pulseCount2 = 0;
    interrupts();

    // Determine tag status safely
    bool isNoTag = true;
    if (strlen(RFIDTag) > 0 && isAlphaNumeric(RFIDTag[0])) {
      isNoTag = false;
    }
    if (strcmp(RFIDTag, "0") == 0) isNoTag = true;
    const char* activeTagStatus = isNoTag ? "-1" : "";

    // --- Tap 1 -----------------------------------------------------------------------------------------------
    if (count1 > 0) {
      pourPulses1 += count1;
      lastPulseTime1 = now;
      if (!pouring1) {
        pouring1 = true;
        Serial.printf("Tap %d: pour started\n", tapNumber1);
      }
    } else if (pouring1 && (now - lastPulseTime1 > POUR_TIMEOUT)) {
      if (pourPulses1 >= MIN_POUR_PULSES) {
        char payload[100];
        snprintf(payload, sizeof(payload), "P;%s;%d;%lu;%s", activeTagStatus, tapNumber1, pourPulses1, RFIDTag);
        if (client.connected()) client.publish(mqtt_topic, payload);
        Serial.printf("Sent: %s\n", payload);
      } else {
        Serial.printf("Tap %d: discarded noise\n", tapNumber1);
      }
      
        // Tap 1 reset
        pouring1 = false;
        pourPulses1 = 0;
        memset(RFIDTag, 0, 16);
        tagIsActive = false;
        detachInterrupt(digitalPinToInterrupt(flowPin1));
        detachInterrupt(digitalPinToInterrupt(flowPin2));
        delay(50);
        mfrc522.PCD_Init();
        delay(50);
        attachInterrupt(digitalPinToInterrupt(flowPin1), pulseCounter1, FALLING);
        attachInterrupt(digitalPinToInterrupt(flowPin2), pulseCounter2, FALLING);
    }

    // --- Tap 2 ---------------------------------------------------------------------------------------------------
    if (count2 > 0) {
      pourPulses2 += count2;
      lastPulseTime2 = now;
      if (!pouring2) {
        pouring2 = true;
        Serial.printf("Tap %d: pour started\n", tapNumber2);      
      }
    } else if (pouring2 && (now - lastPulseTime2 > POUR_TIMEOUT)) {
      if (pourPulses2 >= MIN_POUR_PULSES) {
        char payload[100];
        snprintf(payload, sizeof(payload), "P;%s;%d;%lu;%s", activeTagStatus, tapNumber2, pourPulses2, RFIDTag);
        if (client.connected()) client.publish(mqtt_topic, payload);
        Serial.printf("Sent: %s\n", payload);
      } else {
        Serial.printf("Tap %d: discarded noise\n", tapNumber2);
      }
      
        // Tap 2 reset
        pouring2 = false;
        pourPulses2 = 0;
        memset(RFIDTag, 0, 16);
        tagIsActive = false;
        detachInterrupt(digitalPinToInterrupt(flowPin1));
        detachInterrupt(digitalPinToInterrupt(flowPin2));
        delay(50);
        mfrc522.PCD_Init();
        delay(50);
        attachInterrupt(digitalPinToInterrupt(flowPin1), pulseCounter1, FALLING);
        attachInterrupt(digitalPinToInterrupt(flowPin2), pulseCounter2, FALLING);
    }
    lastCheckTime = now;
  }

  // 4. Temperature tracking (every 15 mins)
if (tempTime == 0 || (now - tempTime >= 900000UL)) {        // Set for 15 minutes, Adjust to your needs
    tempTime = now;

    DS18B20.requestTemperatures();
    temperature_C = DS18B20.getTempCByIndex(0);

  //  Temp Probe sanity check.
    if (temperature_C <= -126.0 || temperature_C == 85.0) {
        Serial.println("DS18B20 bad read, reinitializing...");        
        DS18B20.begin();
        delay(100);
        DS18B20.requestTemperatures();
        delay(750);
        temperature_C = DS18B20.getTempCByIndex(0);
    }

    if (temperature_C > -126.0 && temperature_C != 85.0) {
        temperature_F = temperature_C * 9.0 / 5.0 + 32.0;
        sendTemp(temperature_F, probeName, "F", getTimestamp());
        Serial.printf("Temperature: %.2f°F\n", temperature_F);        
    } else {
        Serial.println("DS18B20 failed after reinit — check wiring/power");        
    }
  }
}

void setup_wifi() {
  Serial.print("Connecting to WiFi"); 
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWiFi connected\nIP: %s\n", WiFi.localIP().toString().c_str()); 
}

void IRAM_ATTR pulseCounter1() { pulseCount1++; }
void IRAM_ATTR pulseCounter2() { pulseCount2++; }

void RFIDCheckFunction() {
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    
    // Build tag directly into a local buffer, no String object
    char localTag[16];
    memset(localTag, 0, sizeof(localTag));
    
    int pos = 0;
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      pos += snprintf(localTag + pos, sizeof(localTag) - pos, "%d", mfrc522.uid.uidByte[i]);
    }
    
    byte bcc = 0;
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      bcc ^= mfrc522.uid.uidByte[i];
    }
if (bcc < 10) {
      snprintf(localTag + pos, sizeof(localTag) - pos, "0%d", bcc);
    } else {
      snprintf(localTag + pos, sizeof(localTag) - pos, "%d", bcc);
    }

    // Uppercase in place
    for (int i = 0; localTag[i]; i++) {
      localTag[i] = toupper(localTag[i]);
    }

    // Single aligned copy, very fast, interrupt-safe
    memcpy(RFIDTag, localTag, 16);
    tagIsActive = true;
    messagePrinted = false;
    lastRfidReadTime = millis();
    mfrc522.PICC_HaltA();
    lastRfidActivity = millis();
  }
}

void RFIDCardAction(char* RFIDTag) {
  if (tagIsActive) {
    if (millis() - lastRfidReadTime >= 45000) {
      memset(RFIDTag, 0, 16);
      tagIsActive = false;
      messagePrinted = false;
      Serial.println("Tag memory cleared.");
    } else if (!messagePrinted) {
      Serial.printf("Processing UID Buffer: %s\n", RFIDTag);
      messagePrinted = true;
    }
  }
}

// Non-blocking client connection routine
bool checkMQTTConnection() {
  Serial.print("Attempting MQTT connection...");
  String clientId = "ESP32-taps-" + String(tapNumber1) + "-" + String(tapNumber2);
  if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
    Serial.println("Connected!");
    client.subscribe("rpints");
    return true;
  }
  Serial.printf("Failed, rc=%d. Will try again.\n", client.state());
  return false;
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("Message received on %s\n", topic);
}  

char* getTimestamp(){
  static char buffer[80];
  time_t timer;
  struct tm* timeinfo;
  time(&timer);
  timeinfo = localtime(&timer);
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", timeinfo);
  return buffer;
}

void sendTemp(float temp, const char* probe, const char* unit, const char* timestamp) {
  char payload[100];
  snprintf(payload, sizeof(payload), "T;%s;%.2f;%s;%s", probe, temp, unit, timestamp);
  if(client.connected()) client.publish(mqtt_topic, payload);
}
