// ---------- Design and Implementation of a Mini Wireless SCADA System for Industrial Automation ----------
// ---------- ESP32_NodeRED_IoT_Project.ino ----------
// Hardware: ESP32 DevKit v1
// Sensors: DHT11, HC-SR04 (ultrasonic), YF-S201 (water flow), SD card (SPI), Relay
// Communication: WiFi + MQTT (no OTA)
// Author: Rajib Hasan

#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <SPI.h>
#include <SD.h>

// -----------------------------
// ======= USER CONFIG =========
// -----------------------------
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* mqtt_server = "MQTT_BROKER_IP_OR_HOST";
const uint16_t mqtt_port = 1883; // change if needed
const char* mqtt_user = ""; // optional
const char* mqtt_pass = ""; // optional

// Pin assignments (chosen to avoid common WiFi/ADC conflicts on ESP32 DevKit v1)
// You can change these later but avoid strapping pins (0,2,15) and ADC2 pins when using WiFi
#define PIN_DHT       21    // DHT11 data pin
#define PIN_TRIG      13    // Ultrasonic TRIG (output)
#define PIN_ECHO      34    // Ultrasonic ECHO (input only pin - safe)
#define PIN_FLOW      4     // Water flow sensor pulse (interrupt)
#define PIN_RELAY     16    // Relay control (pump)
// SD (VSPI): SCK=18, MISO=19, MOSI=23, CS=5
#define PIN_SDCARD_CS 5

// Tank geometry variables (user can adjust later)
float tank_height_cm = 100.0; // set actual tank height in cm
float tank_base_area_cm2 = 7853.98; // e.g., for circular tank: pi*r^2 (r=50cm => area ~7853.98)

// Flow sensor calibration
// Set PULSES_PER_LITER according to your sensor datasheet. Default is 7.5 pulses per liter (example).
// Please calibrate for accurate total liters.
volatile unsigned long pulse_count = 0;
const float PULSES_PER_LITER = 7.5; // <-- adjust for your sensor

// Measurement / timing
unsigned long lastDHTmillis = 0;
const unsigned long DHT_INTERVAL = 1000UL; // 1 second

unsigned long lastPublishMillis = 0;
const unsigned long PUBLISH_INTERVAL = 2000UL; // publish MQTT every 2s

unsigned long lastSDLogMillis = 0;
const unsigned long SD_LOG_INTERVAL = 10000UL; // write to SD every 10s

// Alert thresholds
float low_level_percent_threshold = 10.0; // percent to trigger low level alert
float high_temp_threshold = 50.0; // Celsius, change as necessary

// Modes
bool AUTO_MODE = false; // if true, pump auto controlled by tank level
bool pumpState = false; // current relay state (LOW/HIGH depending on relay wiring)

// MQTT topics
const char* baseTopic = "esp32/garden";
char topic_state[64]; // will be baseTopic/state
char topic_cmd[64];   // baseTopic/cmd
char topic_auto[64];  // baseTopic/auto
char topic_alert[64]; // baseTopic/alert

// Objects
WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(PIN_DHT, DHT11);
File sdFile;

// Sensor readings
float lastTemp = 0.0;
float lastHum = 0.0;
float lastDistanceCm = 0.0;
float lastLevelPercent = 0.0;
float lastFlowLpm = 0.0; // liters per minute
float totalLiters = 0.0;

// ---------- FUNCTION DECLARATIONS ----------
void setup_wifi();
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);
void IRAM_ATTR pulseCounter();
float measureUltrasonic();
void controlPump(bool on);
void publishState();
void sdLog();

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW); // assume LOW is off; adjust if your relay is active LOW
  pumpState = false;

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), pulseCounter, RISING);

  // SD init
  if (!SD.begin(PIN_SDCARD_CS)) {
    Serial.println("SD Card initialization failed!");
  } else {
    Serial.println("SD Card initialized.");
    // create header if not exists
    if (!SD.exists("log.csv")) {
      File f = SD.open("log.csv", FILE_WRITE);
      if (f) {
        f.println("timestamp,temp_c,hum_percent,level_percent,distance_cm,flow_lpm,total_liters,pump");
        f.close();
      }
    }
  }

  // DHT init
  dht.begin();

  // WiFi + MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  snprintf(topic_state, sizeof(topic_state), "%s/state", baseTopic);
  snprintf(topic_cmd, sizeof(topic_cmd), "%s/cmd", baseTopic);
  snprintf(topic_auto, sizeof(topic_auto), "%s/auto", baseTopic);
  snprintf(topic_alert, sizeof(topic_alert), "%s/alert", baseTopic);
  client.setCallback(callback);
}

// ---------- LOOP ----------
void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long now = millis();

  // DHT read interval
  if (now - lastDHTmillis >= DHT_INTERVAL) {
    lastDHTmillis = now;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t)) lastTemp = t;
    if (!isnan(h)) lastHum = h;
  }

  // ultrasonic measure (non-blocking if needed)
  lastDistanceCm = measureUltrasonic();
  // compute level percent
  float level_cm = tank_height_cm - lastDistanceCm;
  if (level_cm < 0) level_cm = 0;
  if (level_cm > tank_height_cm) level_cm = tank_height_cm;
  lastLevelPercent = (level_cm / tank_height_cm) * 100.0;

  // Flow calculations: compute instantaneous flow every second using pulse_count
  static unsigned long lastFlowMillis = 0;
  static unsigned long lastPulseCount = 0;
  if (now - lastFlowMillis >= 1000UL) {
    unsigned long pulses = pulse_count;
    unsigned long deltaPulses = pulses - lastPulseCount;
    lastPulseCount = pulses;
    lastFlowMillis = now;

    // liters per second = pulses / PULSES_PER_LITER
    float liters_per_second = ((float)deltaPulses) / PULSES_PER_LITER;
    lastFlowLpm = liters_per_second * 60.0; // L/min
    totalLiters += ((float)deltaPulses) / PULSES_PER_LITER; // accumulate liters
  }

  // Auto control
  if (AUTO_MODE) {
    if (lastLevelPercent < low_level_percent_threshold) {
      controlPump(true);
    } else if (lastLevelPercent > (low_level_percent_threshold + 5.0)) {
      // hysteresis +5%
      controlPump(false);
    }
  }

  // Publish MQTT periodically
  if (now - lastPublishMillis >= PUBLISH_INTERVAL) {
    lastPublishMillis = now;
    publishState();
  }

  // SD log periodically
  if (now - lastSDLogMillis >= SD_LOG_INTERVAL) {
    lastSDLogMillis = now;
    sdLog();
  }
}

// ---------- FUNCTIONS ----------

void setup_wifi() {
  delay(10);
  Serial.printf("Connecting to %s\n", ssid);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000UL) {
    delay(500);
    Serial.print('.');
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi failed to connect (timeout)");
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      // Subscribe topics
      client.subscribe(topic_cmd);
      client.subscribe(topic_auto);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.print("Message arrived ["); Serial.print(topic); Serial.print("]: "); Serial.println(msg);

  if (String(topic) == topic_cmd) {
    // Expected payload: "ON" or "OFF"
    if (msg.equalsIgnoreCase("ON")) controlPump(true);
    else if (msg.equalsIgnoreCase("OFF")) controlPump(false);
    else if (msg.equalsIgnoreCase("TOGGLE")) controlPump(!pumpState);
  } else if (String(topic) == topic_auto) {
    if (msg == "1" || msg.equalsIgnoreCase("ON")) AUTO_MODE = true;
    else if (msg == "0" || msg.equalsIgnoreCase("OFF")) AUTO_MODE = false;
  }
}

void IRAM_ATTR pulseCounter() {
  pulse_count++;
}

float measureUltrasonic() {
  // Send trigger pulse
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Read echo
  unsigned long duration = pulseIn(PIN_ECHO, HIGH, 30000UL); // timeout 30ms (max ~5m)
  if (duration == 0) return tank_height_cm + 10.0; // no echo
  // speed of sound ~343 m/s -> 0.0343 cm/us; distance = (duration/2) * 0.0343
  float distance_cm = (duration / 2.0) * 0.0343;
  return distance_cm;
}

void controlPump(bool on) {
  pumpState = on;
  // If your relay is active HIGH, use digitalWrite(PIN_RELAY, HIGH) to turn on
  // If active LOW, invert this behavior. Adjust as needed.
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
}

void publishState() {
  // Publish JSON with sensors and states
  // Example: {"temp":24.5,"hum":60.2,"level_pct":72.1,"distance_cm":27.9,"flow_lpm":1.2,"total_liters":12.34,"pump":1,"auto":0}
  char payload[256];
  snprintf(payload, sizeof(payload), "{\"temp\":%.2f,\"hum\":%.2f,\"level_pct\":%.2f,\"distance_cm\":%.2f,\"flow_lpm\":%.2f,\"total_liters\":%.3f,\"pump\":%d,\"auto\":%d}",
           lastTemp, lastHum, lastLevelPercent, lastDistanceCm, lastFlowLpm, totalLiters, pumpState ? 1 : 0, AUTO_MODE ? 1 : 0);
  client.publish(topic_state, payload);

  // Alerts
  if (lastLevelPercent < low_level_percent_threshold) {
    char a[128];
    snprintf(a, sizeof(a), "LOW_LEVEL:%.2f", lastLevelPercent);
    client.publish(topic_alert, a);
  }
  if (lastTemp > high_temp_threshold) {
    char a[128];
    snprintf(a, sizeof(a), "HIGH_TEMP:%.2f", lastTemp);
    client.publish(topic_alert, a);
  }
}

void sdLog() {
  if (!SD.begin(PIN_SDCARD_CS)) return; // try again
  File f = SD.open("log.csv", FILE_APPEND);
  if (f) {
    unsigned long ts = millis();
    char line[256];
    snprintf(line, sizeof(line), "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%d", ts, lastTemp, lastHum, lastLevelPercent, lastDistanceCm, lastFlowLpm, totalLiters, pumpState ? 1 : 0);
    f.println(line);
    f.close();
  }
}

// ---------- END OF FIRMWARE ----------


/*
  ---------- Node-RED Flow (JSON) ----------
  Import this into Node-RED (Menu -> Import -> Clipboard)
  Edit MQTT Broker credentials, and adjust dashboard nodes as needed.
*/

const char* nodeRedFlowJson = R"FLOW(
[{
    "id":"mqtt-in-state","type":"mqtt in","z":"flow","name":"Subscribe state","topic":"esp32/garden/state","qos":"0","datatype":"auto","broker":"mqtt-broker","x":140,"y":120,"wires":[["json-parse","ui-text-temp"]]
  },
  {"id":"json-parse","type":"json","z":"flow","name":"Parse JSON","x":320,"y":120,"wires":[["update-ui","file-log"]]},
  {"id":"update-ui","type":"function","z":"flow","name":"Update UI payload","func":"var s = msg.payload;\nreturn {payload:s};","outputs":1,"x":500,"y":100,"wires":[["ui-temp-gauge","ui-hum-gauge","ui-level-gauge","ui-flow-text"]]},
  {"id":"ui-temp-gauge","type":"ui_chart","z":"flow","name":"Temp Chart","group":"ui_group","order":1,"width":0,"height":0,"label":"Temperature (C)","chartType":"line","x":720,"y":60,"wires":[]},
  {"id":"ui-hum-gauge","type":"ui_chart","z":"flow","name":"Humidity Chart","group":"ui_group","order":2,"label":"Humidity (%)","chartType":"line","x":720,"y":120,"wires":[]},
  {"id":"ui-level-gauge","type":"ui_gauge","z":"flow","name":"Tank Level","group":"ui_group","order":3,"width":0,"height":0,"label":"Level %","value":"payload.level_pct","min":0,"max":100,"x":940,"y":120,"wires":[]},
  {"id":"ui-flow-text","type":"ui_text","z":"flow","name":"Flow","group":"ui_group","order":4,"label":"Flow L/min","format":"{{msg.payload.flow_lpm}}","x":940,"y":180,"wires":[]},
  {"id":"file-log","type":"file","z":"flow","name":"Save state to file","filename":"/home/node-red/garden_logs/state_log.csv","appendNewline":true,"createDir":true,"overwriteFile":"false","encoding":"utf8","x":600,"y":220,"wires":[]},
  {"id":"ui-pump-btn","type":"ui_button","z":"flow","name":"Pump ON/OFF","group":"ui_group","order":5,"width":0,"height":0,"label":"Toggle Pump","payload":"TOGGLE","payloadType":"str","topic":"","x":160,"y":300,"wires":[["mqtt-out-cmd"]]},
  {"id":"mqtt-out-cmd","type":"mqtt out","z":"flow","name":"Pump Cmd","topic":"esp32/garden/cmd","qos":"0","retain":"false","broker":"mqtt-broker","x":380,"y":300,"wires":[]},
  {"id":"ui-auto-switch","type":"ui_switch","z":"flow","name":"Auto Mode","label":"Auto Mode","group":"ui_group","order":6,"width":0,"height":0,"passthru":true,"topic":"","style":"","onvalue":"1","onvalueType":"num","offvalue":"0","offvalueType":"num","x":160,"y":360,"wires":[["mqtt-out-auto"]]},
  {"id":"mqtt-out-auto","type":"mqtt out","z":"flow","name":"Auto Cmd","topic":"esp32/garden/auto","qos":"0","retain":"false","broker":"mqtt-broker","x":380,"y":360,"wires":[]},
  {"id":"mqtt-broker","type":"mqtt-broker","name":"MQTT Broker","broker":"YOUR_MQTT_BROKER","port":"1883","clientid":"node-red","usetls":false,"compatmode":true,"keepalive":"60","cleansession":true,"birthTopic":"","birthQos":"0","birthPayload":"","closeTopic":"","closePayload":"","willTopic":"","willQos":"0","willPayload":""},
  {"id":"ui_group","type":"ui_group","z":"","name":"Garden","tab":"ui_tab","order":1,"disp":true,"width":"6","collapse":false},
  {"id":"ui_tab","type":"ui_tab","z":"","name":"ESP32 Garden","icon":"dashboard","order":1}
]
)FLOW";

// note: Node-RED flow JSON is included above as nodeRedFlowJson for reference; copy/paste to Node-RED import.

