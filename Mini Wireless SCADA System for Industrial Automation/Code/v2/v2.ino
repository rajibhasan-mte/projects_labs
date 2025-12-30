#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

/* ---------- WiFi & MQTT ---------- */
const char* ssid = "Raju";
const char* password = "@@ra7282ju@@";
const char* mqtt_server = "test.mosquitto.org";

const char* data_topic = "esp32/sensors";
const char* pump_topic = "esp32/pump";

/* ---------- Pins ---------- */
#define PUMP_PIN 13
#define ACS_PIN  34

#define TRIG_PIN 4
#define ECHO_PIN 4

#define FLOW_PIN 27

#define DHTPIN 11
#define DHTTYPE DHT11

/* ---------- Objects ---------- */
WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);

/* ---------- Flow Sensor ---------- */
volatile int flowPulseCount = 0;
float totalLiters = 0;
unsigned long lastFlowMillis = 0;

/* ---------- Constants ---------- */
const float calibrationFactor = 7.5; // YF-S201
const float voltage = 12.0;          // Fixed 12V

/* ---------- ISR ---------- */
void IRAM_ATTR flowISR() {
  flowPulseCount++;
}

/* ---------- Pump Control ---------- */
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];

  if (String(topic) == pump_topic) {
    if (msg == "ON") {
      digitalWrite(PUMP_PIN, HIGH);
    } else if (msg == "OFF") {
      digitalWrite(PUMP_PIN, LOW);
    }
  }
}

/* ---------- Ultrasonic ---------- */
int getWaterLevel() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 0;

  float distance = duration * 0.034 / 2; // cm

  float level = map(distance, 30, 5, 0, 100);
  return constrain(level, 0, 100);
}

/* ---------- ACS712 ---------- */
float getPower() {
  int adc = analogRead(ACS_PIN);
  float voltageOut = adc * (3.3 / 4095.0);
  float current = (voltageOut - 1.65) / 0.066; // ACS712-30A
  if (current < 0) current = 0;
  return voltage * current;
}

/* ---------- Setup ---------- */
void setup() {
  Serial.begin(115200);

  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, FALLING);

  dht.begin();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  while (!client.connected()) {
    client.connect("ESP32_WATER_SYSTEM");
    delay(500);
  }

  client.subscribe(pump_topic);
}

/* ---------- Loop ---------- */
void loop() {
  client.loop();

  /* ---- Temperature & Humidity ---- */
  float temp = dht.readTemperature();
  float humi = dht.readHumidity();

  /* ---- Water Level ---- */
  int waterLevel = getWaterLevel();

  /* ---- Flow Calculation ---- */
  if (millis() - lastFlowMillis >= 1000) {
    float flowRate = (flowPulseCount / calibrationFactor);
    totalLiters += flowRate / 60.0;
    flowPulseCount = 0;
    lastFlowMillis = millis();
  }

  /* ---- Power ---- */
  float power = getPower();

  /* ---- JSON Payload ---- */
  char payload[256];
  sprintf(payload,
    "{\"temperature\":%.2f,\"humidity\":%.2f,\"water_level\":%d,\"water_flow\":%.3f,\"power\":%.2f}",
    temp, humi, waterLevel, totalLiters, power
  );

  client.publish(data_topic, payload);
  Serial.println(payload);

  delay(1000);
}
