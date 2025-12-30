#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

/* ---------- WiFi & MQTT ---------- */
const char* ssid = "Raju";
const char* password = "@@ra7282ju@@";
const char* mqtt_server = "test.mosquitto.org";
WiFiClient espClient;
PubSubClient client(espClient);


const char* data_topic = "esp32/sensors";
const char* pump_topic = "esp32/pump";


/* ---------- Constants ---------- */
const float calibrationFactor = 7.5; // YF-S201
const float voltage = 12.0;          // Fixed 12V


/* ---------- Pins ---------- */
#define PUMP_PIN 13
#define ACS_PIN  32


#define TRIG_PIN 4
#define ECHO_PIN 2
#define PIN_FLOW 27     // Water flow sensor pulse (interrupt)
volatile unsigned long pulse_count = 0;
const float PULSES_PER_LITER = 7.5; // <-- adjust for your sensor
float flowRate = 0.0;
// SD (VSPI): SCK=18, MISO=19, MOSI=23, CS=5
#define SD_CS_PIN 5
#define DHTTYPE DHT11
#define DHTPIN 5
DHT dht(DHTPIN, DHTTYPE);
const char* svgFileName = "/temperatue_humidity_power_water";



int temp;
int humi;
int litter = 0;
int water = 20;
int power = 70;


void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  if (String(topic) == pump_topic) {
    if (msg == "ON") {
      digitalWrite(PUMP_PIN, HIGH);
    } else if (msg == "OFF") {
      digitalWrite(PUMP_PIN, LOW);
    }
  }
}

/***what is your name*/

/* ---------- ACS712 ---------- */
float getPower() {
  int adc = analogRead(ACS_PIN);
  float voltageOut = adc * (3.3 / 4095.0);

  float offsetVoltage = 2.40;      
  float sensitivity = 0.185;       

  float current = -(voltageOut - offsetVoltage) / sensitivity;


  if (current < 0.02) current = 0; // noise threshold

  return voltage * current;
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


void startDht(){
  static int cnt = 2000;
  if(cnt >= 50){
    temp = dht.readTemperature();
    humi = dht.readHumidity();
    power = getPower();
    water = getWaterLevel();
    cnt = 1;
  }
  cnt++;
}

void IRAM_ATTR pulseCounter();

void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(ACS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), pulseCounter, RISING);
  digitalWrite(PUMP_PIN, LOW);
  while(0){
    int a = pulse_count / 8;
    Serial.println(a);
    delay(10);
    
  }
  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  while (!client.connected()) {
    client.connect("ESP32_COUNTERS");
    delay(500);
  }

  client.subscribe(pump_topic);   
  Serial.println("MQTT connected & LED topic subscribed");
}

void loop() {
  client.loop();
  startDht();
  watterFlow();
  
  static int cnt1 = 200;
  if(cnt1 >= 100){
    char payload[100];
    sprintf(payload,
      "{\"water\": %d, \"temp\": %d, \"humi\": %d, \"power\": %d, \"litter\" : %d}",
      water, temp, humi, power, litter);
    client.publish(data_topic, payload);
    Serial.println(payload);
    cnt1 = 0;
  }
  
  delay(10);
  cnt1++;
}

void IRAM_ATTR pulseCounter() {
  pulse_count++;
}

void watterFlow(){
  litter = pulse_count / 10;
}

void dataLoginBegin(){
    // Init SD
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card Mount Failed!");
    while (true) delay(500);
  }
  Serial.println("SD Card Initialized.");

  // If file doesn't exist, create and write SVG header
  if (!SD.exists(svgFileName)) {
    File file = SD.open(svgFileName, FILE_WRITE);
    if (file) {
      file.println("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>");
      file.println("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"800\" height=\"400\">");
      file.println("<text x=\"40\" y=\"40\" font-family=\"Arial\" font-size=\"28\" fill=\"#222\">Temperature & Humidity Log</text>");
      file.println("</svg>"); // Will update later, we append before this
      file.close();
    }
  }
  
}
