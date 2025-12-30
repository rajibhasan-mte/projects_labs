#include <WiFi.h>
#include <PubSubClient.h>

// WiFi credentials
const char* ssid = "Raju";
const char* password = "@@ra7282ju@@";

const char* mqtt_server = "test.mosquitto.org";

// Topics
const char* data_topic = "esp32/senor";
const char* pump_topic  = "esp32/pump";

WiFiClient espClient;
PubSubClient client(espClient);

// Built-in LED (ESP32)
const int ledPin = 13;

int water = 0;
int temp = 32;
int humi = 80;
int power = 70;

// 🔁 MQTT callback (LED control)
void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  if (String(topic) == pump_topic) {
    if (msg == "ON") {
      digitalWrite(ledPin, HIGH);
    } else if (msg == "OFF") {
      digitalWrite(ledPin, LOW);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  while(0){
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);
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

  client.subscribe(pump_topic);   // 🔔 subscribe for LED control
  Serial.println("MQTT connected & LED topic subscribed");
}

void loop() {
  client.loop();  // 🔴 VERY IMPORTANT

  if (water >= 100) water = 0;
  if (temp >= 37)  temp = 32;
  if (humi >= 100) humi = 50;
  if (power >= 100) power = 30;

  water++;
  temp++;
  humi++;
  power++;

  char payload[100];
  sprintf(payload,
          "{\"water\": %d, \"temp\": %d, \"humi\": %d, \"power\": %d}",
          water, temp, humi, power);

  client.publish(data_topic, payload);
  Serial.println(payload);

  delay(100);
}
