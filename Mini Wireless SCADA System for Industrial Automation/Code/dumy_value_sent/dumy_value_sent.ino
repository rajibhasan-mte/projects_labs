#include <WiFi.h>
#include <PubSubClient.h>

// WiFi credentials
const char* ssid = "Raju";
const char* password = "@@ra7282ju@@";

const char* mqtt_server = "test.mosquitto.org";
const char* topic = "esp32/counters";

WiFiClient espClient;
PubSubClient client(espClient);

int water = 0;
int temp = 32;
int humi = 80;
int power = 70;

void setup() {
  Serial.begin(115200);

  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // MQTT
  client.setServer(mqtt_server, 1883);
  while (!client.connected()) {
    client.connect("ESP32_COUNTERS");
    delay(500);
  }
  Serial.println("MQTT connected");
}

void loop() {

  if(water >= 100) water = 0;
  
  if(temp >= 37) temp = 32;
  
  if(humi >= 100) humi = 50;
  
  if(power >= 100) power = 30;


  water++;
  temp++;
  humi++;
  power++;


  char payload[80];
  sprintf(payload, "{\"water\": %d, \"temp\" : %d, \"humi\" : %d, \"power\" : %d}", water, temp, humi, power);

  client.publish(topic, payload);
  Serial.println(payload);

  delay(100);
}
