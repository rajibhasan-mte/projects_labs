#include <WiFi.h>
#include <PubSubClient.h>

#define LedPin 13  // ESP32 on-board LED

// WiFi credentials
const char* ssid = "Raju";
const char* password = "@@ra7282ju@@";

// MQTT server and port
const char* mqtt_server = "test.mosquitto.org"; 
const int mqtt_port = 1883; // Standard non-secure MQTT port
char* pumpTopic = "/pumpRajib";


WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

// Corrected Callback function to handle incoming messages
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("]: ");

    // Ensure payload is not empty and get the first character
    if (length > 0) {
        char msg = (char)payload[0]; // Access the first character for the control logic
        
        // Print the full message for debugging
        for (unsigned int i = 0; i < length; i++) {
            Serial.print((char)payload[i]);
        }
        Serial.println();

        // Control LED based on message
        if (msg == '2') {
            digitalWrite(LedPin, HIGH);
            Serial.println("pump ON");
        } else if (msg == '1') {
            digitalWrite(LedPin, LOW);
            Serial.println("pump OFF");
        }
    }
}

// Reconnect to MQTT broker if disconnected
void reconnect() {
    while (!client.connected()) {
        Serial.println("Attempting MQTT connection...");
        // Attempt to connect with a unique client ID
        if (client.connect("ESPClient-LedController")) { 
            Serial.println("Connected");
            //client.subscribe("/LedControl");    // subscribe to topic
            client.subscribe("/pumpRajib");    // subscribe to topic
            Serial.println("Subscribed to" );
            Serial.println(pumpTopic);
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(LedPin, OUTPUT);
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port); // Use the defined port
    client.setCallback(callback);
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop(); // Required to maintain connection and process incoming messages

    //delay(10);
}