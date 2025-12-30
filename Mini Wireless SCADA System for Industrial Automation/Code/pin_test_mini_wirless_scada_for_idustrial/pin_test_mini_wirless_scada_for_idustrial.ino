#include <WiFi.h>
#include "DHT.h"

#define DHTPIN 5         // Safe pin for ESP32
#define DHTTYPE DHT11    // Change to DHT22 if needed
DHT dht(DHTPIN, DHTTYPE);
const int ACS_PIN = 32;   // Safe ADC1 pin

// ACS712 5A version sensitivity
// 185 mV per Amp (0.185 V/A)
const float sensitivity = 0.185;  
// ESP32 ADC reference voltage assumption
const float adcVref = 3.3;

// For ESP32 (12-bit ADC => 0–4095)
const int adcMax = 4095;


// Safe pins for ESP32 even with WiFi enabled
#define TRIG_PIN 4
#define ECHO_PIN 2

long duration;
float distance;

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  dht.begin();
  analogReadResolution(12);  // ensure 12-bit ADC
  // Ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  while(0){
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(1000);
  }

 
}

void loop() {

  int raw = analogRead(ACS_PIN);
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature(); // Celsius
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print(" %  |  Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");


  // // Convert ADC to voltage
  // float voltage = (raw * adcVref) / adcMax;

  // // ACS712 output at 0A = Vcc/2 (≈2.5V when powered from 5V)
  // float offsetVoltage = 2.3571;  

  // // Current calculation:
  // // I = (Vout - Vcc/2) / Sensitivity
  // float current = (voltage - offsetVoltage) / sensitivity;

  // Serial.print("ADC: ");
  // Serial.print(raw);
  // Serial.print("  Voltage: ");
  // Serial.print(voltage, 3);
  // Serial.print(" V  Current: ");
  // Serial.print(current, 3);
  // Serial.println(" A");

  // delay(300);


  // // Trigger pulse
  // digitalWrite(TRIG_PIN, LOW);
  // delayMicroseconds(2);

  // digitalWrite(TRIG_PIN, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(TRIG_PIN, LOW);

  // // Read echo
  // duration = pulseIn(ECHO_PIN, HIGH);

  // // Calculate distance in cm
  // distance = duration * 0.034 / 2;

  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.println(" cm");

  // delay(500);
}
