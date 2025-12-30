const int ACS_PIN = 34;   // Safe ADC1 pin

// ACS712 5A sensitivity (V/A)
//const float sensitivity = 0.185;
const float sensitivity = 0.66;

const float adcVref = 3.3;
const int adcMax = 4095;

float offsetVoltage = 2.5;   // default, will be calibrated

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  delay(500);

  // --- Auto Calibration ---
  long sum = 0;
  const int samples = 200;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(ACS_PIN);
    delay(5);
  }

  float avgRaw = sum / samples;
  offsetVoltage = (avgRaw * adcVref) / adcMax;

  Serial.print("Calibrated 0A Offset Voltage: ");
  Serial.print(offsetVoltage, 4);
  Serial.println(" V");
}

void loop() {
  int raw = analogRead(ACS_PIN);
  while(1){
    raw = analogRead(ACS_PIN);
    Serial.println(raw);
    delay(500);
  }
  float voltage = (raw * adcVref) / adcMax;

  float current = (voltage - offsetVoltage) / sensitivity;

  Serial.print("ADC: ");
  Serial.print(raw);
  Serial.print("  Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V  Current: ");
  Serial.print(current, 3);
  Serial.println(" A");

  delay(300);
}
