// ========================
// BLYNK CONFIGURATION (MUST BE BEFORE Blynk include)
// ========================
// #define BLYNK_TEMPLATE_ID "TMPL6N9jC2Q_Y"
// #define BLYNK_TEMPLATE_NAME "three phase current monitoring"
// #define BLYNK_AUTH_TOKEN "1AKMiIAF6IHBSqH0ViQ2kNv-UujjhSoB"

#define BLYNK_TEMPLATE_ID "TMPL6N9jC2Q_Y"
#define BLYNK_TEMPLATE_NAME "three phase current monitoring"
#define BLYNK_AUTH_TOKEN "pA6wwzyuNHE8vkFb-vnZ4_t9w5RZ1m9Q"

// ========================
// INCLUDES
// ========================
#include <Wire.h>
#include <LiquidCrystal_I2C.h>    // Use an ESP32-compatible LiquidCrystal_I2C library
#include <math.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// ========================
// HARDWARE / USER CONFIG
// ========================
LiquidCrystal_I2C lcd(0x27, 20, 4);

// WiFi credentials
char ssid[] = "Raju";
char pass[] = "@@ra7282ju@@";

// Blynk Virtual Pins
#define VPIN_VOLTAGE      V0
#define VPIN_CURRENT1     V1
#define VPIN_CURRENT2     V2
#define VPIN_CURRENT3     V3
#define VPIN_POWER1       V4
#define VPIN_POWER2       V5
#define VPIN_POWER3       V6
#define VPIN_TOTAL_POWER  V7
#define VPIN_ENERGY       V8
#define VPIN_BILL         V9
#define VPIN_PF1          V10
#define VPIN_PF2          V11
#define VPIN_PF3          V12
#define VPIN_STATUS       V13
#define VPIN_RESET_ENERGY V14

// Pins (ESP32 ADC pins)
#define ZMPT_PIN 35   // Voltage measurement pin (ZMPT101B)
#define ACS1_PIN 34   // Current sensor 1 (ACS712-20A)
#define ACS2_PIN 32   // Current sensor 2 (ACS712-20A)
#define ACS3_PIN 33   // Current sensor 3 (ACS712-20A)

// Sensor parameters
// ACS712-20A sensitivity ~ 100 mV/A (0.100 V/A) -> 100 mV/A
const float ACS1_SENSITIVITY_mV_per_A = 148.59; // mV per Amp for ACS712-20A
const float ACS2_SENSITIVITY_mV_per_A = 140.057; // mV per Amp for ACS712-20A
const float ACS3_SENSITIVITY_mV_per_A = 140.057; // mV per Amp for ACS712-20A

// Voltage calibration: adjust based on your meter
//float voltageCalibration = 225.0 / 0.80; // tune with a multimeter
float voltageCalibration = 223.0; // adjusted to match multimeter reading 227V


// ADC config
const int ADC_RES = 4095;     // 12-bit on ESP32 when analogReadResolution(12) is used
const float ADC_REF_V = 3.3;  // ADC reference on ESP32

// Filtering & thresholds
float currentThreshold = 0.26; // below this consider zero (A)
float unit_rate = 10.0;        // BDT per kWh

// Sampling
const int SAMPLE_COUNT = 2000;      // number of synchronous V/I samples (tune if needed)
const int SAMPLE_DELAY_US = 50;     // microseconds between pair-samples (controls sampling freq)

// Energy tracking
volatile float energy_kWh = 0.0;
unsigned long lastEnergyMillis = 0;

// Calibration zeros (auto-calibrated in setup)
int ACS1_ZERO = 2048;
int ACS2_ZERO = 2048;
int ACS3_ZERO = 2048;
int ZMPT_ZERO = 2048;

// Blynk state
bool blynkConnected = false;
unsigned long lastBlynkSend = 0;

// Alarm threshold
float currentAlarmThreshold = 28.0; // 20A sensor may be 20A safe; adjust accordingly
bool alarmTriggered = false;

// ========================
// PROTOTYPES
// ========================
int calibrateZero(int pin, int samples = 2000);
void measureAll(float &Vrms, float &Irms1, float &Irms2, float &Irms3,
                float &realP1, float &realP2, float &realP3);
float computePowerFactor(float Vrms, float Irms, float realPower);
void sendToBlynk(float Vrms, float I1, float I2, float I3, float P1, float P2, float P3, float totalPower, float energy, float bill);

// ========================
// Blynk WRITE for reset
// ========================
BLYNK_WRITE(VPIN_RESET_ENERGY) {
  if (param.asInt() == 1) {
    energy_kWh = 0.0;
    Blynk.virtualWrite(VPIN_ENERGY, energy_kWh);
    Blynk.virtualWrite(VPIN_BILL, 0);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Energy Reset!");
    delay(800);
    lcd.clear();
  }
}

// ========================
// SETUP
// ========================
void setup() {
  Serial.begin(115200);
  delay(100);

  // WiFi/Blynk
  WiFi.mode(WIFI_STA);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Energy Monitor");
  lcd.setCursor(0,1);
  lcd.print("Initializing...");
  delay(800);

  // Blynk init - this will attempt connect inside Blynk.begin
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // ADC config
  analogReadResolution(12);       // 0..4095
  analogSetAttenuation(ADC_11db); // allow up to ~3.3V reading

  // Initial zero calibration - ask user to remove all loads
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Remove Loads!");
  lcd.setCursor(0,1);
  lcd.print("Calibrating...");
  Serial.println("Please remove all loads. Calibrating zeros...");

  delay(1500);

  ACS1_ZERO = calibrateZero(ACS1_PIN, 1500);
  ACS2_ZERO = calibrateZero(ACS2_PIN, 1500);
  ACS3_ZERO = calibrateZero(ACS3_PIN, 1500);
  ZMPT_ZERO = 2048;  // typical mid-scale for 12-bit ADC

  Serial.print("Calib - ACS1_ZERO: "); Serial.println(ACS1_ZERO);
  Serial.print("Calib - ACS2_ZERO: "); Serial.println(ACS2_ZERO);
  Serial.print("Calib - ACS3_ZERO: "); Serial.println(ACS3_ZERO);
  Serial.print("Calib - ZMPT_ZERO: "); Serial.println(ZMPT_ZERO);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Calib Done");
  lcd.setCursor(0,1);
  lcd.print("IP:");
  if (WiFi.status() == WL_CONNECTED) {
    lcd.print(WiFi.localIP().toString());
  } else {
    lcd.print("No WiFi");
  }

  lastEnergyMillis = millis();
  delay(1000);
  lcd.clear();
}

// ========================
// MAIN LOOP
// ========================
void loop() {
  Blynk.run();

  // measure
  float Vrms=0, I1=0, I2=0, I3=0, P1=0, P2=0, P3=0;
  measureAll(Vrms, I1, I2, I3, P1, P2, P3);

  // ensure minimal values and thresholds
  if (I1 < currentThreshold)I1 = 0.0;
  if (P1 < 5 ) P1 = 0.0;

  if (I2 < currentThreshold) I2 = 0.0;
  if (P2 < 5) P2 = 0.0;

  if (I3 < currentThreshold) I3 = 0.0;
  if (P3 <5) P3 = 0.0;

  float totalPower = P1 + P2 + P3;

  // accumulate energy every loop (based on elapsed time)
  unsigned long now = millis();
  float dt_hours = (now - lastEnergyMillis) / 3600000.0; // ms to hours
  lastEnergyMillis = now;

  if (totalPower > 1.0) { // only accumulate when significant
    energy_kWh += (totalPower * dt_hours) / 1000.0; // W * h /1000 -> kWh
  }

  float bill = energy_kWh * unit_rate;

  // compute PFs
  float pf1 = computePowerFactor(Vrms, I1, P1);
  float pf2 = computePowerFactor(Vrms, I2, P2);
  float pf3 = computePowerFactor(Vrms, I3, P3);
  if(pf1 > currentThreshold){
    if(pf1 < 0.7) pf1 = 0.70;
    if(pf1 > 1.0) pf1 = 0.90;
  }else pf1 = 0.00;

  if(pf2 > currentThreshold){
    if(pf2 < 0.7) pf2 = 0.70;
    if(pf2 > 1.0) pf2 = 0.90;
  }else pf2 = 0.00;

  if(pf3 > currentThreshold){
    if(pf3 < 0.7) pf3 = 0.70;
    if(pf3 > 1.0) pf3 = 0.90;
  }else pf3 = 0.00;

  // LCD display cycling (simple)
  static unsigned long displayTimer = 0;
  static int displayMode = 0;
  if (millis() - displayTimer > 3000) {
    displayTimer = millis();
    displayMode = (displayMode + 1) % 3;
    lcd.clear();
  }

  if (displayMode == 0) {
    // Currents + PF
    lcd.setCursor(0,0);
    lcd.print("I1:"); lcd.print(I1, 3); lcd.print("A ");
    lcd.print("PF:"); lcd.print(pf1, 2);
    lcd.setCursor(0,1);
    lcd.print("I2:"); lcd.print(I2, 3); lcd.print("A ");
    lcd.print("PF:"); lcd.print(pf2, 2);
    lcd.setCursor(0,2);
    lcd.print("I3:"); lcd.print(I3, 3); lcd.print("A ");
    lcd.print("PF:"); lcd.print(pf3, 2);
    lcd.setCursor(0,3);
    lcd.print("V:"); lcd.print(Vrms,1); lcd.print("V T:"); lcd.print((int)totalPower); lcd.print("W");
  } else if (displayMode == 1) {
    // Powers
    lcd.setCursor(0,0);
    lcd.print("P1:"); lcd.print(P1,1); lcd.print("W ");
    lcd.print("P2:"); lcd.print(P2,1); lcd.print("W");
    lcd.setCursor(0,1);
    lcd.print("P3:"); lcd.print(P3,1); lcd.print("W");
    lcd.setCursor(0,2);
    lcd.print("Total:"); lcd.print(totalPower,1); lcd.print("W");
    float avgPF = 0.0;
    if ((pf1+pf2+pf3) > 0.0) avgPF = (pf1 + pf2 + pf3) / 3.0;
    lcd.setCursor(0,3);
    lcd.print("AvgPF:"); lcd.print(avgPF,2);
  } else {
    // Energy & bill
    lcd.setCursor(0,0);
    lcd.print("Energy (kWh):");
    lcd.setCursor(0,1);
    lcd.print(energy_kWh, 4);
    lcd.setCursor(0,2);
    lcd.print("Bill:"); lcd.print(bill, 2); lcd.print(" Tk");
    lcd.setCursor(0,3);
    lcd.print("Rate:"); lcd.print(unit_rate); lcd.print(" Tk/kWh");
  }

  // send data to Blynk every 2s
  if (Blynk.connected() && millis() - lastBlynkSend > 2000) {
    sendToBlynk(Vrms, I1, I2, I3, P1, P2, P3, totalPower, energy_kWh, bill);
    lastBlynkSend = millis();
  }

  // alarm check
  if (!alarmTriggered && (I1 > currentAlarmThreshold || I2 > currentAlarmThreshold || I3 > currentAlarmThreshold)) {
    alarmTriggered = true;
    if (Blynk.connected()) Blynk.logEvent("overcurrent", "Overcurrent detected!");
  }
  if (alarmTriggered && (I1 < currentAlarmThreshold && I2 < currentAlarmThreshold && I3 < currentAlarmThreshold)) {
    alarmTriggered = false;
  }

  // Serial debug
  Serial.print("V:"); Serial.print(Vrms,2);
  Serial.print(" | I1:"); Serial.print(I1,3);
  Serial.print(" | I2:"); Serial.print(I2,3);
  Serial.print(" | I3:"); Serial.print(I3,3);
  Serial.print(" | P1:"); Serial.print(P1,1);
  Serial.print(" | P2:"); Serial.print(P2,1);
  Serial.print(" | P3:"); Serial.print(P3,1);
  Serial.print(" | Tot:"); Serial.print(totalPower,1);
  Serial.print(" | kWh:"); Serial.print(energy_kWh,4);
  Serial.print(" | Bill:"); Serial.print(bill,2);
  Serial.print(" | PFs:"); Serial.print(pf1,2); Serial.print(","); Serial.print(pf2,2); Serial.print(","); Serial.println(pf3,2);

  delay(100); // small pause
}

// ========================
// FUNCTIONS
// ========================

// Calibrate zero offset for a pin (returns average ADC)
int calibrateZero(int pin, int samples) {
  long sum = 0;
  int minv = 4095, maxv = 0;
  for (int i=0; i<samples; i++) {
    int v = analogRead(pin);
    sum += v;
    if (v < minv) minv = v;
    if (v > maxv) maxv = v;
    delayMicroseconds(100);
  }
  int avg = sum / samples;
  int noise = maxv - minv;
  Serial.print("Calib pin "); Serial.print(pin); Serial.print(" avg="); Serial.print(avg); Serial.print(" noise="); Serial.println(noise);
  return avg;
}

// Measure Vrms, Irms for each channel and compute instantaneous real powers
void measureAll(float &Vrms, float &Irms1, float &Irms2, float &Irms3,
                float &realP1, float &realP2, float &realP3) {

  // Use double accumulators to avoid overflow when squaring many ADC values
  double sumVsq = 0.0;
  double sumIsq1 = 0.0;
  double sumIsq2 = 0.0;
  double sumIsq3 = 0.0;
  double sumP1 = 0.0;
  double sumP2 = 0.0;
  double sumP3 = 0.0;

  int validSamples = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int v_adc = analogRead(ZMPT_PIN);
    int i1_adc = analogRead(ACS1_PIN);
    int i2_adc = analogRead(ACS2_PIN);
    int i3_adc = analogRead(ACS3_PIN);

    // centered readings (raw)
    double v_center = (double)(v_adc - ZMPT_ZERO);
    double i1_center = (double)(i1_adc - ACS1_ZERO);
    double i2_center = (double)(i2_adc - ACS2_ZERO);
    double i3_center = (double)(i3_adc - ACS3_ZERO);

    // accumulate squares (for RMS)
    sumVsq += v_center * v_center;
    sumIsq1 += i1_center * i1_center;
    sumIsq2 += i2_center * i2_center;
    sumIsq3 += i3_center * i3_center;

    // convert raw to instantaneous voltage and current values (in SI units)
    // ADC -> voltage: (raw * ADC_REF_V / ADC_RES)
    double V_inst = (v_center * (ADC_REF_V / ADC_RES)) * voltageCalibration; // Volts instantaneous (centered)
    // Current: convert adc-centered value -> volts then to mV then divide by mV/A
    double I1_inst = ((i1_center * (ADC_REF_V / ADC_RES)) * 1000.0) / ACS1_SENSITIVITY_mV_per_A; // Amp
    double I2_inst = ((i2_center * (ADC_REF_V / ADC_RES)) * 1000.0) / ACS2_SENSITIVITY_mV_per_A;
    double I3_inst = ((i3_center * (ADC_REF_V / ADC_RES)) * 1000.0) / ACS3_SENSITIVITY_mV_per_A;

    // accumulate instantaneous real power sums (V * I)
    sumP1 += V_inst * I1_inst;
    sumP2 += V_inst * I2_inst;
    sumP3 += V_inst * I3_inst;

    validSamples++;
    delayMicroseconds(SAMPLE_DELAY_US);
  }

  if (validSamples == 0) {
    Vrms = Irms1 = Irms2 = Irms3 = realP1 = realP2 = realP3 = 0.0;
    return;
  }

  // compute mean squares (raw)
  double meanVsq_raw = sumVsq / (double)validSamples;
  double meanI1sq_raw = sumIsq1 / (double)validSamples;
  double meanI2sq_raw = sumIsq2 / (double)validSamples;
  double meanI3sq_raw = sumIsq3 / (double)validSamples;

  // Convert raw-square means into voltage/current RMS
  double Vrms_converted = sqrt(meanVsq_raw) * (ADC_REF_V / ADC_RES) * voltageCalibration;
  double Irms1_converted = (sqrt(meanI1sq_raw) * (ADC_REF_V / ADC_RES) * 1000.0) / ACS1_SENSITIVITY_mV_per_A;
  double Irms2_converted = (sqrt(meanI2sq_raw) * (ADC_REF_V / ADC_RES) * 1000.0) / ACS2_SENSITIVITY_mV_per_A;
  double Irms3_converted = (sqrt(meanI3sq_raw) * (ADC_REF_V / ADC_RES) * 1000.0) / ACS3_SENSITIVITY_mV_per_A;

  // Real power average (P = average of instantaneous V*I)
  double P1_avg = sumP1 / (double)validSamples;
  double P2_avg = sumP2 / (double)validSamples;
  double P3_avg = sumP3 / (double)validSamples;

  // Fix sign inversion if a channel shows consistently negative real power
  // (common when sensor orientation is reversed). We flip sign so output shows positive magnitudes.
  if (P1_avg < 0 && fabs(P1_avg) > 0.5) {
    P1_avg = -P1_avg;
    Irms1_converted = fabs(Irms1_converted);
  }
  if (P2_avg < 0 && fabs(P2_avg) > 0.5) {
    P2_avg = -P2_avg;
    Irms2_converted = fabs(Irms2_converted);
  }
  if (P3_avg < 0 && fabs(P3_avg) > 0.5) {
    P3_avg = -P3_avg;
    Irms3_converted = fabs(Irms3_converted);
  }


  // assign results
  Vrms = (float)Vrms_converted;
  Irms1 = (float)Irms1_converted;
  Irms2 = (float)Irms2_converted;
  Irms3 = (float)Irms3_converted;

  realP1 = (float)P1_avg;
  realP2 = (float)P2_avg;
  realP3 = (float)P3_avg;

  // clamp very small negatives to zero
  if (realP1 < 0.0 && fabs(realP1) < 0.5) realP1 = 0.0;
  if (realP2 < 0.0 && fabs(realP2) < 0.5) realP2 = 0.0;
  if (realP3 < 0.0 && fabs(realP3) < 0.5) realP3 = 0.0;
}

// Compute power factor (real / apparent)
float computePowerFactor(float Vrms, float Irms, float realPower) {
  if (Irms <= 0.001 || Vrms <= 1.0) return 0.0;
  float apparent = Vrms * Irms;
  if (apparent <= 0.001) return 0.0;
  float pf = realPower / apparent;
  if (pf < 0.0) pf = 0.0;
  if (pf > 1.0) pf = 1.0;
  return pf;
}

// Send data to Blynk
void sendToBlynk(float Vrms, float I1, float I2, float I3, float P1, float P2, float P3, float totalPower, float energy, float bill) {
  Blynk.virtualWrite(VPIN_VOLTAGE, Vrms);
  Blynk.virtualWrite(VPIN_CURRENT1, I1);
  Blynk.virtualWrite(VPIN_CURRENT2, I2);
  Blynk.virtualWrite(VPIN_CURRENT3, I3);

  Blynk.virtualWrite(VPIN_POWER1, P1);
  Blynk.virtualWrite(VPIN_POWER2, P2);
  Blynk.virtualWrite(VPIN_POWER3, P3);
  Blynk.virtualWrite(VPIN_TOTAL_POWER, totalPower);

  Blynk.virtualWrite(VPIN_ENERGY, energy);
  Blynk.virtualWrite(VPIN_BILL, bill);

  // compute PFs and send
  float pf1 = computePowerFactor(Vrms, I1, P1);
  float pf2 = computePowerFactor(Vrms, I2, P2);
  float pf3 = computePowerFactor(Vrms, I3, P3);

  if(pf1 > currentThreshold){
    if(pf1 < 0.7) pf1 = 0.70;
    if(pf1 > 1.0) pf1 = 0.90;
  }else pf1 = 0.00;

  if(pf2 > currentThreshold){
    if(pf2 < 0.7) pf2 = 0.70;
    if(pf2 > 1.0) pf2 = 0.90;
  }else pf2 = 0.00;

  if(pf3 > currentThreshold){
    if(pf3 < 0.7) pf3 = 0.70;
    if(pf3 > 1.0) pf3 = 0.90;
  }else pf3 = 0.00;

  Blynk.virtualWrite(VPIN_PF1, pf1);
  Blynk.virtualWrite(VPIN_PF2, pf2);
  Blynk.virtualWrite(VPIN_PF3, pf3);

  // status
  Blynk.virtualWrite(VPIN_STATUS, alarmTriggered ? "ALARM: Overcurrent" : "Online");
}
