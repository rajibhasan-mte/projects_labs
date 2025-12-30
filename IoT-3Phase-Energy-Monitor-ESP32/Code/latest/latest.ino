#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

#define ZMPT_PIN 35   // Voltage sensor (ZMPT101B)
#define ACS_PIN 32    // Current sensor (ACS712 20A)


// ---------------------------
// USER CALIBRATION PARAMETERS
// ---------------------------
//float voltageCalibration = 285.0; // Adjust based on your multimeter


float voltageCalibration = 228.0 / 0.75; // Adjust based on your multimeter
float ACS_SENSITIVITY = 100.0;           // mV/A for ACS712-20A

int ACS_ZERO_RAW = 2048;                  // auto-calibrate at no load
float currentCalibration = 1.27;          // Adjust based on multimeter
float unit_rate = 10.0;                    // BDT per kWh
float currentThreshold = 0.2;             // Current below 0.2A considered zero
// ---------------------------

// ---------------------------
// Function Prototypes
// ---------------------------
int calibrateACSZero(int samples=1000);
float readVrmsSamples(int pin, int samples=2000);
float readIrmsSamples(int pin, int zeroRaw, int samples=2000);
float computeRealPower(int voltagePin, int currentPin, int zeroRaw, int samples=2000);

// ---------------------------
// Global Variables
// ---------------------------
float energy_kWh = 0;
unsigned long lastMillis = 0;

// ---------------------------
// Setup
// ---------------------------
void setup(){
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0,0);
  lcd.print("Smart Energy Meter");
  delay(1500);

  // Auto-calibrate ACS712 zero
  ACS_ZERO_RAW = calibrateACSZero();
  Serial.print("ACS712 zero raw value: ");
  Serial.println(ACS_ZERO_RAW);

  lastMillis = millis();
}

// ---------------------------
// Loop
// ---------------------------
void loop(){
  // Read RMS values
  float Vrms = readVrmsSamples(ZMPT_PIN);
  float Irms = readIrmsSamples(ACS_PIN, ACS_ZERO_RAW);

  // Apply current threshold filter
  if(Irms < currentThreshold) Irms = 0;

  // Compute real power
  float realPower = computeRealPower(ZMPT_PIN, ACS_PIN, ACS_ZERO_RAW);

  // Set power = 0 if current below threshold
  if(Irms == 0) realPower = 0;

  // Apparent power
  float apparentPower = Vrms * Irms;

  // Real-time Power Factor
  float powerFactor = 0;
  if(apparentPower > 0.01) powerFactor = realPower / apparentPower;

  // Energy accumulation and billing
  unsigned long now = millis();
  float dt = (now - lastMillis) / 3600000.0; // hours
  lastMillis = now;
  energy_kWh += realPower * dt / 1000.0;
  float bill = energy_kWh * unit_rate;

  // ---------------- LCD DISPLAY ------------------
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("V:"); lcd.print(Vrms,1); lcd.print("V ");
  lcd.setCursor(11,0);
  lcd.print("PF:"); lcd.print(powerFactor,2);

  lcd.setCursor(0,1);
  lcd.print("I:"); lcd.print(Irms,2); lcd.print("A ");
  lcd.setCursor(11,1);
  lcd.print("VA:"); lcd.print(apparentPower,0);

  lcd.setCursor(0,2);
  lcd.print("P:"); lcd.print(realPower,1); lcd.print("W ");
  lcd.setCursor(11,2);
  lcd.print("kWh:"); lcd.print(energy_kWh,3);

  lcd.setCursor(0,3);
  lcd.print("Bill:"); lcd.print(bill,2); lcd.print(" Tk");

  // ---------------- SERIAL MONITOR ------------------
  Serial.print("Vrms: "); Serial.print(Vrms,2);
  Serial.print(" V, Irms: "); Serial.print(Irms,2);
  Serial.print(" A, P: "); Serial.print(realPower,2);
  Serial.print(" W, PF: "); Serial.print(powerFactor,2);
  Serial.print(", kWh: "); Serial.print(energy_kWh,3);
  Serial.print(", Bill: "); Serial.println(bill,2);

  delay(500);
}

// ---------------------------
// Functions
// ---------------------------

// ACS712 zero calibration
int calibrateACSZero(int samples){
  long sum = 0;
  Serial.println("Calibrating ACS712 zero, remove load...");
  delay(2000);
  for(int i=0;i<samples;i++){
    sum += analogRead(ACS_PIN);
    delayMicroseconds(50);
  }
  return sum / samples;
}

// Read RMS voltage samples
float readVrmsSamples(int pin, int samples){
  long sum = 0;
  for(int i=0;i<samples;i++){
    int adc = analogRead(pin);
    float centered = adc - 2048; // remove mid-point
    sum += centered * centered;
  }
  float mean = sum / (float)samples;
  float rms = sqrt(mean);
  return rms * 3.3 / 4095.0 * voltageCalibration;
}

// Read RMS current samples
float readIrmsSamples(int pin, int zeroRaw, int samples){
  long sum = 0;
  for(int i=0;i<samples;i++){
    int adc = analogRead(pin);
    float centered = adc - zeroRaw;
    sum += centered * centered;
  }
  float mean = sum / (float)samples;
  float rms = sqrt(mean);
  float Irms = (rms * 3.3 / 4095.0 * 1000.0) / ACS_SENSITIVITY;
  Irms *= currentCalibration; // apply calibration factor
  return Irms;
}

// Compute instantaneous real power with negative power fix
float computeRealPower(int voltagePin, int currentPin, int zeroRaw, int samples){
  float V, I, Psum = 0;
  for(int i=0;i<samples;i++){
    int v_adc = analogRead(voltagePin);
    int i_adc = analogRead(currentPin);

    V = ((v_adc - 2048) * 3.3 / 4095.0) * voltageCalibration;
    I = ((i_adc - zeroRaw) * 3.3 / 4095.0 * 1000.0) / ACS_SENSITIVITY;
    I *= currentCalibration;

    // --------------------------
    // Fix negative power
    // --------------------------
    if(V * I < 0) I = -I;

    Psum += V * I;
  }
  return Psum / samples;  // average instantaneous power
}
