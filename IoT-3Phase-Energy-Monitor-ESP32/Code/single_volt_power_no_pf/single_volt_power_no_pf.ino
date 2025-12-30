#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);

#define ZMPT_PIN 35       // ZMPT101B voltage sensor
#define ACS_PIN 34        // ACS712 20A current sensor

// ---------------------------
// USER CALIBRATION PARAMETERS
// ---------------------------
float voltageCalibration = 223.0 / 0.835; // adjust with measured values
float ACS_SENSITIVITY = 100.0;           // mV/A for ACS712-20A
int ACS_ZERO_RAW = 2048;                  // will auto-calibrate at no load
float currentCalibration = 1.27;  // adjust based on your measurement

float unit_rate = 8.5; // BDT per kWh

// ---------------------------

float readVrms(int pin, float calibrationFactor, int samples=2000){
  long sum = 0;
  for(int i=0;i<samples;i++){
    int adc = analogRead(pin);
    float centered = adc - 2048;  // remove mid-point DC
    sum += centered * centered;
  }
  float mean = sum / (float)samples;
  float rms = sqrt(mean);
  float voltage = (rms * 3.3 / 4095.0);
  return voltage * calibrationFactor;
}

float readIrms(int pin, int zeroRaw, int samples=2000){
  long sum = 0;
  for(int i=0;i<samples;i++){
    int adc = analogRead(pin);
    float centered = adc - zeroRaw;
    sum += centered * centered;
  }
  float mean = sum / (float)samples;
  float rms = sqrt(mean);
  float Irms = (rms * 3.3 / 4095.0) * 1000.0 / ACS_SENSITIVITY;
  Irms *= currentCalibration; // Apply calibration factor
  if(Irms < 0.01) Irms = 0;
  return Irms;
}


/*
float readIrms(int pin, int zeroRaw, float sensitivity, int samples=2000){
  long sum = 0;
  for(int i=0;i<samples;i++){
    int adc = analogRead(pin);
    float centered = adc - zeroRaw;  // subtract no-load offset
    sum += centered * centered;
  }
  float mean = sum / (float)samples;
  float rms = sqrt(mean);
  float Irms = (rms * 3.3 / 4095.0) * 1000.0;  // in mV
  Irms = Irms / sensitivity;                   // in A
  if(Irms < 0.05) Irms = 0;                   // filter noise
  return Irms;
}

*/

// ---------------------------

float energy_kWh = 0;
unsigned long lastMillis = 0;

// ----------------------------
// ACS712 zero calibration
// ----------------------------
int calibrateACSZero(int samples=1000){
  long sum = 0;
  Serial.println("Calibrating ACS712 zero, remove load...");
  delay(2000);
  for(int i=0;i<samples;i++){
    sum += analogRead(ACS_PIN);
    delayMicroseconds(50);
  }
  int zero = sum / samples;
  return zero;
}


void setup(){
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0,0);
  lcd.print("Smart Energy Meter");
  delay(1500);

  // Auto calibrate ACS712 zero
  ACS_ZERO_RAW = calibrateACSZero();
  Serial.print("ACS712 zero raw value: ");
  Serial.println(ACS_ZERO_RAW);
}

void loop(){

  // Sample Vrms
  float Vrms = readVrms(ZMPT_PIN, voltageCalibration);

  // Sample Irms
  float Irms = readIrms(ACS_PIN, ACS_ZERO_RAW, ACS_SENSITIVITY);

  // Power calculations
  float apparentPower = Vrms * Irms;
  float power_factor = 0.95; // estimated for bulb+fan, can refine later
  float realPower = apparentPower * power_factor;

  // Energy calculation
  unsigned long now = millis();
  float dt = (now - lastMillis) / 3600000.0; // hours
  lastMillis = now;
  energy_kWh += (realPower * dt) / 1000.0;

  float bill = energy_kWh * unit_rate;

  // ---------------- LCD DISPLAY ------------------
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("V:"); lcd.print(Vrms,1); lcd.print("V ");
  lcd.setCursor(11,0);
  lcd.print("PF:"); lcd.print(power_factor,2);

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
  Serial.print(" W, PF: "); Serial.print(power_factor,2);
  Serial.print(", kWh: "); Serial.print(energy_kWh,3);
  Serial.print(", Bill: "); Serial.println(bill,2);

  delay(500);
}


