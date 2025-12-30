/* Calibration helper for ZMPT101B on ESP32
   - Uses LiquidCrystal_I2C for a 20x4 LCD.
   - Stores calibration factor in Preferences (persistent).
   - After calibration it applies the factor to displayed voltage.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ZMPT101B.h>
#include <Preferences.h>

#define SENSITIVITY 500.0f    // initial sensitivity - may need tuning
// Change the analog pin to whatever you're using. If using ADC pin 35, use 35.
ZMPT101B voltageSensor(35, 50.0); // sensor pin, mains frequency 50 Hz

LiquidCrystal_I2C lcd(0x27, 20, 4); // change address if needed
Preferences prefs;

float calibrationFactor = 1.0; // saved factor
const char *PREF_NAMESPACE = "zmpt"; 
const char *KEY_FACTOR = "kfactor";

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); } // wait for serial on some boards

  pinMode(4, OUTPUT); // buzzer if needed

  voltageSensor.setSensitivity(SENSITIVITY);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  // load saved calibration factor
  prefs.begin(PREF_NAMESPACE, false);
  calibrationFactor = prefs.getFloat(KEY_FACTOR, 1.0f); // default 1.0
  prefs.end();

  lcd.setCursor(0,0);
  lcd.print("ZMPT Calibration");
  lcd.setCursor(0,1);
  lcd.print("Factor:");
  lcd.print(calibrationFactor, 6);
  delay(1500);
  lcd.clear();

  Serial.println();
  Serial.println("ZMPT101B calibration helper");
  Serial.print("Current saved factor: ");
  Serial.println(calibrationFactor, 6);
  Serial.println("If you want to recalibrate, type 'c' and press Enter.");
  Serial.println("Otherwise the system will just display corrected voltage.");
}

void loop() {
  float rawV = voltageSensor.getRmsVoltage(); // measured by library
  float corrected = rawV * calibrationFactor;

  // display on LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Voltage (RMS):");
  lcd.setCursor(0,1);
  lcd.print(corrected, 2);
  lcd.print(" V");
  lcd.setCursor(0,3);
  lcd.print("K=");
  lcd.print(calibrationFactor, 6);

  // print to serial for info
  Serial.print("Raw: ");
  Serial.print(rawV, 4);
  Serial.print("  Corrected: ");
  Serial.println(corrected, 4);

  // check for serial command to recalibrate
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s == "c" || s == "C") {
      doCalibration();
    } else {
      // If serial input looks numeric, treat it as measured true voltage
      bool isNumber = s.length() > 0 && (isDigit(s.charAt(0)) || s.charAt(0) == '-' || s.charAt(0)=='.');
      if (isNumber) {
        float trueV = s.toFloat();
        if (trueV > 0.1) {
          float measured = voltageSensor.getRmsVoltage();
          float k = trueV / measured;
          calibrationFactor = k;
          prefs.begin(PREF_NAMESPACE, false);
          prefs.putFloat(KEY_FACTOR, calibrationFactor);
          prefs.end();
          Serial.print("Saved new factor: ");
          Serial.println(calibrationFactor, 8);
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Saved K=");
          lcd.print(calibrationFactor, 6);
          delay(1500);
        }
      }
    }
  }

  delay(1000);
}

void doCalibration() {
  Serial.println();
  Serial.println("=== Calibration procedure ===");
  Serial.println("1) Connect your multimeter to the same mains.");
  Serial.println("2) Measure RMS voltage with multimeter (e.g. 221.0).");
  Serial.println("3) Type the multimeter value here in Serial and press Enter.");
  Serial.println("Example: 221");
  Serial.println("============================");
}
