/* ESP32: 3x ACS712 + ZMPT101B energy meter + 20x4 I2C LCD
   - Computes Vrms, Irms (per channel), real power, accumulates energy (kWh)
   - Shows values and bill (10 Tk / unit)
   - Uses LiquidCrystal_I2C library (Install from Library Manager)
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); // change address if needed

// ADC pins
const int V_PIN = 35; // ZMPT101B
const int I1_PIN = 33; // ACS712 channel 1
const int I2_PIN = 34; // ACS712 channel 2
const int I3_PIN = 32; // ACS712 channel 3

// ADC params
const float ADC_MAX = 4095.0; // 12-bit
const float VREF = 3.3;       // approx (ESP32)

// Sampling
const int N = 3000; // sample window
const unsigned int SAMPLE_US = 200; // 5kHz approx -> 200us
// adjust N and SAMPLE_US to cover many mains cycles (50Hz -> 20ms per cycle)

// Calibration (SET THESE after calibration)
float ZMPT_OFFSET_ADC = 2048.0; // measured ADC mid value
float ACS1_OFFSET_ADC  = 2048.0;
float ACS2_OFFSET_ADC  = 2048.0;
float ACS3_OFFSET_ADC  = 2048.0;

// Sensitivities
const float ACS_V_PER_A = 0.066; // change per your ACS712 version (V/A)
const float ZMPT_VOLTSCALE = 230.0 / 0.5; // placeholder: (actual mains V) / measured Vpeak; calibrate

// Energy storage
Preferences prefs;
const char* PREF_KEY = "energy_kwh";
double energy_kwh = 0.0;

// Billing
const float PRICE_PER_UNIT = 10.0; // 10 Tk per kWh

// helper for converting ADC to voltage (approx)
float adcToVoltage(int raw) {
  return (raw / ADC_MAX) * VREF;
}

// Save energy periodically
unsigned long lastSaveMillis = 0;
const unsigned long SAVE_INTERVAL = 60000; // save every 60s

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA, SCL
  lcd.init();
  lcd.backlight();

  analogSetPinAttenuation(V_PIN, ADC_11db);
  analogSetPinAttenuation(I1_PIN, ADC_11db);
  analogSetPinAttenuation(I2_PIN, ADC_11db);
  analogSetPinAttenuation(I3_PIN, ADC_11db);

  // load persisted energy
  prefs.begin("energy", false);
  energy_kwh = prefs.getDouble(PREF_KEY, 0.0);
  Serial.printf("Loaded energy_kwh = %.6f\n", energy_kwh);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Energy Meter Boot");
  delay(800);
}

void loop() {
  // variables for running sums (offset removed later)
  double v_sum = 0, v_sq_sum = 0;
  double i1_sum = 0, i1_sq_sum = 0, p1_sum = 0;
  double i2_sum = 0, i2_sq_sum = 0, p2_sum = 0;
  double i3_sum = 0, i3_sq_sum = 0, p3_sum = 0;

  unsigned long startMicros = micros();
  for (int n=0; n<N; n++) {
    int rawV = analogRead(V_PIN);
    int rawI1 = analogRead(I1_PIN);
    int rawI2 = analogRead(I2_PIN);
    int rawI3 = analogRead(I3_PIN);

    float v_adc = ((float)rawV / ADC_MAX) * VREF;
    float i1_adc = ((float)rawI1 / ADC_MAX) * VREF;
    float i2_adc = ((float)rawI2 / ADC_MAX) * VREF;
    float i3_adc = ((float)rawI3 / ADC_MAX) * VREF;

    // Convert offsets from ADC to voltage
    float zmptOffsetV = (ZMPT_OFFSET_ADC / ADC_MAX) * VREF;
    float acs1OffsetV  = (ACS1_OFFSET_ADC / ADC_MAX) * VREF;
    float acs2OffsetV  = (ACS2_OFFSET_ADC / ADC_MAX) * VREF;
    float acs3OffsetV  = (ACS3_OFFSET_ADC / ADC_MAX) * VREF;

    // AC components (volts and amps)
    float v_ac = (v_adc - zmptOffsetV) * ZMPT_VOLTSCALE; // scaled to mains volts (calibrate)
    float i1_ac = (i1_adc - acs1OffsetV) / ACS_V_PER_A; // amps
    float i2_ac = (i2_adc - acs2OffsetV) / ACS_V_PER_A;
    float i3_ac = (i3_adc - acs3OffsetV) / ACS_V_PER_A;

    // accumulate sums
    v_sum += v_ac;
    v_sq_sum += (double)v_ac * v_ac;

    i1_sum += i1_ac; i1_sq_sum += (double)i1_ac * i1_ac; p1_sum += (double)v_ac * i1_ac;
    i2_sum += i2_ac; i2_sq_sum += (double)i2_ac * i2_ac; p2_sum += (double)v_ac * i2_ac;
    i3_sum += i3_ac; i3_sq_sum += (double)i3_ac * i3_ac; p3_sum += (double)v_ac * i3_ac;

    // precise sampling delay
    unsigned long nextMicros = startMicros + (unsigned long)((n+1) * SAMPLE_US);
    while (micros() < nextMicros) { /* wait */ }
  }

  double v_mean = v_sum / N;
  double v_rms = sqrt((v_sq_sum / N) - (v_mean * v_mean));

  double i1_mean = i1_sum / N;
  double i1_rms = sqrt((i1_sq_sum / N) - (i1_mean * i1_mean));
  double p1 = p1_sum / N;

  double i2_mean = i2_sum / N;
  double i2_rms = sqrt((i2_sq_sum / N) - (i2_mean * i2_mean));
  double p2 = p2_sum / N;

  double i3_mean = i3_sum / N;
  double i3_rms = sqrt((i3_sq_sum / N) - (i3_mean * i3_mean));
  double p3 = p3_sum / N;

  double P_total = p1 + p2 + p3;
  double S_total = v_rms * (i1_rms + i2_rms + i3_rms); // total apparent approx
  double pf_total = (S_total > 1e-6) ? (P_total / S_total) : 0.0;

  // Calculate elapsed time for this measurement window
  static unsigned long lastMillis = millis();
  unsigned long nowMillis = millis();
  unsigned long dtMillis = nowMillis - lastMillis;
  if (dtMillis == 0) dtMillis = (unsigned long)((N * SAMPLE_US) / 1000); // fallback
  lastMillis = nowMillis;

  // Integrate energy: P_total (W) * time (hours) -> Wh
  double dtHours = (double)dtMillis / 3600000.0; // ms -> hours
  double added_kwh = (P_total * dtHours) / 1000.0; // W * hours -> kWh
  energy_kwh += added_kwh;

  // Billing
  double bill_tk = energy_kwh * PRICE_PER_UNIT;

  // Display (rotate pages every loop)
  static int page = 0;
  page = (page + 1) % 3;

  lcd.clear();
  if (page == 0) {
    lcd.setCursor(0,0); lcd.printf("V RMS: %.1f V  PF: %.2f", v_rms, pf_total);
    lcd.setCursor(0,1); lcd.printf("I1:%.2fA I2:%.2fA I3:%.2fA", i1_rms, i2_rms, i3_rms);
    lcd.setCursor(0,2); lcd.printf("P1:%.1fW P2:%.1fW", p1, p2);
    lcd.setCursor(0,3); lcd.printf("P3:%.1fW Tot:%.1fW", p3, P_total);
  } else if (page == 1) {
    lcd.setCursor(0,0); lcd.printf("Energy: %.6f kWh", energy_kwh);
    lcd.setCursor(0,1); lcd.printf("Units: %.3f", energy_kwh);
    lcd.setCursor(0,2); lcd.printf("Rate: %.2f Tk/unit", PRICE_PER_UNIT);
    lcd.setCursor(0,3); lcd.printf("Bill: %.2f Tk", bill_tk);
  } else {
    lcd.setCursor(0,0); lcd.printf("Vrms:%.1f V  I1:%.2f A", v_rms, i1_rms);
    lcd.setCursor(0,1); lcd.printf("I2:%.2f A  I3:%.2f A", i2_rms, i3_rms);
    lcd.setCursor(0,2); lcd.printf("Ptot: %.1f W  PF:%.2f", P_total, pf_total);
    lcd.setCursor(0,3); lcd.printf("Saved: %.3f kWh", energy_kwh);
  }

  Serial.printf("Vrms:%.2f V I:%.2f,%.2f,%.2f P:%.2f kWh:%.6f Bill:%.2f\n",
                v_rms, i1_rms, i2_rms, i3_rms, P_total, energy_kwh, bill_tk);

  // Persist energy occasionally
  if (millis() - lastSaveMillis > SAVE_INTERVAL) {
    prefs.putDouble(PREF_KEY, energy_kwh);
    lastSaveMillis = millis();
    Serial.println("Saved energy to prefs.");
  }

  delay(300); // adjust UI refresh rate
}
