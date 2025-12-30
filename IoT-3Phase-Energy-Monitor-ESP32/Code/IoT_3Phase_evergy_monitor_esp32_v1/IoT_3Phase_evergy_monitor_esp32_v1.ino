// ESP32: power factor using ZMPT101B and ACS712
// Assumes Arduino core for ESP32

const int V_PIN = 35; // ZMPT101B analog output to ADC1_CH6
const int I_PIN = 34; // ACS712 analog output to ADC1_CH7

// Calibration constants (YOU MUST CALIBRATE these on your board)
const float ADC_MAX = 4095.0;      // 12-bit ADC
const float VREF = 3.3;            // ESP32 ADC reference (approx)
const float VOLTAGE_DIVIDER_GAIN = 1.0; // If ZMPT board scales down, put factor here (calibrate)
const float ZMPT_SENSITIVITY = 1.0; // convert measured volts (after center removal) to mains volts (calibrate)
                                      // e.g. if ADC->volts * ZMPT_SENSITIVITY = actual mains volts
const float ACS_V_PER_A = 0.066;   // ACS712 sensitivity in V/A (example: 66mV/A for 30A version)
// ACS offset (V) — ideally Vcc/2 but read dynamically in calibration
float acsOffset = 0.0;
float zmptOffset = 0.0;

const int N = 5000; // number of samples (tune this; larger = smoother)
const int samplingDelayMicros = 200; // sampling interval in microseconds (gives 5 kHz: 200us -> 5kHz)

void setup() {
  Serial.begin(115200);
  analogSetPinAttenuation(V_PIN, ADC_11db);
  analogSetPinAttenuation(I_PIN, ADC_11db);
  delay(1000);
  Serial.println("Calibrating offsets...");

  // Quick offset calibration (no current & no voltage reference required? do while voltage present)
  // For better calibration, put a known-zero current load and measure averages.
  const int CAL_SAMPLES = 500;
  long vsum = 0, isum = 0;
  for (int k=0;k<CAL_SAMPLES;k++){
    vsum += analogRead(V_PIN);
    isum += analogRead(I_PIN);
    delay(2);
  }
  zmptOffset = (float)vsum / CAL_SAMPLES;
  acsOffset = (float)isum / CAL_SAMPLES;

  Serial.print("zmptOffset ADC: "); Serial.println(zmptOffset);
  Serial.print("acsOffset  ADC: "); Serial.println(acsOffset);
  delay(2000);
}

void loop() {
  // Buffers not stored (we compute running sums to save RAM)
  double v_sum = 0, v_sq_sum = 0;
  double i_sum = 0, i_sq_sum = 0;
  double p_sum = 0;

  unsigned long startMicros = micros();
  for (int n=0; n<N; n++){
    int rawV = analogRead(V_PIN);
    int rawI = analogRead(I_PIN);

    // Convert raw ADC to voltage (approx)
    float v_adc = ((float)rawV / ADC_MAX) * VREF;
    float i_adc = ((float)rawI / ADC_MAX) * VREF;

    // remove offsets (convert offsets from ADC counts to voltage)
    float zmptOffsetV = (zmptOffset / ADC_MAX) * VREF;
    float acsOffsetV  = (acsOffset  / ADC_MAX) * VREF;

    // Centered signals (AC components)
    float v_ac = (v_adc - zmptOffsetV) * VOLTAGE_DIVIDER_GAIN * ZMPT_SENSITIVITY; // maps to mains volts
    float i_ac = (i_adc - acsOffsetV) / ACS_V_PER_A; // maps to amps (signed)

    // accumulate sums
    v_sum += v_ac;
    i_sum += i_ac;
    v_sq_sum += (double)v_ac * v_ac;
    i_sq_sum += (double)i_ac * i_ac;
    p_sum += (double)v_ac * i_ac;

    // sample spacing
    // Busy-wait to maintain sampling rate
    unsigned long nextMicros = startMicros + (unsigned long)((n+1) * samplingDelayMicros);
    while (micros() < nextMicros) { /* spin */ }
  }

  // compute means and RMS
  double v_mean = v_sum / N;
  double i_mean = i_sum / N;
  double v_rms = sqrt( (v_sq_sum / N) - (v_mean * v_mean) );
  double i_rms = sqrt( (i_sq_sum / N) - (i_mean * i_mean) );

  // real power (average of instantaneous p)
  double real_power = p_sum / N;

  double apparent = v_rms * i_rms;
  double pf = 0.0;
  if (apparent > 1e-6) pf = real_power / apparent;

  // Print results
  Serial.print("Vrms: "); Serial.print(v_rms, 3); Serial.print(" V, ");
  Serial.print("Irms: "); Serial.print(i_rms, 3); Serial.print(" A, ");
  Serial.print("P: "); Serial.print(real_power, 3); Serial.print(" W, ");
  Serial.print("S: "); Serial.print(apparent, 3); Serial.print(" VA, ");
  Serial.print("PF: "); Serial.println(pf, 4);

  delay(100); // wait a bit before next window
}

