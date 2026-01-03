#include <Arduino.h>
#include <math.h>

// ---------- BTS7960 pins ----------
const int RPWM_PIN = 14;
const int LPWM_PIN = 15;
const int REN_PIN  = 26;
const int LEN_PIN  = 25;

const int PWM_CH_R = 0;
const int PWM_CH_L = 1;
const int PWM_FREQ = 20000;   // 20 kHz
const int PWM_RES  = 8;       // 0–255

// ---------- Current sensor ----------
const int   ADC_PIN = 34;     // ADC1 pin recommended
const float SENSOR_VCC = 3.3; // set sesuai supply sensor kamu (3.3 atau 5.0)

// ACS758 "100B": typ 20 mV/A @5V, ratiometric terhadap VCC
float sens_mV_per_A = 20.0f * (SENSOR_VCC / 5.0f);

float offset_mV = 0.0f;

// filter / sampling
const int OFFSET_SAMPLES = 800;   // makin besar makin stabil
const int READ_SAMPLES   = 80;

// deadband: di bawah ini dianggap 0A (tune sesuai noise sistem kamu)
float DEAD_BAND_A = 0.30f;

// auto-zero saat STOP (lebih kecil = lebih lambat tapi lebih stabil)
float OFFSET_TRACK_ALPHA = 0.002f; // 0.2% per update

// ---------- timing ----------
enum State { RUN, STOP };
State st = STOP;

uint32_t t_state = 0;
const uint32_t DUR_RUN_MS  = 10000;
const uint32_t DUR_STOP_MS = 2000;

uint8_t dutyRun = 180;

uint32_t t_print = 0;
const uint32_t PRINT_INTERVAL_MS = 1000; // 1 detik

// ---------- Motor helpers ----------
void pwmSetup() {
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(RPWM_PIN, PWM_CH_R);
  ledcAttachPin(LPWM_PIN, PWM_CH_L);
}

void motorStopPWM() {
  ledcWrite(PWM_CH_R, 0);
  ledcWrite(PWM_CH_L, 0);
}

void driverEnable(bool en) {
  digitalWrite(REN_PIN, en ? HIGH : LOW);
  digitalWrite(LEN_PIN, en ? HIGH : LOW);
}

void motorForward(uint8_t duty) {
  ledcWrite(PWM_CH_R, duty);
  ledcWrite(PWM_CH_L, 0);
}

void motorReverse(uint8_t duty) {
  ledcWrite(PWM_CH_R, 0);
  ledcWrite(PWM_CH_L, duty);
}

// ---------- ADC helpers ----------
float readMilliVoltsAvg(int pin, int n) {
  uint32_t sum = 0;
  for (int i = 0; i < n; i++) {
    sum += analogReadMilliVolts(pin);
    delayMicroseconds(300);
  }
  return (float)sum / (float)n;
}

void calibrateOffset() {
  // Pastikan benar-benar “tenang”
  motorStopPWM();
  driverEnable(false);
  delay(300);

  offset_mV = readMilliVoltsAvg(ADC_PIN, OFFSET_SAMPLES);
}

float readCurrentA(float *vout_mV = nullptr) {
  float v_mV = readMilliVoltsAvg(ADC_PIN, READ_SAMPLES);
  if (vout_mV) *vout_mV = v_mV;

  float I = (v_mV - offset_mV) / sens_mV_per_A;

  // deadband supaya noise tidak jadi arus palsu
  if (fabsf(I) < DEAD_BAND_A) I = 0.0f;

  return I;
}

void setup() {
  Serial.begin(115200);

  // Driver pins
  pinMode(REN_PIN, OUTPUT);
  pinMode(LEN_PIN, OUTPUT);

  // PWM
  pwmSetup();
  motorStopPWM();
  driverEnable(false); // mulai dalam keadaan disable

  // ADC
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);

  // kalibrasi offset awal (harus 0A)
  calibrateOffset();

  Serial.println("=== READY ===");
  Serial.printf("ACS758 100B | SENSOR_VCC=%.2f V | sens=%.2f mV/A\n", SENSOR_VCC, sens_mV_per_A);
  Serial.printf("Offset=%.1f mV | Deadband=%.2f A\n", offset_mV, DEAD_BAND_A);
  Serial.println("Ketik 'r' untuk RUN, 's' untuk STOP, 'z' untuk re-zero offset.");

  st = STOP;
  t_state = millis();
}

void loop() {
  uint32_t now = millis();

  // --- optional control via Serial ---
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'r') { st = RUN;  t_state = now; }
    if (c == 's') { st = STOP; t_state = now; }
    if (c == 'z') {
      calibrateOffset();
      Serial.printf("Re-zero done. Offset=%.1f mV\n", offset_mV);
    }
  }

  // --- auto state cycling (RUN 10s, STOP 2s) ---
  if (st == RUN && (now - t_state) >= DUR_RUN_MS) {
    st = STOP;
    t_state = now;
  } else if (st == STOP && (now - t_state) >= DUR_STOP_MS) {
    st = RUN;
    t_state = now;
  }

  // --- apply motor command ---
  if (st == RUN) {
    driverEnable(true);
    motorForward(dutyRun); // ganti ke motorReverse(dutyRun) kalau perlu
  } else {
    // STOP: benar-benar matikan PWM + disable driver
    motorStopPWM();
    driverEnable(false);
  }

  // --- read current + optional offset tracking when STOP ---
  float vout_mV = 0.0f;
  float I = readCurrentA(&vout_mV);

  // Saat STOP dan arus dianggap 0, update offset perlahan (offset tracking)
  if (st == STOP && I == 0.0f) {
    offset_mV = (1.0f - OFFSET_TRACK_ALPHA) * offset_mV + OFFSET_TRACK_ALPHA * vout_mV;
  }

  // --- print slower ---
  if (now - t_print >= PRINT_INTERVAL_MS) {
    t_print = now;
    Serial.printf("t=%lu ms | state=%s | duty=%d | Vout=%.1f mV | offset=%.1f mV | I=%.3f A\n",
                  (unsigned long)now,
                  (st == RUN ? "RUN " : "STOP"),
                  (st == RUN ? dutyRun : 0),
                  vout_mV, offset_mV, I);
  }
}
