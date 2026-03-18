#include <Arduino.h>

/* ===================== PINS ===================== */
// Motor 1
#define ENC1_A 34
#define ENC1_B 35
#define IN1_1  25
#define IN2_1  26
#define ENA_1  27

// Motor 2
#define ENC2_A 32
#define ENC2_B 33
#define IN1_2  14
#define IN2_2  12
#define ENA_2  13

/* ===================== PWM ===================== */
#define PWM_FREQ 20000
#define PWM_RES  8
#define PWM_CH1  0
#define PWM_CH2  1

/* ===================== ENCODERS ===================== */
volatile long enc1 = 0;
volatile long enc2 = 0;

/* --- Encoder 1: unchanged (dual interrupt) --- */
void IRAM_ATTR isrEnc1A() {
  enc1 += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? 1 : -1;
}
void IRAM_ATTR isrEnc1B() {
  enc1 += (digitalRead(ENC1_A) != digitalRead(ENC1_B)) ? 1 : -1;
}

/* --- Encoder 2: SENSITIVITY FIX (single interrupt) --- */
void IRAM_ATTR isrEnc2A() {
  enc2 += (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ? 1 : -1;
}

/* ===================== ROBOT GEOMETRY ===================== */
const float l1 = 0.12;
const float l2 = 0.15;

/* ===================== MOTOR & CONTROL ===================== */
const float b1 = 0.8, b2 = 0.8;
const float d1 = 0.01, d2 = 0.01;
const float M1 = 0.1125;
const float M2 = 0.0012;

const float Kp1 = 200.0, Kd1 = 5.0;
const float Kp2 = 800.0, Kd2 = 10.0;

/* ===================== ENCODER SCALE ===================== */
const float radPerCount = TWO_PI / (44.0 * 44.727);

/* ===================== CARTESIAN CIRCLE ===================== */
const float xc = 0.18;
const float yc = 0.00;
const float R  = 0.04;
const float w  = 2.0;

/* ===================== STATE ===================== */
float q1, q2, dq1, dq2;
float q1_last = 0, q2_last = 0;
float q1d = 0, q2d = 0;
float q1d_last = 0, q2d_last = 0;

float t = 0;
unsigned long lastMicros;

/* ===================== MOTOR DRIVE ===================== */
void driveMotor(int in1, int in2, int pwm_ch, float V) {
  V = constrain(V, -12.0, 12.0);
  int pwm = abs(V) * 255.0 / 12.0;

  if (V > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (V < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    pwm = 0;
  }

  ledcWrite(pwm_ch, pwm);
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);

  pinMode(IN1_1, OUTPUT); pinMode(IN2_1, OUTPUT);
  pinMode(IN1_2, OUTPUT); pinMode(IN2_2, OUTPUT);

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA_1, PWM_CH1);
  ledcAttachPin(ENA_2, PWM_CH2);

  attachInterrupt(ENC1_A, isrEnc1A, CHANGE);
  attachInterrupt(ENC1_B, isrEnc1B, CHANGE);

  /* --- SENSITIVITY FIX APPLIED HERE --- */
  attachInterrupt(ENC2_A, isrEnc2A, RISING);

  lastMicros = micros();

  Serial.println("ESP32 2-DOF Cartesian Circle Controller READY");
  Serial.println("q1(deg), q1d(deg), V1(V), q2(deg), q2d(deg), V2(V)");
}

/* ===================== LOOP ===================== */
void loop() {

  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6;
  if (dt < 0.002) return;
  lastMicros = now;
  t += dt;

  noInterrupts();
  long c1 = enc1;
  long c2 = enc2;
  interrupts();

  q1 = c1 * radPerCount;
  q2 = c2 * radPerCount;

  dq1 = (q1 - q1_last) / dt;
  dq2 = (q2 - q2_last) / dt;
  q1_last = q1;
  q2_last = q2;

  float xd = xc + R * cos(w * t);
  float yd = yc + R * sin(w * t);

  float cos_q2 = (xd*xd + yd*yd - l1*l1 - l2*l2) / (2*l1*l2);
  cos_q2 = constrain(cos_q2, -1.0, 1.0);

  q2d = atan2(sqrt(1.0 - cos_q2*cos_q2), cos_q2);
  q1d = atan2(yd, xd) - atan2(l2*sin(q2d), l1 + l2*cos(q2d));

  float dq1d = (q1d - q1d_last) / dt;
  float dq2d = (q2d - q2d_last) / dt;
  q1d_last = q1d;
  q2d_last = q2d;

  float v1 = Kp1 * (q1d - q1) + Kd1 * (dq1d - dq1);
  float v2 = Kp2 * (q2d - q2) + Kd2 * (dq2d - dq2);

  float V1 = (M1 * v1 + d1 * dq1) / b1;
  float V2 = (M2 * v2 + d2 * dq2) / b2;

  driveMotor(IN1_1, IN2_1, PWM_CH1, V1);
  driveMotor(IN1_2, IN2_2, PWM_CH2, V2);

  /* ---------- TELEMETRY ---------- */
  Serial.print("q1 = ");
  Serial.print(q1 * RAD_TO_DEG, 2);
  Serial.print(" | q1d = ");
  Serial.print(q1d * RAD_TO_DEG, 2);
  Serial.print(" | V1 = ");
  Serial.print(V1, 2);
  Serial.print(" || ");

  Serial.print("q2 = ");
  Serial.print(q2 * RAD_TO_DEG, 2);
  Serial.print(" | q2d = ");
  Serial.print(q2d * RAD_TO_DEG, 2);
  Serial.print(" | V2 = ");
  Serial.println(V2, 2);

  Serial.print("enc1 = ");
  Serial.print(c1);
  Serial.print(" | enc2 = ");
  Serial.println(c2);
}
