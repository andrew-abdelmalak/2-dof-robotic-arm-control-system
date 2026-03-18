/* =========================================================
   2-DOF ROBOT ARM – JOINT-SPACE CIRCLE TRAJECTORY
   q1d = q1c + R cos(wt)
   q2d = q2c + R sin(wt)

   - Decoupled computed torque
   - Conditional voltage dead-zone compensation
   - Parameters from init_params.m
   ========================================================= */

#include <Arduino.h>

/* ===================== PINS ===================== */
// Motor 1
const int ENC1_A = 2;
const int ENC1_B = 3;
const int IN1_1  = 8;
const int IN2_1  = 9;
const int ENA_1  = 10;

// Motor 2
const int ENC2_A = A4;   // A4
const int ENC2_B = A5;   // A5
const int IN1_2  = 6;
const int IN2_2  = 7;
const int ENA_2  = 5;

/* ===================== ENCODERS ===================== */
volatile long enc1 = 0;
volatile long enc2 = 0;

void isrEnc1A() { enc1 += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? 1 : -1; }
void isrEnc1B() { enc1 += (digitalRead(ENC1_A) != digitalRead(ENC1_B)) ? 1 : -1; }

ISR(PCINT1_vect) {
  static uint8_t last = PINC;
  uint8_t curr = PINC;
  uint8_t diff = curr ^ last;

  if (diff & _BV(4)) enc2 += (bitRead(curr, 4) == bitRead(curr, 5)) ? 1 : -1;
  if (diff & _BV(5)) enc2 += (bitRead(curr, 4) != bitRead(curr, 5)) ? 1 : -1;

  last = curr;
}

/* ===================== PARAMETERS (FROM init_params.m) ===================== */
// COM distances
const float a1 = 0.05162;
const float a2 = 0.00504;

// Total masses
const float m1 = 1.020 + 0.09045;
const float m2 = 1.109;

// Total inertias
const float I1 = 8.0234e-05 + 7.0248e-06 + 7.0248e-06;
const float I2 = 1.4850e-05 + 7.0248e-06;

// Effective inertias
const float M1 = I1 + m1 * a1 * a1;
const float M2 = I2 + m2 * a2 * a2;

// Motor constants
const float b1 = 0.8;
const float b2 = 0.4;
const float d1 = 0.01;
const float d2 = 0.01;

// Gains
const float Kp1 = 8000;
const float Kd1 = 500;
const float Kp2 = 20000;
const float Kd2 = 500;

/* ===================== ENCODER SCALE ===================== */
const float radPerCount = TWO_PI / (44.0 * 44.727);

/* ===================== TRAJECTORY ===================== */
float q1c = 0.0;
float q2c = 0.0;
float R = 60.0 * DEG_TO_RAD;
float w = 1.5;
float t = 0.0;

/* ===================== STATE ===================== */
float q1, dq1, q2, dq2;
float q1_last = 0, q2_last = 0;

/* ===================== DEAD-ZONE ===================== */
const float V_DEAD = 1;
const float DQ_EPS = 0.05;

/* ===================== TIMING ===================== */
unsigned long lastMicros;

/* ===================== MOTOR DRIVE ===================== */
void driveMotor(int in1, int in2, int ena, float V) {
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

  analogWrite(ena, pwm);
}

/* ===================== DEAD-ZONE HANDLER ===================== */
float applyDeadZone(float V, float dq, float dq_d) {
  if (fabs(dq) < DQ_EPS && fabs(dq_d) < DQ_EPS && fabs(V) > 0 && fabs(V) < V_DEAD) {
    return copysign(V_DEAD, V);
  }
  return V;
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);

  pinMode(IN1_1, OUTPUT); pinMode(IN2_1, OUTPUT); pinMode(ENA_1, OUTPUT);
  pinMode(IN1_2, OUTPUT); pinMode(IN2_2, OUTPUT); pinMode(ENA_2, OUTPUT);

  pinMode(ENC1_A, INPUT_PULLUP); pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP); pinMode(ENC2_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), isrEnc1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), isrEnc1B, CHANGE);

  PCICR |= _BV(PCIE1);
  PCMSK1 |= _BV(PCINT12) | _BV(PCINT13);

  lastMicros = micros();
  Serial.println("2-DOF Joint-Space Circle Controller READY");
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

  // Joint-space circle
  float q1d = q1c + R * cos(w * t);
  float q2d = q2c + R * sin(w * t);

  float dq1d = -R * w * sin(w * t);
  float dq2d =  R * w * cos(w * t);

  float ddq1d = -R * w * w * cos(w * t);
  float ddq2d = -R * w * w * sin(w * t);

  float v1 = ddq1d + Kp1 * (q1d - q1) + Kd1 * (dq1d - dq1);
  float v2 = ddq2d + Kp2 * (q2d - q2) + Kd2 * (dq2d - dq2);

  float V1 = (M1 * v1 + d1 * dq1) / b1;
  float V2 = (M2 * v2 + d2 * dq2) / b2;

  V1 = applyDeadZone(V1, dq1, dq1d);
  V2 = applyDeadZone(V2, dq2, dq2d);

  driveMotor(IN1_1, IN2_1, ENA_1, V1);
  driveMotor(IN1_2, IN2_2, ENA_2, V2);

  Serial.print("q1: "); Serial.print(q1 * RAD_TO_DEG, 1);
  Serial.print(" q2: "); Serial.print(q2 * RAD_TO_DEG, 1);
  Serial.print(" | V1: "); Serial.print(V1, 2);
  Serial.print(" V2: "); Serial.println(V2, 2);
    Serial.print(" q2d: "); Serial.println(q2d, 2);

}
