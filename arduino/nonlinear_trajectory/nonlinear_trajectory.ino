/* =========================================================
   1-DOF ROBOT ARM – HYBRID CONTROL
   - Trajectory tracking (MOVE)
   - Dead-zone compensated PD (HOLD)
   ========================================================= */

#include <Arduino.h>

/* ===================== PINS ===================== */
const int ENC_A = 2;
const int ENC_B = 3;
const int IN1   = 8;
const int IN2   = 9;
const int ENA   = 10;

/* ===================== ENCODER ===================== */
volatile long encoderCount = 0;
void isrA() { encoderCount += (digitalRead(ENC_A) == digitalRead(ENC_B)) ? 1 : -1; }
void isrB() { encoderCount += (digitalRead(ENC_A) != digitalRead(ENC_B)) ? 1 : -1; }

/* ===================== ROBOT PARAMS ===================== */
const float a1 = 0.08162;
const float m1 = 0.119;
const float I1 = 9.73e-05;
const float M_eff = I1 + m1 * a1 * a1;

/* ===================== MOTOR (ESTIMATED) ===================== */
const float b1 = 0.8;
const float d1 = 0.1;

/* ===================== GAINS ===================== */
float Kp_move = 50.0;
float Kd_move = 10.0;

float Kp_hold = 30.0;
float Kd_hold = 4.0;

/* ===================== DEAD-ZONE ===================== */
const float V_DEAD = 2;   // minimum voltage to move motor

/* ===================== ENCODER ===================== */
const float CPR = 44.0;
const float gear = 44.727;
const float radPerCount = TWO_PI / (CPR * gear);

/* ===================== STATE ===================== */
float q = 0, dq = 0, last_q = 0;

/* ===================== TRAJECTORY ===================== */
float q0 = 0, qf = 0;
float qd = 0, dq_d = 0, ddq_d = 0;
float Ttraj = 1.5;
float t_traj = 0;

enum Mode { MOVE, HOLD };
Mode mode = HOLD;

/* ===================== TIMING ===================== */
unsigned long lastMicros;

/* ===================== MOTOR DRIVE ===================== */
void motorDrive(float V) {
  V = constrain(V, -12.0, 12.0);
  int pwm = abs(V) * 255.0 / 12.0;

  if (V > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else if (V < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  else pwm = 0;

  analogWrite(ENA, pwm);
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, CHANGE);

  lastMicros = micros();
  Serial.println("Hybrid trajectory + hold controller ready");
}

/* ===================== LOOP ===================== */
void loop() {

  /* ---------- SERIAL ---------- */
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'T' || c == 't') {
      q0 = q;
      qf = Serial.parseFloat() * DEG_TO_RAD;
      t_traj = 0;
      mode = MOVE;
      Serial.println("Trajectory started");
    }
  }

  /* ---------- TIMING ---------- */
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6;
  if (dt < 0.002) return;
  lastMicros = now;

  /* ---------- STATE ---------- */
  long c;
  noInterrupts(); c = encoderCount; interrupts();
  q = c * radPerCount;
  dq = (q - last_q) / dt;
  last_q = q;

  float V = 0;

  /* ---------- MOVE MODE ---------- */
  if (mode == MOVE) {
    t_traj += dt;
    float s = constrain(t_traj / Ttraj, 0.0, 1.0);

    qd = q0 + (qf - q0) * (3*s*s - 2*s*s*s);
    dq_d = (6*(qf - q0)/Ttraj) * (s - s*s);
    ddq_d = (6*(qf - q0)/(Ttraj*Ttraj)) * (1 - 2*s);

    float e  = qd - q;
    float ed = dq_d - dq;

    float v = ddq_d + Kp_move*e + Kd_move*ed;
    V = (M_eff*v + d1*dq) / b1;

    if (s >= 1.0) {
      mode = HOLD;
    }
  }

  /* ---------- HOLD MODE ---------- */
  else {
    float e  = qf - q;
    float ed = -dq;

    V = Kp_hold*e + Kd_hold*ed;

    // dead-zone compensation
    if (fabs(e) > 0.5 * DEG_TO_RAD && fabs(V) < V_DEAD) {
      V = copysign(V_DEAD, V);
    }
  }

  motorDrive(V);

  /* ---------- DEBUG ---------- */
  Serial.print("q: "); Serial.print(q * RAD_TO_DEG, 2);
  Serial.print(" V: "); Serial.print(V, 2);
  Serial.print(" mode: "); Serial.println(mode == MOVE ? "MOVE" : "HOLD");
}
