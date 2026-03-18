#include <Arduino.h>

/* ===================== PINS ===================== */
// -------- Joint 1 --------
#define ENC1_A 32
#define ENC1_B 33
#define IN1_1  12
#define IN2_1  14
#define ENA_1  13

// -------- Joint 2 --------
#define ENC2_A 18
#define ENC2_B 19
#define IN1_2  25
#define IN2_2  26
#define ENA_2  27

/* ===================== PWM ===================== */
#define PWM_FREQ 20000
#define PWM_RES  8
#define PWM_CH1  0
#define PWM_CH2  1

/* ===================== ENCODERS ===================== */
volatile long enc1 = 0;
volatile long enc2 = 0;

void IRAM_ATTR isr1A() { enc1 += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? 1 : -1; }
void IRAM_ATTR isr1B() { enc1 += (digitalRead(ENC1_A) != digitalRead(ENC1_B)) ? 1 : -1; }

void IRAM_ATTR isr2A() { enc2 += (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ? 1 : -1; }
void IRAM_ATTR isr2B() { enc2 += (digitalRead(ENC2_A) != digitalRead(ENC2_B)) ? 1 : -1; }

/* ===================== PARAMETERS ===================== */
const float CPR = 44.0;
const float gear = 44.727;
const float radPerCount = TWO_PI / (CPR * gear);

const float V_DEAD = 2.0;

/* ===================== GAINS ===================== */
float Kp_move = 50, Kd_move = 10;
float Kp_hold = 30, Kd_hold = 4;

/* ===================== STATE ===================== */
float q1=0, dq1=0, q1_last=0;
float q2=0, dq2=0, q2_last=0;

/* ===================== TRAJECTORY ===================== */
float q1_0=0, q1_f=0, q1_d=0, dq1_d=0, ddq1_d=0;
float q2_0=0, q2_f=0, q2_d=0, dq2_d=0, ddq2_d=0;

float Ttraj = 1.5;
float t_traj = 0;

enum Mode { MOVE, HOLD };
Mode mode = HOLD;

/* ===================== MOTOR DRIVE ===================== */
void motorDrive(int in1, int in2, int ch, float V) {
  V = constrain(V, -12.0, 12.0);
  int pwm = abs(V) * 255.0 / 12.0;

  if (V > 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (V < 0) { digitalWrite(in1, LOW); digitalWrite(in2, HIGH); }
  else pwm = 0;

  ledcWrite(ch, pwm);
}

/* ===================== SETUP ===================== */
unsigned long lastMicros;

void setup() {
  Serial.begin(115200);

  pinMode(IN1_1,OUTPUT); pinMode(IN2_1,OUTPUT);
  pinMode(IN1_2,OUTPUT); pinMode(IN2_2,OUTPUT);

  pinMode(ENC1_A,INPUT_PULLUP); pinMode(ENC1_B,INPUT_PULLUP);
  pinMode(ENC2_A,INPUT_PULLUP); pinMode(ENC2_B,INPUT_PULLUP);

  attachInterrupt(ENC1_A, isr1A, CHANGE);
  attachInterrupt(ENC1_B, isr1B, CHANGE);
  attachInterrupt(ENC2_A, isr2A, CHANGE);
  attachInterrupt(ENC2_B, isr2B, CHANGE);

  ledcSetup(PWM_CH1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA_1, PWM_CH1);
  ledcAttachPin(ENA_2, PWM_CH2);

  lastMicros = micros();
  Serial.println("2-DOF Hybrid Controller READY");
}

/* ===================== LOOP ===================== */
void loop() {

  /* ----- SERIAL COMMAND ----- */
  if (Serial.available()) {
    char c = Serial.read();
    if (c=='T'||c=='t') {
      q1_0 = q1; q2_0 = q2;
      q1_f = Serial.parseFloat()*DEG_TO_RAD;
      q2_f = Serial.parseFloat()*DEG_TO_RAD;
      t_traj = 0;
      mode = MOVE;
      Serial.println("2-DOF Trajectory STARTED");
    }
  }

  unsigned long now = micros();
  float dt = (now-lastMicros)*1e-6;
  if (dt < 0.002) return;
  lastMicros = now;

  /* ----- STATE ----- */
  long c1,c2;
  noInterrupts(); c1=enc1; c2=enc2; interrupts();

  q1 = c1*radPerCount;  dq1 = (q1-q1_last)/dt;  q1_last=q1;
  q2 = c2*radPerCount;  dq2 = (q2-q2_last)/dt;  q2_last=q2;

  float V1=0, V2=0;

  /* ----- MOVE MODE ----- */
  if (mode==MOVE) {
    t_traj += dt;
    float s = constrain(t_traj/Ttraj,0,1);

    q1_d = q1_0 + (q1_f-q1_0)*(3*s*s-2*s*s*s);
    q2_d = q2_0 + (q2_f-q2_0)*(3*s*s-2*s*s*s);

    dq1_d = (6*(q1_f-q1_0)/Ttraj)*(s-s*s);
    dq2_d = (6*(q2_f-q2_0)/Ttraj)*(s-s*s);

    ddq1_d = (6*(q1_f-q1_0)/(Ttraj*Ttraj))*(1-2*s);
    ddq2_d = (6*(q2_f-q2_0)/(Ttraj*Ttraj))*(1-2*s);

    V1 = ddq1_d + Kp_move*(q1_d-q1) + Kd_move*(dq1_d-dq1);
    V2 = ddq2_d + Kp_move*(q2_d-q2) + Kd_move*(dq2_d-dq2);

    if (s>=1.0) mode=HOLD;
  }

  /* ----- HOLD MODE ----- */
  else {
    V1 = Kp_hold*(q1_f-q1) + Kd_hold*(-dq1);
    V2 = Kp_hold*(q2_f-q2) + Kd_hold*(-dq2);

    if (fabs(q1_f-q1)>0.5*DEG_TO_RAD && fabs(V1)<V_DEAD) V1=copysign(V_DEAD,V1);
    if (fabs(q2_f-q2)>0.5*DEG_TO_RAD && fabs(V2)<V_DEAD) V2=copysign(V_DEAD,V2);
  }

  motorDrive(IN1_1,IN2_1,PWM_CH1,V1);
  motorDrive(IN1_2,IN2_2,PWM_CH2,V2);

  /* ----- DEBUG ----- */
  Serial.print("q1: "); Serial.print(q1*RAD_TO_DEG,1);
  Serial.print(" q2: "); Serial.print(q2*RAD_TO_DEG,1);
  Serial.print(" mode: "); Serial.println(mode==MOVE?"MOVE":"HOLD");
}
