#include "MotorControl.h"
#include "DbgLog.h"

#ifdef USE_L298NX2
  static L298NX2 motors(EN_A, IN1, IN2, EN_B, IN3, IN4);
#else
  static L298N motorA(EN_A, IN1, IN2);
  static L298N motorB(EN_B, IN3, IN4);
#endif

static inline uint8_t clamp255(int v){ return (uint8_t)constrain(v, 0, 255); }

// Last commanded values for telemetry
static volatile char gLastDirA = 'F';
static volatile char gLastDirB = 'F';
static volatile int  gLastPwmA = 0;
static volatile int  gLastPwmB = 0;

void motor_get_last(char& dA, int& pA, char& dB, int& pB){
  noInterrupts();
  dA = gLastDirA; dB = gLastDirB;
  pA = gLastPwmA; pB = gLastPwmB;
  interrupts();
}

void motor_init(){
#ifdef USE_L298NX2
  motors.setSpeedA(0);
  motors.setSpeedB(0);
  motors.stop();
#else
  motorA.setSpeed(0);
  motorB.setSpeed(0);
  motorA.stop();
  motorB.stop();
#endif
  noInterrupts();
  gLastPwmA = 0; gLastPwmB = 0;
  gLastDirA = 'F'; gLastDirB = 'F';
  interrupts();
}

void motor_apply(char dA,int pA,char dB,int pB){
  int a = clamp255(pA);
  int b = clamp255(pB);

  noInterrupts();
  gLastDirA = dA; gLastDirB = dB;
  gLastPwmA = a;  gLastPwmB = b;
  interrupts();

#ifdef USE_L298NX2
  motors.setSpeedA(a);
  motors.setSpeedB(b);
  if (dA == 'B') motors.backwardA(); else motors.forwardA();
  if (dB == 'B') motors.backwardB(); else motors.forwardB();
#else
  motorA.setSpeed(a);
  motorB.setSpeed(b);
  if (dA == 'B') motorA.backward(); else motorA.forward();
  if (dB == 'B') motorB.backward(); else motorB.forward();
#endif
}

// --- STOP (implementation) ---
void motor_stop_impl(){
#ifdef USE_L298NX2
  motors.setSpeedA(0);
  motors.setSpeedB(0);
  motors.stop();
#else
  motorA.setSpeed(0);
  motorB.setSpeed(0);
  motorA.stop();
  motorB.stop();
#endif
  noInterrupts();
  gLastPwmA = 0; gLastPwmB = 0;
  gLastDirA = 'F'; gLastDirB = 'F';   // IMPORTANT: keep telemetry consistent
  interrupts();
}

// --- STOP (debug wrapper) ---
void motor_stop_dbg(const char* file, int line){
  dbg_push(EV_MOTOR_STOP, (int32_t)line, (int32_t)dbg_hash16(file), 0);

static uint32_t lastPrintMs = 0;
uint32_t now = millis();
if (now - lastPrintMs > 1000) {   // print at most 1/sec
  lastPrintMs = now;
  Serial.printf("[MOTOR] STOP called at %s:%d (t=%lu ms)\n",
                file, line, (unsigned long)now);
}
}

void motor_full_digital(char dA, char dB, int ms){
  ms = constrain(ms, 1, 5000);

  noInterrupts();
  gLastDirA = dA; gLastDirB = dB;
  gLastPwmA = 255; gLastPwmB = 255;
  interrupts();

#ifdef USE_L298NX2
  motors.setSpeedA(255); motors.setSpeedB(255);
  if (dA == 'B') motors.backwardA(); else motors.forwardA();
  if (dB == 'B') motors.backwardB(); else motors.forwardB();
#else
  motorA.setSpeed(255); motorB.setSpeed(255);
  if (dA == 'B') motorA.backward(); else motorA.forward();
  if (dB == 'B') motorB.backward(); else motorB.forward();
#endif
  unsigned long t0 = millis();
  while ((int)(millis() - t0) < ms) { delay(2); }
  motor_stop();
}

void motor_pwm_test(int pA, int pB, int ms){
  ms = constrain(ms, 1, 5000);

  int a = clamp255(pA);
  int b = clamp255(pB);

  noInterrupts();
  gLastPwmA = a; gLastPwmB = b;
  interrupts();

#ifdef USE_L298NX2
  motors.setSpeedA(a);
  motors.setSpeedB(b);
#else
  motorA.setSpeed(a);
  motorB.setSpeed(b);
#endif
  unsigned long t0 = millis();
  while ((int)(millis() - t0) < ms) { delay(2); }
  motor_stop();
}

void gen_random_command(char& outDirA,int& outPwmA,char& outDirB,int& outPwmB,int& outAngle){
  outDirA=(random(0,2)==0)?'F':'B';
  outDirB=(random(0,2)==0)?'F':'B';
  outPwmA=random(80,181);
  outPwmB=random(80,181);
  int jitter=random(-30,31);
  outAngle=constrain(90+jitter,60,120);
}


