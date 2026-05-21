#include "ServoControl.h"
#include "DbgLog.h"

static Servo myServo;
static uint32_t lastLog = 0;

// Servo stall detection: track if servo position isn't changing
static int lastCommandedAngle = SERVO_CENTER;
static uint32_t lastAngleChangeMs = 0;
static uint8_t stallCounter = 0;  // increments if servo doesn't respond
static const uint32_t SERVO_STALL_TIMEOUT_MS = 800;  // 800ms of no change = stall
static const uint8_t SERVO_STALL_THRESHOLD = 3;  // 3 consecutive stalls
static const int SERVO_ATTACH_MIN_US = 500;
static const int SERVO_ATTACH_MAX_US = 2500;

static int servo_angle_to_us(int deg){
  deg = constrain(deg, 0, 180);
  return map(deg, 0, 180, SERVO_ATTACH_MIN_US, SERVO_ATTACH_MAX_US);
}

void servo_write_us(int us){
  myServo.writeMicroseconds(us);
}

bool servo_attach_to_pin(int pin){
  myServo.detach();

  // Reserve ONLY timer 3 for the servo (leave 0–2 for motors)
  ESP32PWM::allocateTimer(3);

  myServo.setPeriodHertz(50);
  bool ok = myServo.attach(pin, SERVO_ATTACH_MIN_US, SERVO_ATTACH_MAX_US);
  if(ok){
    Serial.printf("[SERVO] attach OK on GPIO %d (timer3, 500–2500us)\n", pin);
    servo_write_us(servo_angle_to_us(SERVO_CENTER));
  }else{
    Serial.printf("[SERVO] attach **FAILED** on GPIO %d\n", pin);
  }
  return ok;
}

bool servo_is_attached(){ return myServo.attached(); }

void servo_setup(int pin){
  bool ok = servo_attach_to_pin(pin);
  if(ok) myServo.writeMicroseconds(servo_angle_to_us(SERVO_CENTER));
}

int servo_shape_autonomous_angle(int deg){
  deg = constrain(deg, SERVO_MIN, SERVO_MAX);

  // MG90S curvature tests showed moderate right commands are physically sharper
  // than matching left commands. Keep raw manual/calibration commands direct, but
  // compress autonomous mid-right steering so old controller magnitudes remain
  // closer to the observed left/right curvature.
  const int err = deg - SERVO_CENTER;
  if (err <= 0) return deg;

  int shapedErr = err;
  if (err <= 10) {
    shapedErr = (err * 6 + 5) / 10;                 // +10 -> +6
  } else if (err <= 20) {
    shapedErr = 6 + (((err - 10) * 11 + 5) / 10);   // +20 -> +17
  } else {
    shapedErr = 17 + (((err - 20) * 13 + 5) / 10);  // +30 -> +30
  }

  return constrain(SERVO_CENTER + shapedErr, SERVO_MIN, SERVO_MAX);
}

static void servo_set_angle_clamped(int deg, int minDeg, int maxDeg, const char* tag){
  deg = constrain(deg, minDeg, maxDeg);

  // STALL DETECTION: Check if servo is responding
  uint32_t now = millis();
  if (deg != lastCommandedAngle) {
    lastAngleChangeMs = now;
    stallCounter = 0;  // Reset stall counter on new command
    lastCommandedAngle = deg;
  } else {
    // Same angle commanded again - check if servo has been stuck
    if (now - lastAngleChangeMs > SERVO_STALL_TIMEOUT_MS) {
      stallCounter++;
      // Log stall event
      if (stallCounter == SERVO_STALL_THRESHOLD) {
        Serial.printf("[SERVO] **STALL DETECTED** at deg=%d after %lu ms\n", 
                      deg, now - lastAngleChangeMs);
        dbg_push(EV_SERVO_STALL, deg, (int32_t)(now - lastAngleChangeMs), stallCounter);
      }
    }
  }

  // Direct MG90S command. Bug2/WF now tune against this servo's real range.
  const int us = servo_angle_to_us(deg);
  myServo.writeMicroseconds(us);

  uint32_t log_now = millis();
  if(log_now - lastLog > 500){
    lastLog = log_now;
    Serial.printf("[SERVO] %s angle=%d => %dus (stall=%u)\n",
                  tag, deg, us, stallCounter);
  }
}

void servo_set_angle(int deg){
  servo_set_angle_clamped(deg, SERVO_MIN, SERVO_MAX, "ctrl");
}

void servo_set_angle_calibration(int deg){
  servo_set_angle_clamped(deg, SERVO_CAL_MIN, SERVO_CAL_MAX, "cal");
}

// Query stall status
bool servo_is_stalled() {
  return stallCounter >= SERVO_STALL_THRESHOLD;
}

// Reset stall counter (call after recovery)
void servo_reset_stall() {
  stallCounter = 0;
  lastAngleChangeMs = millis();
}
