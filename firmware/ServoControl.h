#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO_PIN 18

// Direct steering range for the installed MG90S with the current linkage.
//
// Measured physical travel was about 55..95..130. Start with a 5-degree
// safety margin at both ends while tuning the controller for this servo.
// These are now the real command values used by Bug2/WF/pure pursuit, not an
// old-servo logical range mapped to a separate physical range.
#define SERVO_MIN 60
#define SERVO_CENTER 95
#define SERVO_MAX 125

// Calibration-only physical endpoints. Normal Bug2/WF/Nav control should stay
// inside SERVO_MIN..SERVO_MAX until curvature tests justify widening it.
#define SERVO_CAL_MIN 55
#define SERVO_CAL_MAX 130

void servo_setup(int pin);
void servo_set_angle(int deg);
void servo_set_angle_calibration(int deg);
int servo_shape_autonomous_angle(int deg);
bool servo_attach_to_pin(int pin);
void servo_write_us(int us);
bool servo_is_attached();

// Servo stall detection
bool servo_is_stalled();
void servo_reset_stall();
