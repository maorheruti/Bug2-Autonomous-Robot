#pragma once
#include <Arduino.h>

// Uncomment to trace every motor_stop() call with file/line
//#define MOTOR_STOP_TRACE

// #define USE_L298NX2

#ifdef USE_L298NX2
  #include <L298NX2.h>
#else
  #include <L298N.h>
#endif

#define IN1 27
#define IN2 26
#define EN_A 14
#define IN3 25
#define IN4 33
#define EN_B 12

void motor_init();
void motor_apply(char dA,int pA,char dB,int pB);

// Internal stop impl (do not call directly)
void motor_stop_impl();

// Debug stop wrapper
void motor_stop_dbg(const char* file, int line);

#ifdef MOTOR_STOP_TRACE
  // Any call to motor_stop() will print who called it
  #define motor_stop() motor_stop_dbg(__FILE__, __LINE__)
#else
  #define motor_stop() motor_stop_impl()
#endif

void motor_full_digital(char dA, char dB, int ms);
void motor_pwm_test(int pA, int pB, int ms);

void gen_random_command(char& outDirA,int& outPwmA,char& outDirB,int& outPwmB,int& outAngle);

// Telemetry for dashboard
void motor_get_last(char& dA, int& pA, char& dB, int& pB);


