
#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include <Servo.h>
#include <Arduino.h>

extern volatile uint32_t Servo1ControlPWM;
extern volatile uint32_t Servo2ControlPWM;
extern volatile uint32_t Servo3ControlPWM;
extern volatile uint32_t Servo4ControlPWM;

#define SERVO1_PIN 2
#define SERVO2_PIN 14
#define SERVO3_PIN 3
#define SERVO4_PIN 22

#define SERVO1_HOME_DEG 65
#define SERVO2_HOME_DEG 63
#define SERVO3_HOME_DEG 54
#define SERVO4_HOME_DEG 64

#define SERVO_MAX_ANGLE 60
#define SERVO_MIN_ANGLE -60

void ServoControl_SetAngle(int8_t angle1, int8_t angle2, int8_t angle3, int8_t angle4);
void ServoControl_Init();
void ServoControlTest();

#endif