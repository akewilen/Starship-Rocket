
#ifndef MOTOR_H
#define MOTOR_H

#include <Servo.h>
#include <Arduino.h>

#define MOTOR1_PIN 7 //Upper motor
#define MOTOR2_PIN 8 //Lower motor

#define Speed_Min 0
#define Speed_Max 100

void Motor_SetSpeed(int8_t speed);
void Motor_Init();
void MotorCalibration();

#endif