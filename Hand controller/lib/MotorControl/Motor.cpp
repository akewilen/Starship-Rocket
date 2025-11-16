
#include "Servo.h"
#include "Arduino.h"
#include "Motor.h"

Servo Motor1;
Servo Motor2;

volatile uint32_t MotorControlPWM = 0;

void Motor_SetSpeed(int8_t speed)
{
  // Constrain speeds to defined limits
  int motor_speed = int(constrain(speed, Speed_Min, Speed_Max));


  // Map speed percentage to servo write values (1050-1940 degrees)
  int motor1_write = map(motor_speed, 0, 100, 1100, 1940);
  int motor2_write = map(motor_speed, 0, 100, 1100, 1940);
  
  Motor1.write(motor1_write);
  Motor2.write(motor2_write);
}

void Motor_Init()
{
  Motor1.attach(MOTOR1_PIN);
  Motor2.attach(MOTOR2_PIN);
  
  Motor_SetSpeed(0);
}

void MotorCalibration() {
    Motor_SetSpeed(100);
    delay(5000); // Run motors at full speed for 5 seconds
    Motor_SetSpeed(0);
    delay(2000); // Wait for 2 seconds
}