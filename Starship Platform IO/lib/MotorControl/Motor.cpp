
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

  // Map speed percentage to servo write values (1100-1940 degrees)
  float motor1_write = map((float)motor_speed, 0, 100, 1100, 1940);
  float motor2_write = map((float)motor_speed, 0, 100, 1100, 1940);

  float dutyCycle1 = motor1_write / 20000.0f; // 20ms period for 50Hz
  float dutyCycle2 = motor2_write / 20000.0f; //
  
  noInterrupts();
  analogWrite(MOTOR1_PIN, uint32_t(dutyCycle1 * 4095)); // 12-bit resolution
  analogWrite(MOTOR2_PIN, uint32_t(dutyCycle2 * 4095)); // 12-bit resolution
  interrupts();

  //Motor1.write(motor1_write);
  //Motor2.write(motor2_write);
}

void Motor_Init()
{
  analogWriteResolution(12); // Set PWM resolution to 12 bits (0-4095)
  analogWriteFrequency(MOTOR1_PIN, 50); // Set PWM frequency to 50 Hz
  analogWriteFrequency(MOTOR2_PIN, 50); // Set PWM frequency to 50 Hz

  //Motor1.attach(MOTOR1_PIN);
  //delay(10); // Short delay to ensure proper initialization
  //Motor2.attach(MOTOR2_PIN);
  //delay(10); // Short delay to ensure proper initialization
  
  Motor_SetSpeed(0);
}

void MotorCalibration() {
    Motor_SetSpeed(100);
    delay(5000); // Run motors at full speed for 5 seconds
    Motor_SetSpeed(0);
    delay(2000); // Wait for 2 seconds
}