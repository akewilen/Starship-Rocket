
#include "ServoControl.h"
#include "Arduino.h"
#include "Servo.h"

Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;

volatile uint32_t Servo1ControlPWM = 0;
volatile uint32_t Servo2ControlPWM = 0;
volatile uint32_t Servo3ControlPWM = 0;
volatile uint32_t Servo4ControlPWM = 0;

void ServoControl_SetAngle(int8_t angle1, int8_t angle2, int8_t angle3, int8_t angle4)
{
    int theta1 = angle1 + SERVO1_HOME_DEG;
    int theta2 = angle2 + SERVO2_HOME_DEG;
    int theta3 = angle3 + SERVO3_HOME_DEG;
    int theta4 = angle4 + SERVO4_HOME_DEG;

    // Constrain angles to defined limits
    theta1 = int(constrain(theta1, SERVO1_HOME_DEG + SERVO_MIN_ANGLE, SERVO1_HOME_DEG + SERVO_MAX_ANGLE));
    theta2 = int(constrain(theta2, SERVO2_HOME_DEG + SERVO_MIN_ANGLE, SERVO2_HOME_DEG + SERVO_MAX_ANGLE));
    theta3 = int(constrain(theta3, SERVO3_HOME_DEG + SERVO_MIN_ANGLE, SERVO3_HOME_DEG + SERVO_MAX_ANGLE));
    theta4 = int(constrain(theta4, SERVO4_HOME_DEG + SERVO_MIN_ANGLE, SERVO4_HOME_DEG + SERVO_MAX_ANGLE));

    // Map angles to servo write values (900-2100 degrees)
    int servo1_write = map(theta1, 0, 120, 900, 2100);
    int servo2_write = map(theta2, 0, 120, 900, 2100);
    int servo3_write = map(theta3, 0, 120, 900, 2100);
    int servo4_write = map(theta4, 0, 120, 900, 2100);
    
    Servo1.write(servo1_write);
    Servo2.write(servo2_write);
    Servo3.write(servo3_write);
    Servo4.write(servo4_write);
}

void ServoControl_Init()
{
    Servo1.attach(SERVO1_PIN);
    Servo2.attach(SERVO2_PIN);
    Servo3.attach(SERVO3_PIN);
    Servo4.attach(SERVO4_PIN);
    
    ServoControl_SetAngle(0, 0, 0, 0);
}

void ServoControlTest() {
    // Sweep servo 1 and 3 from -30 to 30
    for (int angle = -60; angle <= 60; angle++) {
        ServoControl_SetAngle(angle, 0, -angle, 0);
        delay(20);
    }
    ServoControl_SetAngle(0, 0, 0, 0);

    // Sweep servo 2 and 4 from -30 to 30
    for (int angle = -60; angle <= 60; angle++) {
        ServoControl_SetAngle(0, angle, 0, -angle);
        delay(20);
    }
    ServoControl_SetAngle(0, 0, 0, 0);

    for (int angle = 60; angle >= -60; angle--) {
        ServoControl_SetAngle(angle, angle, angle, angle);
        delay(20);
    }
    ServoControl_SetAngle(0, 0, 0, 0);
}

void MapMomentsToServoAngles(double Mx, double My, double Mz, uint8_t throttle) {
    int roll_rudder, pitch_rudder, yaw_rudder;

    // Throttle compensation factor 
    double constrainedThrottle = constrain(static_cast<double>(throttle), 0.6*255.0, 255.0);
    double throttleCompensation = map(constrainedThrottle, 0.6*255.0, 255.0, 1.0653, 0.8039f);

    // Map moments to servo angles with throttle compensation
    roll_rudder = MX_MY_SLOPE*(int)Mx*throttleCompensation;
    pitch_rudder = MX_MY_SLOPE*(int)My*throttleCompensation;
    yaw_rudder = MX_MY_SLOPE*0.5f*(int)Mz*throttleCompensation;

    // Set servo angles with Saturation
    ServoControl_SetAngle(roll_rudder+yaw_rudder, pitch_rudder+yaw_rudder, 
                        -roll_rudder+yaw_rudder, -pitch_rudder+yaw_rudder);
}
