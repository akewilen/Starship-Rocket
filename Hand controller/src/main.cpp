#include <Arduino.h>
#include <ServoControl.h>
#include <Motor.h>
#include <Servo.h>

#define SERIAL_BAUD_RATE 9600

// Function to read serial input and set servo angles and motor speed
void processSerialControlInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove whitespace
    
    // Expected format: "angle1,angle2,angle3,angle4,motorSpeed" or "angle1 angle2 angle3 angle4 motorSpeed"
    int values[5] = {0, 0, 0, 0, 0}; // 4 servo angles + 1 motor speed
    int valueCount = 0;
    
    // Parse comma-separated or space-separated values
    unsigned int startIndex = 0;
    unsigned int inputLen = input.length();
    for (unsigned int i = 0; i <= inputLen && valueCount < 5; i++) {
      if (i == inputLen || input.charAt(i) == ',' || input.charAt(i) == ' ') {
        if (i > startIndex) {
          String valueStr = input.substring(startIndex, i);
          valueStr.trim();
          if (valueStr.length() > 0) {
            values[valueCount] = valueStr.toInt();
            valueCount++;
          }
        }
        startIndex = i + 1;
      }
    }
    
    if (valueCount >= 5) {
      // Constrain servo angles to valid range (-30 to 30)
      for (int i = 0; i < 4; i++) {
        values[i] = constrain(values[i], SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
      }
      
      // Constrain motor speed to valid range (0 to 100)
      values[4] = constrain(values[4], Speed_Min, Speed_Max);
      
      // Set servo angles
      ServoControl_SetAngle(values[0], values[1], values[2], values[3]);
      
      // Set motor speed
      Motor_SetSpeed(values[4]);
      
      // Send confirmation
      Serial.print("Servos set to: ");
      Serial.print(values[0]); Serial.print(", ");
      Serial.print(values[1]); Serial.print(", ");
      Serial.print(values[2]); Serial.print(", ");
      Serial.print(values[3]);
      Serial.print(" | Motor speed: ");
      Serial.println(values[4]);
    } else if (valueCount >= 4) {
      // Backward compatibility: if only 4 values provided, set servos only
      for (int i = 0; i < 4; i++) {
        values[i] = constrain(values[i], SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
      }
      
      ServoControl_SetAngle(values[0], values[1], values[2], values[3]);
      
      Serial.print("Servos set to: ");
      Serial.print(values[0]); Serial.print(", ");
      Serial.print(values[1]); Serial.print(", ");
      Serial.print(values[2]); Serial.print(", ");
      Serial.println(values[3]);
    } else {
      Serial.println("Error: Please send 4 or 5 values. Format:");
      Serial.println("  'angle1,angle2,angle3,angle4' (servos only)");
      Serial.println("  'angle1,angle2,angle3,angle4,motorSpeed' (servos + motor)");
      Serial.println("Valid ranges: Servos -30 to 30 degrees, Motor 0 to 100%");
    }
  }
}

void setup() {
  //initialize serial communication
  Serial.println("Initializing Serial Communication...");
  Serial.begin(SERIAL_BAUD_RATE);

  // Initialize Servo Control and Motor
  Serial.println("Initializing Servo ");
  ServoControl_Init();

  Serial.println("Initializing Motor...");
  Motor_Init();
  MotorCalibration();

  // Run Servo Control Test
  Serial.println("Running Servo Control Test...");
  ServoControlTest();
 
}

void loop() {
  // Continuously check for serial input and update servos and motor
  processSerialControlInput();
  
  // Small delay to prevent overwhelming the serial buffer
  delay(50);
}

