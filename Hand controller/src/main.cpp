#include <Arduino.h>
#include <ServoControl.h>
#include <Motor.h>
#include <Servo.h>
#include <HandController.h>
#include <SDCardLoggin.h>
#include <IMU.h>
#include <Wire.h>

#define SERIAL_BAUD_RATE 9600

uint8_t throttle;
uint8_t pithch;
uint8_t yaw;  
uint8_t roll;
uint8_t killSwitchValue;

// IMU raw values (int16_t from sensors)
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;
int16_t mx_raw, my_raw, mz_raw;

// IMU scaled values for display
float ax_ms2, ay_ms2, az_ms2;
float gx_dps, gy_dps, gz_dps;
float mx, my, mz;

// Global variables for system state
volatile bool emergencyStop = false;
HandController handController;

// Debug and logging variables
unsigned long lastDebugTime = 0;
unsigned long lastLogTime = 0;
unsigned long loopStartTime = 0;  // Timestamp zero point
const unsigned long DEBUG_INTERVAL = 1000; // Print debug info every 1 second
const unsigned long LOG_INTERVAL = 100;    // Log data every 100ms (10 Hz)

// Function to process servo/motor commands from string input
void processSerialCommand(String input) {
  // Only process commands if system is not in emergency stop
  if (emergencyStop) {
    return;
  }
    
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

// Debug function to print PPM values and SD card status
void printPPMDebug() {
  Serial.println("=== System Debug Info ===");
  Serial.print("Throttle: "); Serial.print(throttle); Serial.print(" (0-255)");
  Serial.print(" | Pitch: "); Serial.print(pithch); Serial.print(" (0-255)");
  Serial.print(" | Yaw: "); Serial.print(yaw); Serial.print(" (0-255)");
  Serial.print(" | Roll: "); Serial.print(roll); Serial.print(" (0-255)");
  Serial.print(" | KillSwitch: "); Serial.print(killSwitchValue); Serial.println(" (0-255)");
  Serial.println("SD Card Status: " + String(sdLogger.isReady() ? "READY" : "NOT READY"));
  if (sdLogger.isReady()) {
    Serial.println("Log File: " + sdLogger.getCurrentLogFile());
    Serial.println("Flight Time: " + String((millis() - loopStartTime) / 1000.0, 1) + " seconds");
  }
  Serial.println("Note: 127 = center, 0 = min, 255 = max. KillSwitch <127 = EMERGENCY STOP");
  Serial.println("Commands: debug, status");
  Serial.println("=========================");
}



void setup() {
  //initialize serial communication
  Serial.println("Initializing Serial Communication...");
  Serial.begin(SERIAL_BAUD_RATE);
  
  // Initialize Hand Controller first
  Serial.println("Initializing Hand Controller...");
  handController.attach(throttle, pithch, yaw, roll, killSwitchValue);
  handController.init();
  Serial.println("Hand controller initialized with killswitch on pin 0");

  // Initialize SD Card Logging
  Serial.println("Initializing SD Card Logging...");
  if (sdLogger.init()) {
    sdLogger.logMessage("System startup - Hand Controller initialized");
    Serial.println("SD card logging ready - logging flight data continuously");
  } else {
    Serial.println("SD card logging failed - continuing without logging");
  }

  // Initialize Servo Control and Motor
  Serial.println("Initializing Servo ");
  ServoControl_Init();

  Serial.println("Initializing Motor...");
  Motor_Init();
  MotorCalibration();

  // Run Servo Control Test
  Serial.println("Running Servo Control Test...");
  ServoControlTest();

  // Initialize IMU
  Serial.println("Initializing IMU...");
  IMU_Init();
  Serial.println("IMU initialized");
 
}

void loop() {
  // Set timestamp zero point on first loop iteration
  if (loopStartTime == 0) {
    loopStartTime = millis();
    if (sdLogger.isReady()) {
      sdLogger.logMessage("Flight started - Loop timestamp zero point set");
    }
  }
  
  // Read all inputs including killswitch
  handController.readInputs();
  
  // Read IMU data (raw int16_t values)
  IMU_Read(ax_ms2, ay_ms2, az_ms2, gx_dps, gy_dps, gz_dps, mx_raw, my_raw, mz_raw);
  
  // Magnetometer values - convert to float for consistency
  mx = (float)mx_raw;
  my = (float)my_raw;
  mz = (float)mz_raw;
  
  // Check killswitch - if under center (127), activate emergency stop
  if (killSwitchValue < 127) {
    emergencyStop = true;
  } else {
    emergencyStop = false;
  }
  
  // Emergency stop loop
  while (emergencyStop) {
    // Set everything to safe state once
    ServoControl_SetAngle(0, 0, 0, 0);
    Motor_SetSpeed(0);
    Serial.println("*** EMERGENCY STOP ACTIVE - KillSwitch < 127 ***");
    
    // Wait and recheck killswitch
    delay(500);
    handController.readInputs();
    if (killSwitchValue >= 127) {
      emergencyStop = false;
      Serial.println("Emergency stop cleared - System operational");
      break;
    }
  }
  
  // Normal operation - process commands
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input == "debug") {
      printPPMDebug();
    } else if (input == "status") {
      Serial.println("SYSTEM OPERATIONAL");
      printPPMDebug();
    } else {
      processSerialCommand(input);
    }
  }
  
  // Log flight data continuously
  unsigned long currentTime = millis();
  if (sdLogger.isReady() && (currentTime - lastLogTime >= LOG_INTERVAL)) {
    // Log with timestamp relative to loop start (flight time)
    sdLogger.logDataWithCustomTime(currentTime - loopStartTime, throttle, pithch, yaw, roll, killSwitchValue, emergencyStop, ax_ms2, ay_ms2, az_ms2, gx_dps, gy_dps, gz_dps);
    lastLogTime = currentTime;
  }
  
  // Print debug info periodically
  if (currentTime - lastDebugTime >= DEBUG_INTERVAL) {
    float flightTime = (currentTime - loopStartTime) / 1000.0;
    //Serial.println("T:" + String(flightTime, 1) + "s PPM[T:" + String(throttle) + " R:" + String(roll) + " P:" + String(pithch) + " Y:" + String(yaw) + " K:" + String(killSwitchValue));
    Serial.println("T:" + String(flightTime, 1) + "s | IMU[AX:" + String(ax_ms2, 2) + " AY:" + String(ay_ms2, 2) + " AZ:" + String(az_ms2, 2) +
                   " GX:" + String(gx_dps, 2) + " GY:" + String(gy_dps, 2) + " GZ:" + String(gz_dps, 2) +
                   " MX:" + String(mx, 1) + " MY:" + String(my, 1) + " MZ:" + String(mz, 1) + "]");
    lastDebugTime = currentTime;
  }

  
  
  delay(10);
}

