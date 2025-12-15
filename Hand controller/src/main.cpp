#include <Arduino.h>
#include <ServoControl.h>
#include <Motor.h>
#include <Servo.h>
#include <HandController.h>
#include <SDCardLoggin.h>
#include <IMU.h>
#include <Wire.h>
#include <MadgwickAHRS.h>
#include <Kgain.h>
#define SERIAL_BAUD_RATE 9600

uint8_t throttle;
uint8_t RefPitch;
uint8_t RefYaw;  
uint8_t RefRoll;
uint8_t killSwitchValue;

// IMU raw values (int16_t from sensors)
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;
int16_t mx_raw, my_raw, mz_raw;

// IMU scaled values for display
float ax_ms2, ay_ms2, az_ms2;
float gx_rps, gy_rps, gz_rps;

Eigen::Vector3d Acc_Bias(0.0, 0.0, 0.0); // Ax, Ay, Az
Eigen::Vector3d Gyro_Bias(0.0, 0.0, 0.0); // Gx, Gy, Gz
Eigen::Vector3d Mag_Bias(0.0, 0.0, 0.0); // Mx, My, Mz

double roll; double pitch; double yaw;
double ref_roll = 0; double ref_pitch = 0; double ref_yawRate = 0;

// Global variables for system state
volatile bool emergencyStop = false;
HandController handController;

// Debug and logging variables
unsigned long lastDebugTime = 0;
unsigned long lastLogTime = 0;
unsigned long lastControlTime = 0; // For 100Hz control loop
unsigned long loopStartTime = 0;  // Timestamp zero point
const unsigned long DEBUG_INTERVAL = 100; // Print debug info every 100ms
const unsigned long LOG_INTERVAL = 10;    // Log data every 10ms
const unsigned long CONTROL_INTERVAL = 10; // Control loop every 10ms (100 Hz)

Eigen::Matrix4d P0 = Eigen::Matrix4d::Identity();
Eigen::Matrix3d Rw, Rw_unscaled;
Eigen::Matrix3d Ra, Ra_unscaled;
Eigen::Vector3d f = Eigen::Vector3d(0.0, 0.0, 0.0);
Eigen::Vector4d x0 = Eigen::Vector4d(1, 0, 0, 0);

Madgwick filter;

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
      
      ref_roll = values[0];
      ref_pitch = values[1];
      ref_yawRate = values[2];
      throttle = values[3];
      //ServoControl_SetAngle(values[0], values[1], values[2], values[3]);
      
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
  Serial.print(" | Pitch: "); Serial.print(RefPitch); Serial.print(" (0-255)");
  Serial.print(" | Yaw: "); Serial.print(RefYaw); Serial.print(" (0-255)");
  Serial.print(" | Roll: "); Serial.print(RefRoll); Serial.print(" (0-255)");
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
  handController.attach(throttle, RefPitch, RefYaw, RefRoll, killSwitchValue);
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
  throttle = 0;

  // Run Servo Control Test
  Serial.println("Running Servo Control Test...");
  ServoControlTest();

  // Initialize IMU
  Serial.println("Initializing IMU...");
  IMU_Init();
  Acc_Bias = IMU_ACC_BIAS_READ();
  Gyro_Bias = IMU_GYRO_BIAS_READ();
  Mag_Bias = IMU_MAG_BIAS_READ();

  Serial.println("IMU initialized");

  Ra_unscaled << 0.2596, 0.0135, 0.0025,
    0.0135, 0.2925, -0.0086,
    0.0025, -0.0086, 0.1750;
  Ra = Ra_unscaled * 1e-3;

  Rw_unscaled << 0.1695,    0.0121 ,   0.0024,
               0.0121,    0.1928 ,   0.0070,
               0.0024,    0.0070 ,   0.1443;
  Rw = Rw_unscaled * 1e-3;
 
}

void loop() {
  // Set timestamp zero point on first loop iteration
  if (loopStartTime == 0) {
    loopStartTime = millis();
    if (sdLogger.isReady()) {
      sdLogger.logMessage("Flight started - Loop timestamp zero point set");
    }
  }

  // 100Hz control loop - only run if 10ms has passed
  unsigned long currentTime = millis();
  if (currentTime - lastControlTime >= CONTROL_INTERVAL) {
    lastControlTime = currentTime;

    // Check killswitch - if under center (127), activate emergency stop
    if (killSwitchValue < 127) {
      emergencyStop = true;
    } else {
      emergencyStop = false;
    }

    // --------- Read IMU & get attitude ---------
    IMU_Read(ax_ms2, ay_ms2, az_ms2, gx_rps, gy_rps, gz_rps, mx_raw, my_raw, mz_raw);

    Eigen::Vector3d acc_input(ax_ms2 - Acc_Bias[0],
                              ay_ms2 - Acc_Bias[1],
                              az_ms2 - Acc_Bias[2]);

    Eigen::Vector3d gyro_input(gx_rps - Gyro_Bias[0],
                               gy_rps - Gyro_Bias[1],
                               gz_rps - Gyro_Bias[2]);

    filter.updateIMU(gyro_input[0] * 180 / M_PI, gyro_input[1] * 180 / M_PI, gyro_input[2] * 180 / M_PI,
                      acc_input[0], acc_input[1], acc_input[2]);

    // Get raw angles from filter
    double roll_raw = filter.getRoll();
    double pitch_raw = filter.getPitch();
    double yaw_raw = 180.0 - filter.getYaw();
    
    // Apply lowpass filter to roll, pitch, yaw
    static double roll_filtered = 0.0;
    static double pitch_filtered = 0.0;
    static double yaw_filtered = 0.0;
    static bool first_run = true;
    
    if (first_run) {
        // Initialize filters with first values
        roll_filtered = roll_raw;
        pitch_filtered = pitch_raw;
        yaw_filtered = yaw_raw;
        first_run = false;
    }
    
    // Lowpass filter coefficient (0 < alpha < 1, smaller = more filtering)
    const double alpha = 0.1;
    roll_filtered = alpha * roll_raw + (1.0 - alpha) * roll_filtered;
    pitch_filtered = alpha * pitch_raw + (1.0 - alpha) * pitch_filtered;
    yaw_filtered = alpha * yaw_raw + (1.0 - alpha) * yaw_filtered;
    
    // Update global variables with filtered values
    roll = roll_filtered;
    pitch = pitch_filtered;
    yaw = yaw_filtered;

    // ----------- Calculate K gain ----------------
    double roll_err = (roll - ref_roll)*M_PI/180.0;
    double pitch_err = (pitch - ref_pitch)*M_PI/180.0;
    double yawRate_err = (gyro_input[2] - ref_yawRate)*M_PI/180.0;

    //State feedback control vector State vector [p q r phi_err theta_err psi]'
    Eigen::Vector3d U_K = U_K_att(gyro_input[0], gyro_input[1], yawRate_err, roll_err, pitch_err, yaw);

    // ----------- Set servo and thrust ------------
    MapMomentsToServoAngles((double)U_K(0), (double)U_K(1), (double)U_K(2), throttle);

    Motor_SetSpeed(throttle);

  }

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
  
//Emergency stop loop
 while (emergencyStop) {
   // Set everything to safe state once
   //ServoControl_SetAngle(0, 0, 0, 0);
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
 
 // Log flight data continuously
 if (sdLogger.isReady() && (currentTime - lastLogTime >= LOG_INTERVAL)) {
   // Log with timestamp relative to loop start (flight time)
   sdLogger.logDataWithCustomTime(currentTime - loopStartTime, throttle, roll, pitch, yaw, killSwitchValue, emergencyStop, ax_ms2, ay_ms2, az_ms2, gx_rps, gy_rps, gz_rps);
   lastLogTime = currentTime;
 }
 
 // Print debug info periodically
 if (currentTime - lastDebugTime >= DEBUG_INTERVAL) {
   int flightTime = (currentTime - loopStartTime);
   //Serial.print(String(throttle) + " " + String(roll,3) + " " + String(pitch,3) + " " + String(yaw,3) + "\n");
   //Serial.println("T:" + String(flightTime, 1) + "s | IMU[AX:" + String(ax_ms2, 2) + " AY:" + String(ay_ms2, 2) + " AZ:" + String(az_ms2, 2) +
   //               " GX:" + String(gx_dps, 2) + " GY:" + String(gy_dps, 2) + " GZ:" + String(gz_dps, 2) +
   //               " MX:" + String(mx, 1) + " MY:" + String(my, 1) + " MZ:" + String(mz, 1) + "]");
   lastDebugTime = currentTime;
 }
}
