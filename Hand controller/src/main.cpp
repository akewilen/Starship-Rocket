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
#include <PID.h>

#define SERIAL_BAUD_RATE 115200


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

double roll, pitch, yaw;
double roll_err_int, pitch_err_int, yawRate_err_int;

// Previous error and anti-windup terms for trapezoidal integration
double prev_roll_err = 0.0, prev_pitch_err = 0.0, prev_yawRate_err = 0.0;
// Previous yaw for derivative calculation
static double prev_yaw = 0.0;
static bool yaw_init = true;
Eigen::Vector3d prev_aw_term(0.0, 0.0, 0.0);

uint8_t ref_throttle = 0;
double ref_roll = 0; double ref_pitch = 0; double ref_yawRate = 0;

HandController handController;

// Global variables for system state
volatile bool emergencyStop = false;
volatile bool newDataReady = false;
unsigned long currentTime = 0;
unsigned long lastDebugTime = 0;
unsigned long lastLogTime = 0;
unsigned long lastControlTime = 0; // For 100Hz control loop
unsigned long loopStartTime = 0;  // Timestamp zero point
const unsigned long DEBUG_INTERVAL = 250; // Print debug info every 100ms
const unsigned long LOG_INTERVAL = 10;    // Log data every 10ms
const unsigned long CONTROL_INTERVAL = 10; // Control loop every 10ms (100 Hz)
volatile double dt_s = 0.010; // Control loop time step in seconds

// IMU lowpass filter coefficients (0 < alpha < 1, smaller = more filtering)
const double alpha_gyro = 0.4;  // Less aggressive filtering for 100Hz
const double alpha_acc = 0.2;   // Moderate filtering for 100Hz

Eigen::Matrix4d P0 = Eigen::Matrix4d::Identity();
Eigen::Matrix3d Rw, Rw_unscaled;
Eigen::Matrix3d Ra, Ra_unscaled;
Eigen::Vector3d f = Eigen::Vector3d(0.0, 0.0, 0.0);
Eigen::Vector4d x0 = Eigen::Vector4d(1, 0, 0, 0);

Madgwick filter;

IntervalTimer controlTimer;

PID rollPID(0.0, 0.0, 0.0, -45, 45); // Start with zero gains for tuning
PID pitchPID(0.0, 0.0, 0.0, -45, 45); // Start with zero gains for tuning
PID yawRatePID(0.0, 0.0, 0.0, -10, 10); // Start with zero gains for tuning

// Current PID gains for roll/pitch (shared)
double current_P = 0.0;
double current_I = 0.0;
double current_D = 0.0;

void controlLoopISR() {
    // ISR overrun detection
    static volatile bool isrRunning = false;
    static unsigned long isrStartTime = 0;
    unsigned long currentTime = micros();
    
    // Check if previous ISR is still running (overrun condition)
    if (isrRunning) {
        Serial.println("*** CRITICAL: ISR OVERRUN! Previous ISR still executing! ***");
        return; // Exit immediately to prevent stack overflow
    }
    
    // Mark ISR as running and record start time
    isrRunning = true;
    isrStartTime = currentTime;
    
    // emergencyStop is now set directly by HandController based on killswitch value

  if(!emergencyStop){
  // --------- Read IMU & get attitude ---------
    IMU_Read(ax_ms2, ay_ms2, az_ms2, gx_rps, gy_rps, gz_rps, mx_raw, my_raw, mz_raw);
    Eigen::Vector3d acc_input(ax_ms2 - Acc_Bias[0],
                              ay_ms2 - Acc_Bias[1],
                              az_ms2 - Acc_Bias[2]);
    Eigen::Vector3d gyro_input(gx_rps - Gyro_Bias[0],
                               gy_rps - Gyro_Bias[1],
                               gz_rps - Gyro_Bias[2]);

    // Static variables for lowpass filtering IMU data
    static Eigen::Vector3d acc_filtered(0.0, 0.0, 9.82);
    static Eigen::Vector3d gyro_filtered(0.0, 0.0, 0.0);
    static bool imu_filter_init = true;
    
    if (imu_filter_init) {
        acc_filtered = acc_input;
        gyro_filtered = gyro_input;
        imu_filter_init = false;
    }
    
    // Apply separate lowpass filtering for gyro and accelerometer
    acc_filtered = alpha_acc * acc_input + (1.0 - alpha_acc) * acc_filtered;
    gyro_filtered = alpha_gyro * gyro_input + (1.0 - alpha_gyro) * gyro_filtered;

    filter.updateIMU(gyro_filtered[0] * 180 / M_PI, gyro_filtered[1] * 180 / M_PI, gyro_filtered[2] * 180 / M_PI,
                      acc_filtered[0], acc_filtered[1], acc_filtered[2]);
    // Get angles directly from filter (no additional filtering)

    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = 180.0 - filter.getYaw();
    
    // Calculate yaw rate from derivative of filtered yaw (less noisy than gyro)
    double yawRate;
    if (yaw_init) {
        prev_yaw = yaw;
        yawRate = 0.0;
        yaw_init = false;
    } else {
        yawRate = (yaw - prev_yaw) / dt_s; // deg/s
        prev_yaw = yaw;
    }

    // ----------- Calculate K gain ----------------
    double roll_err = (roll - ref_roll)*M_PI/180.0;
    double pitch_err = (pitch - ref_pitch)*M_PI/180.0;
    double yawRate_err = (yawRate - ref_yawRate)*M_PI/180.0;
    //State feedback control vector State vector [p q r phi_err theta_err psi]'

    double roll_PID = -rollPID.compute(roll_err, dt_s);
    double pitch_PID = -pitchPID.compute(pitch_err, dt_s);  
    double yawRate_PID = -yawRatePID.compute(yawRate_err, dt_s);

    // 1. Calculate unconstrained moment (what LQR wants)
    Eigen::Vector3d U_K = U_K_att(gyro_input[0], gyro_input[1], gyro_input[2], roll_err, pitch_err, 0.0);
    Eigen::Vector3d U_K_I_result = U_K_I(roll_err_int, pitch_err_int, yawRate_err_int);
    Eigen::Vector3d U_K_unconstrained = U_K + U_K_I_result;
    
    // 2. Apply physical limits (Saturation)
    // Moment saturation limits based on ±45 degree rudder limits
    // From ServoControl: rudder = MX_MY_SLOPE * Moment * throttleCompensation * 0.5
    // At worst case throttleCompensation = 0.8039, MX_MY_SLOPE = 24.3309
    const double MOMENT_SAT_LIMIT = 4.6; // ±45 deg / (24.3309 * 0.8039 * 0.5)
    
    Eigen::Vector3d U_K_saturated = U_K_unconstrained;
    U_K_saturated(0) = constrain(U_K_saturated(0), -MOMENT_SAT_LIMIT, MOMENT_SAT_LIMIT);
    U_K_saturated(1) = constrain(U_K_saturated(1), -MOMENT_SAT_LIMIT, MOMENT_SAT_LIMIT);  
    U_K_saturated(2) = constrain(U_K_saturated(2), -MOMENT_SAT_LIMIT, MOMENT_SAT_LIMIT);
    
    // 3. Update the Integrator with Anti-Windup
    Eigen::Vector3d saturation_error = U_K_saturated - U_K_unconstrained;
    Integral_Error_Update_AntiWindup(roll_err, pitch_err, yawRate_err, dt_s, 
                                    roll_err_int, pitch_err_int, yawRate_err_int,
                                    saturation_error, prev_roll_err, prev_pitch_err, prev_yawRate_err, prev_aw_term);

    // 4. Apply lowpass filter to control outputs for actuator smoothing
    static Eigen::Vector3d U_K_filtered(0.0, 0.0, 0.0);
    static bool control_filter_init = true;
    
    if (control_filter_init) {
        U_K_filtered = U_K_saturated;
        control_filter_init = false;
    }
    
    // Lowpass filter coefficient (0 < alpha < 1, smaller = more filtering)
    const double alpha_control = 0.3;  // Less aggressive filtering than orientation
    U_K_filtered = alpha_control * U_K_saturated + (1.0 - alpha_control) * U_K_filtered;

    // ----------- Set servo and thrust ------------
    //MapMomentsToServoAngles((double)U_K_filtered(0), (double)U_K_filtered(1), (double)U_K_filtered(2), ref_throttle);
    
    ServoControl_SetAngle((int8_t)(-pitch_PID), (int8_t)(-roll_PID), (int8_t)(pitch_PID), (int8_t)(roll_PID));

    Motor_SetSpeed(ref_throttle);

    //Print control data for live plotting: ref_angles actual_angles errors integrals unconstrained_moments saturated_moments filtered_moments
    //Serial.print(ref_roll); Serial.print(" ");
    //Serial.print(ref_pitch); Serial.print(" ");
    //Serial.print(ref_yawRate); Serial.print(" ");
    //Serial.print(roll); Serial.print(" ");
    //Serial.print(pitch); Serial.print(" ");
    //Serial.print(yawRate); Serial.print(" ");
    //Serial.print(U_K_I_result(0)); Serial.print(" ");
    //Serial.print(U_K_I_result(1)); Serial.print(" ");
    //Serial.print(U_K_I_result(2)); Serial.print(" ");
    //Serial.print(U_K_filtered(0)); Serial.print(" ");
    //Serial.print(U_K_filtered(1)); Serial.print(" ");
    //Serial.print(U_K_filtered(2)); Serial.print(" ");
    //Serial.print(Mx_PID); Serial.print(" ");
    //Serial.print(My_PID); Serial.print(" ");
    //Serial.print(Mz_PID); Serial.print(" ");
    //Serial.println(ref_throttle);

  } else {
    // In emergency stop, set everything to safe state
    Motor_SetSpeed(0);
  }
  
  // ISR completion detection
  unsigned long isrEndTime = micros();
  unsigned long isrExecutionTime = isrEndTime - isrStartTime;
  
  // Warn if ISR took too long (> 4ms leaves 1ms margin for next cycle)
  if (isrExecutionTime > 4000) {
    Serial.print("*** WARNING: ISR took ");
    Serial.print(isrExecutionTime);
    Serial.println("μs - Risk of overrun! ***");
  }
  
  // Mark ISR as completed
  isrRunning = false;
  
  // Indicate new data is ready for main loop processing
  newDataReady = true;
}

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
      ref_throttle = values[3];
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

// Function to tune PID values via serial commands
void PIDtuner(String input) {
  // Only process commands if system is not in emergency stop
  if (emergencyStop) {
    return;
  }
  
  if (input.length() < 2) {
    Serial.println("Error: Invalid PID command format");
    Serial.println("Format: P100 (sets P gain to 100), I50 (sets I gain to 50), D25 (sets D gain to 25)");
    return;
  }
  
  char command = input.charAt(0);
  String valueStr = input.substring(1);
  double value = valueStr.toFloat();
  
  switch (command) {
    case 'P':
    case 'p':
      current_P = value;
      rollPID.setGains(current_P, current_I, current_D);
      pitchPID.setGains(current_P, current_I, current_D);
      Serial.print("P gain set to: ");
      Serial.println(value);
      break;
      
    case 'I':
    case 'i':
      current_I = value;
      rollPID.setGains(current_P, current_I, current_D);
      pitchPID.setGains(current_P, current_I, current_D);
      Serial.print("I gain set to: ");
      Serial.println(value);
      break;
      
    case 'D':
    case 'd':
      current_D = value;
      rollPID.setGains(current_P, current_I, current_D);
      pitchPID.setGains(current_P, current_I, current_D);
      Serial.print("D gain set to: ");
      Serial.println(value);
      break;
      
    default:
      Serial.println("Error: Unknown PID command");
      Serial.println("Valid commands: P (proportional), I (integral), D (derivative)");
      Serial.println("Example: P100, I50, D25");
      break;
  }
  
  // Display current gains
  Serial.print("Current gains - P: ");
  Serial.print(current_P);
  Serial.print(", I: ");
  Serial.print(current_I);
  Serial.print(", D: ");
  Serial.println(current_D);
}

// Debug function to print PPM values and SD card status
void printPPMDebug() {
  
  // Also show raw PPM debug info
  handController.debugPPM();
}

void setup() {
  //initialize serial communication
  Serial.println("Initializing Serial Communication...");
  Serial.begin(SERIAL_BAUD_RATE);
  
  // Initialize Hand Controller first
  Serial.println("Initializing Hand Controller...");
  handController.attach(ref_throttle, ref_roll, ref_pitch, ref_yawRate, emergencyStop);
  handController.init();
  Serial.println("Hand controller initialized - receives serial from Arduino Nano");

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
  ref_throttle = 0;

  // Run Servo Control Test
  Serial.println("Running Servo Control Test...");
  ServoControlTest();

  // Initialize IMU
  Serial.println("Initializing IMU...");
  IMU_Init( 1000/CONTROL_INTERVAL ); // Set IMU sample rate to match control loop rate (100 Hz)
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

  // Initialize control loop timer for 100 Hz operation
  if (controlTimer.begin(controlLoopISR, CONTROL_INTERVAL * 1000)) {
    Serial.println("Control loop timer started at 100 Hz.");
  } else {
    Serial.println("Error: Control loop timer failed to initialize!");
  } 

  rollPID.reset();
  pitchPID.reset();
  yawRatePID.reset();
}

void loop() {
  // Set timestamp zero point on first loop iteration
  if (loopStartTime == 0) {
    loopStartTime = millis();
    if (sdLogger.isReady()) {
      sdLogger.logMessage("Flight started - Loop timestamp zero point set");
    }
  }

  // Read Hand Controller inputs
  handController.readInputs();
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input == "debug") {
      printPPMDebug();
    } else if (input == "status") {
      Serial.println("SYSTEM OPERATIONAL");
      printPPMDebug();
    } else {
      PIDtuner(input);
      // processSerialCommand(input);  // Commented out - using PIDtuner instead
    }
  }
  
  //Emergency stop loop
  while (emergencyStop) {
    // Set everything to safe state once
    //ServoControl_SetAngle(0, 0, 0, 0);
    Motor_SetSpeed(0);
    Serial.println("*** EMERGENCY STOP ACTIVE ***");
   
    // Wait and recheck killswitch (handController sets emergencyStop directly)
    delay(500);
    handController.readInputs();
    if (!emergencyStop) {
      Serial.println("Emergency stop cleared - System operational");
      break;
    }
  }
 
 // Log flight data continuously
  currentTime = millis();
  if (sdLogger.isReady() && (currentTime - lastLogTime >= LOG_INTERVAL)) {
    // Log with timestamp relative to loop start (flight time)
    // Pass emergencyStop as kill indicator: 0=killed, 255=safe
    uint8_t killIndicator = emergencyStop ? 0 : 255;
    sdLogger.logDataWithCustomTime(currentTime - loopStartTime, ref_throttle, roll, pitch, yaw, killIndicator, 
                                  emergencyStop, ax_ms2, ay_ms2, az_ms2, gx_rps, gy_rps, gz_rps,
                                  ref_roll, ref_pitch, ref_yawRate);
    lastLogTime = currentTime;
  }
 
  // Print debug info periodically
  if (currentTime - lastDebugTime >= DEBUG_INTERVAL) {
    //printPPMDebug();
   lastDebugTime = currentTime;
 }
}
