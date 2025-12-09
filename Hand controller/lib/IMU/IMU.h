#ifndef IMU_H
#define IMU_H

#include <Wire.h>

// Undefine Arduino macros that conflict with Eigen
#ifdef B0
#undef B0
#endif
#ifdef B1
#undef B1
#endif
#ifdef B2
#undef B2
#endif
#ifdef B3
#undef B3
#endif
#ifdef F
#undef F
#endif

#include <..\..\.pio\libdeps\teensy41\Eigen\Dense>

#define IMU_PIN_SDA  18  // Teensy 4.1 default SDA pin
#define IMU_PIN_SCL  19  // Teensy 4.1 default SCL pin

// ---------- ICM20600 (acc + gyro) ----------
#define ICM_ADDR       0x69
#define REG_PWR_MGMT1  0x6B
#define REG_ACCEL_X    0x3B   // ACCEL_XOUT_H
#define REG_SMPLRT_DIV   0x19 // For setting sample rate

// ---------- AK09918 (magnetometer) ----------
#define AK_ADDR        0x0C
#define AK_REG_ST1     0x10 //Data status 1
#define AK_REG_HXL     0x11   // HXL,HXH,HYL,HYH,HZL,HZH (LSB first)
#define AK_REG_ST2     0x18 // Data status 2
#define AK_REG_CNTL2   0x31 //control settings 1
#define AK_REG_CNTL3   0x32 //control settings 2

// Magnetometer sample rates
#define AK09918_NORMAL 0x01
#define AK09918_CONTINUOUS_10HZ 0x02
#define AK09918_CONTINUOUS_20HZ 0x04
#define AK09918_CONTINUOUS_50HZ 0x06
#define AK09918_CONTINUOUS_100HZ 0x08

//constants 
static float pi = 3.14159265359;
static float sensitivity = 131.0;
// Calibration offsets (Bias of sensors, computed in matlab)

//Unit conversions 
static float deg2rad = pi / 180.0;
static float rad2deg = 180.0 / pi;
static float ACC_SENS = 16384.0;   // LSB/g for ±2g
static float gravity = 9.81;

void IMU_Init();
void IMU_Read(float &ax_ms2, float &ay_ms2, float &az_ms2,
              float &gx_dps, float &gy_dps, float &gz_dps,
              int16_t &mx, int16_t &my, int16_t &mz);

// Helper functions for scaling
void ScaleGyro(int16_t gx_raw, int16_t gy_raw, int16_t gz_raw,
               float &gx_dps, float &gy_dps, float &gz_dps);
void ScaleAcc(int16_t ax_raw, int16_t ay_raw, int16_t az_raw,
              float &ax_ms2, float &ay_ms2, float &az_ms2);

Eigen::Vector3d IMU_ACC_BIAS_READ();
Eigen::Vector3d IMU_GYRO_BIAS_READ();
Eigen::Vector3d IMU_MAG_BIAS_READ();

#endif