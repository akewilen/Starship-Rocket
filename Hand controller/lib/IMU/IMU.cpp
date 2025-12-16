#include "IMU.h"
#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <..\..\.pio\libdeps\teensy41\Eigen\Dense>

// ---------- ICM helpers ----------
void icmWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void icmReadBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);       // repeated start
  Wire.requestFrom(ICM_ADDR, len);
  for (uint8_t i = 0; i < len && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}

// ---------- AK helpers ----------
void akWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(AK_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t akRead1(uint8_t reg) {
  Wire.beginTransmission(AK_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(AK_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0;
}

void akReadBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(AK_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(AK_ADDR, len);
  for (uint8_t i = 0; i < len && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}


void icmSetSampleRate(uint16_t rate_hz) {
  if (rate_hz > 1000) rate_hz = 1000;
  if (rate_hz < 4)    rate_hz = 4;   

  uint8_t div = (uint8_t)((1000 / rate_hz) - 1);
  icmWrite(REG_SMPLRT_DIV, div);
}
// Read one mag sample; returns true if DRDY and data OK
bool akReadMag(int16_t &mx, int16_t &my, int16_t &mz) {
  // 1) Check data ready (ST1, bit0 = DRDY)
  uint8_t st1 = akRead1(AK_REG_ST1);
  if (!(st1 & 0x01)) {
    return false;   // no new data yet
  }

  // 2) Read 6 data bytes HXL..HZH  (LSB first!)
  uint8_t buf[6];
  akReadBytes(AK_REG_HXL, buf, 6);

  mx = (int16_t)((buf[1] << 8) | buf[0]); // HXH:HXL
  my = (int16_t)((buf[3] << 8) | buf[2]); // HYH:HYL
  mz = (int16_t)((buf[5] << 8) | buf[4]); // HZH:HZL

  // 3) Read ST2 to clear DRDY & possible overflow flag
  (void)akRead1(AK_REG_ST2);

  return true;
}

void IMU_Init() {
  // Set I2C pins BEFORE calling Wire.begin() on Teensy 4.1
  Wire.setSDA(IMU_PIN_SDA);
  Wire.setSCL(IMU_PIN_SCL);
  Wire.begin();   // Initialize I2C with configured pins
  Wire.setClock(400000); // Set I2C speed to 400kHz for better performance
  
  Serial.println("IMU: Initializing I2C on pins SDA=" + String(IMU_PIN_SDA) + ", SCL=" + String(IMU_PIN_SCL));
  
  // Test I2C communication
  Serial.println("IMU: Testing I2C devices...");
  Wire.beginTransmission(ICM_ADDR);
  uint8_t icm_error = Wire.endTransmission();
  Serial.println("ICM20600 (0x" + String(ICM_ADDR, HEX) + "): " + (icm_error == 0 ? "FOUND" : "NOT FOUND"));
  
  Wire.beginTransmission(AK_ADDR);
  uint8_t ak_error = Wire.endTransmission();
  Serial.println("AK09918 (0x" + String(AK_ADDR, HEX) + "): " + (ak_error == 0 ? "FOUND" : "NOT FOUND"));
  
  // ----- ICM20600 init -----
  Serial.println("IMU: Initializing ICM20600...");
  // Wake up device: clear SLEEP bit in PWR_MGMT_1 (0x6B)
  icmWrite(REG_PWR_MGMT1, 0x00);
  delay(100);
  icmSetSampleRate(100);
  Serial.println("IMU: ICM20600 configured");
  
  // ----- AK09918 init -----
  Serial.println("IMU: Initializing AK09918...");
  // Soft reset
  akWrite(AK_REG_CNTL3, 0x01);
  delay(50);
  // Continuous measurement mode 4 (100 Hz)
  akWrite(AK_REG_CNTL2, 0x08);
  delay(50);
  Serial.println("IMU: AK09918 configured");
  
  Serial.println("IMU: Initialization complete");
}

void IMU_Read(float &ax_ms2, float &ay_ms2, float &az_ms2,
              float &gx_dps, float &gy_dps, float &gz_dps,
              int16_t &mx, int16_t &my, int16_t &mz) {
    
    // ---------- ACC + GYRO ----------
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    uint8_t buf[14];
    
    // Read all 14 bytes starting from accelerometer registers
    icmReadBytes(REG_ACCEL_X, buf, 14);
    
    // Parse accelerometer data (bytes 0-5)
    ax = (int16_t)((buf[0] << 8) | buf[1]);
    ay = (int16_t)((buf[2] << 8) | buf[3]);
    az = (int16_t)((buf[4] << 8) | buf[5]);
    gx = (int16_t)((buf[8]  << 8) | buf[9]);
    gy = (int16_t)((buf[10] << 8) | buf[11]);
    gz = (int16_t)((buf[12] << 8) | buf[13]);

    // Return in rad/s and m/s^2
    gx_dps = ((float)gx / SENSITIVITY) * M_PI/180.0;
    gy_dps = ((float)gy / SENSITIVITY) * M_PI/180.0;
    gz_dps = ((float)gz / SENSITIVITY) * M_PI/180.0;
    
    ax_ms2 = (float)ax * GRAVITY / ACC_SENS;
    ay_ms2 = (float)ay * GRAVITY / ACC_SENS;
    az_ms2 = (float)az * GRAVITY / ACC_SENS;
    
    // ---------- MAG ----------
    int16_t mx_new, my_new, mz_new;
    if (akReadMag(mx_new, my_new, mz_new)) {
        mx = mx_new;
        my = my_new;
        mz = mz_new;
    }
}

Eigen::Vector3d IMU_ACC_BIAS_READ() {
  const int num_readings = 100;
  const int reading_delay_ms = 10; // 100Hz

  Eigen::Vector3d acc_sum(0.0, 0.0, 0.0);
  Eigen::Vector3d expected_acc(0.0, 0.0, 9.82);
  for (int i = 0; i < num_readings; ++i) {
    // Temporary variables to hold the readings for each loop
    int16_t mx_r, my_r, mz_r;
    float ax_m, ay_m, az_m, gx_d, gy_d, gz_d;

    IMU_Read(ax_m, ay_m, az_m, gx_d, gy_d, gz_d, mx_r, my_r, mz_r);
    
    acc_sum[0] += ax_m;
    acc_sum[1] += ay_m;
    acc_sum[2] += az_m;
    
    delay(reading_delay_ms);
  }

  acc_sum = acc_sum / num_readings;

  Serial.print("Accelerometer Bias (m/s^2): ");
  Serial.print("Ax: "); Serial.print(acc_sum[0], 4);
  Serial.print(" | Ay: "); Serial.print(acc_sum[1], 4);
  Serial.print(" | Az: "); Serial.println(acc_sum[2], 4);

  acc_sum = acc_sum - expected_acc;

  return acc_sum;
}

Eigen::Vector3d IMU_GYRO_BIAS_READ() {
  const int num_readings = 100;
  const int reading_delay_ms = 10; // 100Hz

  Eigen::Vector3d gyro_sum(0.0, 0.0, 0.0);
  for (int i = 0; i < num_readings; ++i) {
    // Temporary variables to hold the readings for each loop
    int16_t mx_r, my_r, mz_r;
    float ax_m, ay_m, az_m, gx_d, gy_d, gz_d;

    IMU_Read(ax_m, ay_m, az_m, gx_d, gy_d, gz_d, mx_r, my_r, mz_r);
    
    gyro_sum[0] += gx_d;
    gyro_sum[1] += gy_d;
    gyro_sum[2] += gz_d;
    
    delay(reading_delay_ms);
  }

  gyro_sum = gyro_sum / num_readings;

  Serial.print("Gyroscope Bias (rad/s): ");
  Serial.print("Gx: "); Serial.print(gyro_sum[0], 4);
  Serial.print(" | Gy: "); Serial.print(gyro_sum[1], 4);
  Serial.print(" | Gz: "); Serial.println(gyro_sum[2], 4);

  return gyro_sum;
}

Eigen::Vector3d IMU_MAG_BIAS_READ() {
  const int num_readings = 100;
  const int reading_delay_ms = 10; // 100Hz

  Eigen::Vector3d mag_sum(0.0, 0.0, 0.0);
  for (int i = 0; i < num_readings; ++i) {
    // Temporary variables to hold the readings for each loop
    int16_t mx_r, my_r, mz_r;
    float ax_m, ay_m, az_m, gx_d, gy_d, gz_d;

    IMU_Read(ax_m, ay_m, az_m, gx_d, gy_d, gz_d, mx_r, my_r, mz_r);
    
    mag_sum[0] += mx_r;
    mag_sum[1] += my_r;
    mag_sum[2] += mz_r;
    
    delay(reading_delay_ms);
  }

  mag_sum = mag_sum / num_readings;

  Serial.print("Magnetometer Bias (raw): ");
  Serial.print("Mx: "); Serial.print(mag_sum[0], 4);
  Serial.print(" | My: "); Serial.print(mag_sum[1], 4);
  Serial.print(" | Mz: "); Serial.println(mag_sum[2], 4);

  return mag_sum;
}