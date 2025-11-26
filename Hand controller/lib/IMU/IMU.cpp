#include "IMU.h"
#include <Wire.h>
#include <Arduino.h>


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

void ScaleGyro(int16_t gx_raw, int16_t gy_raw, int16_t gz_raw,
               float &gx_dps, float &gy_dps, float &gz_dps)
{
    gx_dps = (float)gx_raw / sensitivity;
    gy_dps = (float)gy_raw / sensitivity;
    gz_dps = (float)gz_raw / sensitivity;
}

void ScaleAcc(int16_t ax_raw, int16_t ay_raw, int16_t az_raw,
                float &ax_ms2, float &ay_ms2, float &az_ms2)
{
    ax_ms2 = (float)ax_raw * gravity / ACC_SENS;
    ay_ms2 = (float)ay_raw * gravity / ACC_SENS;
    az_ms2 = (float)az_raw * gravity / ACC_SENS;
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

    gx_dps = (float)gx / sensitivity;
    gy_dps = (float)gy / sensitivity;
    gz_dps = (float)gz / sensitivity;
    
    ax_ms2 = (float)ax * gravity / ACC_SENS;
    ay_ms2 = (float)ay * gravity / ACC_SENS;
    az_ms2 = (float)az * gravity / ACC_SENS;
    
    // ---------- MAG ----------
    int16_t mx_new, my_new, mz_new;
    if (akReadMag(mx_new, my_new, mz_new)) {
        mx = mx_new;
        my = my_new;
        mz = mz_new;
    }
}

