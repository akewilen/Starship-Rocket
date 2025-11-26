

#include <Arduino.h>
#include <Wire.h>
#include "AK09918.h"
#include "ICM20600.h"

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

// ------- Defines sample rate of magnetometer
#define AK09918_NORMAL 0x01,
#define AK09918_CONTINUOUS_10HZ 0x02,
#define AK09918_CONTINUOUS_20HZ 0x04,
#define AK09918_CONTINUOUS_50HZ 0x06,
#define AK09918_CONTINUOUS_100HZ 0x08,

static ICM20600 icm(true);
static AK09918 ak;

//constants 
static float pi = 3.14159265359;
static float sensitivity = 131.0;
// Calibration offsets (Bias of sensors, computed in matlab)
static float ax_offset = 0.0408f, ay_offset = 0.3858f, az_offset = 0.15f;
static float gx_offset = -0.5573f, gy_offset = -0.1036f, gz_offset =  -0.3663f;
static float mx_offset = 83.3796, my_offset = 220.8556, mz_offset =  217.8227;
//Unit conversions 
static float deg2rad = pi / 180.0;
static float rad2deg = 180.0 / pi;
static float ACC_SENS = 16384.0;   // LSB/g for ±2g
static float gravity = 9.81;



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

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== ICM20600 + AK09918 raw test (CSV) ===");

  // CSV header
  Serial.println("ax,ay,az,gx,gy,gz,mx,my,mz");

  Wire.begin(21, 22);   // ESP32: SDA=21, SCL=22
  delay(10);

  // ----- ICM20600 init -----
  // Wake up device: clear SLEEP bit in PWR_MGMT_1 (0x6B)
  icmWrite(REG_PWR_MGMT1, 0x00);
  delay(100);
  icmSetSampleRate(100);

  // ----- AK09918 init -----
  // Soft reset
  akWrite(AK_REG_CNTL3, 0x01);
  delay(50);
  // Continuous measurement mode 4 (100 Hz)
  akWrite(AK_REG_CNTL2, 0x08);
  delay(50);
}

void loop() {
  // ---------- ACC + GYRO ----------
  uint8_t buf[14];

  icmReadBytes(REG_ACCEL_X, buf, 14);

  int16_t ax = (buf[0] << 8) | buf[1];
  int16_t ay = (buf[2] << 8) | buf[3];
  int16_t az = (buf[4] << 8) | buf[5];

  int16_t gx = (buf[8]  << 8) | buf[9];
  int16_t gy = (buf[10] << 8) | buf[11];
  int16_t gz = (buf[12] << 8) | buf[13];

  float gx_dps, gy_dps, gz_dps;
  float ax_ms2, ay_ms2, az_ms2;

  ScaleGyro(gx, gy, gz, gx_dps, gy_dps, gz_dps);
  ScaleAcc(ax, ay, az, ax_ms2, ay_ms2, az_ms2);

  // ---------- MAG ----------
  static int16_t mx = 0, my = 0, mz = 0;  // keep last values if no new data
  int16_t mx_new, my_new, mz_new;
  if (akReadMag(mx_new, my_new, mz_new)) {
    mx = mx_new;// - mx_offset;
    my = my_new;// - my_offset;
    mz = mz_new;// - mz_offset;
  }

  // ---------- CSV output ----------
  Serial.print(ax_ms2); Serial.print(',');
  Serial.print(ay_ms2); Serial.print(',');
  Serial.print(az_ms2); Serial.print(',');
  Serial.print(gx_dps); Serial.print(',');
  Serial.print(gy_dps); Serial.print(',');
  Serial.print(gz_dps); Serial.print(',');
  Serial.print(mx); Serial.print(',');
  Serial.print(my); Serial.print(',');
  Serial.println(mz);

  delay(10);  // ~50 Hz logging
}
