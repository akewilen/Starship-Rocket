

#include <Arduino.h>
#include "optical_flow.h"

Optical_flow flow;

//float D = 0.485;
//int res = 451;
//float s = 0.0011; //s = D/count


//calibration
float H = 0.71; //test height
float L = 0.6; //test length
double k = k = 0.0018331247;

// --- Calibration state ---
static int64_t total_dx = 0;
static bool calibrating = false;
static uint32_t calib_start_ms = 0;
static const uint32_t CALIB_TIME_MS = 10000; // time to move L meters

void setup() {
  Serial.begin(115200);
  while (!Serial) { } // optional depending on board

  if (!flow.begin()) {
    Serial.println("Optical flow init failed. Halting.");
    while (true) { delay(1000); }
  }
}
void loop() {
  static int32_t acc_dx = 0, acc_dy = 0;
  static uint32_t t0 = micros();

  // Low-pass filter state
  static float vx_f = 0.0f, vy_f = 0.0f;

  // Choose cutoff frequency (Hz)
  const float fc = 10.0f;                 // try 5–15 Hz as a starting point
  const float tau = 1.0f / (2.0f * (float)M_PI * fc);

  int16_t dx = 0, dy = 0;

  if (flow.readMotionPolled(dx, dy)) {
    acc_dx += dx;
    acc_dy += dy;
    if (calibrating) {
    total_dx += dx;   // RAW counts, no filtering
  }
  }

  uint32_t now = micros();
  uint32_t dt_us = now - t0;
  
  if (dt_us >= 10000) {                   // 10 ms window

    if (calibrating) {
  if (calib_start_ms == 0) {
    calib_start_ms = millis();
    total_dx = 0;
    Serial.println("Calibration started: move 0.6 m straight in X");
  }

  if (millis() - calib_start_ms >= CALIB_TIME_MS) {
    if (fabs((double)total_dx) > 1.0) {
      k = (double)L / ((double)H * (double)total_dx);
    } else {
      k = 0.0;
    }

    calibrating = false;

    Serial.print("Calibration done. total_dx = ");
    Serial.print((double)total_dx);
    Serial.print("  k = ");
    Serial.println(k, 10);
  }

  // During calibration, do NOT compute velocity
  acc_dx = 0;
  acc_dy = 0;
  t0 = now;
  return;
}


    float T = dt_us * 1e-6f;              // Period time in seconds
    float flow_x = (float)acc_dx / T;
    float flow_y = (float)acc_dy / T;
    float vx = k * H *  flow_x;
    float vy = k * H *  flow_y;


    // Low-pass alpha using the true window duration
    float alpha = T / (tau + T);

    vx_f += alpha * (vx - vx_f);
    vy_f += alpha * (vy - vy_f);

    float vel_f = sqrtf(vx_f * vx_f + vy_f * vy_f);

 
    Serial.print("vx_f: "); Serial.print(vx_f);
    Serial.print("  vy_f: "); Serial.print(vy_f);
   // Serial.print("  vel_f: "); Serial.print(vel_f);
   // Serial.print("  vx: "); Serial.print(vx);
   // Serial.print("  vy: "); Serial.print(vy);
    Serial.print("  flowx: "); Serial.print(flow_x );
    Serial.print("  flowy: "); Serial.println(flow_y );
  

    acc_dx = 0;
    acc_dy = 0;
    t0 = now;
  }
}
