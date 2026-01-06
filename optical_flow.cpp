#include "optical_flow.h"
#include <SPI.h>
#include "Bitcraze_PMW3901.h"

Bitcraze_PMW3901 flow_PMW3901(PMW3901_CS_PIN);

bool Optical_flow::begin() {
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, PMW3901_CS_PIN);

  Serial.println("PMW3901 SPI initialized (polling mode).");
  delay(50);

  bool ok = flow_PMW3901.begin();
  Serial.println(ok ? "PMW3901 begin OK" : "PMW3901 begin FAILED");
  _nextReadUs = micros();
  return ok;
}

void Optical_flow::setPollRateHz(uint16_t hz) {
  if (hz == 0) hz = 1;
  _periodUs = 1000000UL / hz;
}

bool Optical_flow::readMotionPolled(int16_t &dx, int16_t &dy) {
  const uint32_t now = micros();
  if ((int32_t)(now - _nextReadUs) < 0) {
    dx = 0; dy = 0;
    return false; // not time yet
  }
  _nextReadUs += _periodUs;

  int16_t x = 0, y = 0;
  flow_PMW3901.readMotionCount(&x, &y);

  // Treat zero motion as "no new usable data"
  if (x == 0 && y == 0) {
    dx = 0; dy = 0;
    return false;
  }

  dx = x; dy = y;
  return true;
}
