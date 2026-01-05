
#include <Wire.h>
#include <Arduino.h>
#include "Bitcraze_PMW3901.h"



// ESP32-WROOM (VSPI)
#define PMW3901_CS_PIN   5
#define PMW3901_INT_PIN  21

#define SPI_SCK_PIN      18
#define SPI_MISO_PIN     19
#define SPI_MOSI_PIN     23

// Interrupt service routine for optical flow sensor
extern volatile bool flowDataReady; // Flag to indicate if new data is available

void flowISR();
static void setupFlowISR(); // Function to set up the interrupt for the optical flow sensor


class Optical_flow {
public:
  bool begin();
  bool readMotionPolled(int16_t &dx, int16_t &dy);  // returns true only when new nonzero motion
  void setPollRateHz(uint16_t hz);

private:
  uint32_t _nextReadUs = 0;
  uint32_t _periodUs   = 10000; // 10 ms
};