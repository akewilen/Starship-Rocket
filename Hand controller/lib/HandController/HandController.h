#ifndef HANDCONTROLLER_H
#define HANDCONTROLLER_H

//grey pin 0
//green 27
//purple 26
//yellow 1

#define THROTTLE_PIN 1
#define PITCH_PIN 27
#define YAW_PIN 26
#define ROLL_PIN 28
#define KILL_SWITCH_PIN 0

#include "stdint.h"

// PPM signal constants
#define PPM_MIN_PULSE 1000    // Minimum pulse width in microseconds
#define PPM_MAX_PULSE 2000    // Maximum pulse width in microseconds
#define PPM_CENTER_PULSE 1500 // Center/neutral pulse width
#define PPM_TIMEOUT_MS 50     // Timeout for pulse validity in milliseconds

class HandController {
  public:
    void attach(uint8_t &throttleRef, uint8_t &pitchRef, uint8_t &yawRef, uint8_t &rollRef, uint8_t &killSwitchRef);
    void readInputs();
    void init();
    
  private:
    uint8_t pulseToValue(uint16_t pulseWidth);    // Convert pulse width to 0-255 value
    bool isPulseValid(uint16_t pulseWidth);       // Check if pulse is within valid range
    void setupPulseReading();
    
    uint8_t *throttle;
    uint8_t *pitch;
    uint8_t *yaw;
    uint8_t *roll;
    uint8_t *killSwitch;
    
    // Non-blocking pulse measurement variables
    static volatile uint32_t pulseStart[5];
    static volatile uint16_t pulseWidth[5];
    static volatile uint32_t lastPulseTime[5];
    static volatile bool pulseValid[5];
    
    // Interrupt handlers
    static void throttleISR();
    static void pitchISR();
    static void yawISR();
    static void rollISR();
    static void killSwitchISR();
    
    static void handlePulse(uint8_t channel, uint8_t pin);
};

#endif