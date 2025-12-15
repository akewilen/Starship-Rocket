
#include "HandController.h"
#include "Arduino.h"

void HandController::attach(uint8_t &throttleRef, uint8_t &pitchRef, uint8_t &yawRef, uint8_t &rollRef, uint8_t &killSwitchRef) {
    throttle = &throttleRef;
    pitch = &pitchRef;
    yaw = &yawRef;
    roll = &rollRef;
    killSwitch = &killSwitchRef;
}

void HandController::init() {
    // Configure PPM input pins (no pullup needed for PPM signals)
    pinMode(THROTTLE_PIN, INPUT);
    pinMode(PITCH_PIN, INPUT);
    pinMode(YAW_PIN, INPUT);
    pinMode(ROLL_PIN, INPUT);
    pinMode(KILL_SWITCH_PIN, INPUT);
}

void HandController::readInputs() {
    // Read PPM pulses and convert to 0-255 range
    if (throttle) {
        uint16_t pulse = readPPMPulse(THROTTLE_PIN);
        *throttle = pulseToValue(pulse);
    }
    if (pitch) {
        uint16_t pulse = readPPMPulse(PITCH_PIN);
        *pitch = pulseToValue(pulse);
    }
    if (yaw) {
        uint16_t pulse = readPPMPulse(YAW_PIN);
        *yaw = pulseToValue(pulse);
    }
    if (roll) {
        uint16_t pulse = readPPMPulse(ROLL_PIN);
        *roll = pulseToValue(pulse);
    }
    if (killSwitch) {
        uint16_t pulse = readPPMPulse(KILL_SWITCH_PIN);
        *killSwitch = pulseToValue(pulse);
    }
}



// Read PPM pulse width from specified pin
uint16_t HandController::readPPMPulse(uint8_t pin) {
    uint16_t pulseWidth = pulseIn(pin, HIGH, PPM_TIMEOUT);
    
    // If no pulse detected or invalid, return center value
    if (pulseWidth == 0 || !isPulseValid(pulseWidth)) {
        return PPM_CENTER_PULSE;
    }
    
    return pulseWidth;
}

// Convert PPM pulse width (1000-2000μs) to 0-255 value
uint8_t HandController::pulseToValue(uint16_t pulseWidth) {
    // Ensure pulse is within valid range
    if (!isPulseValid(pulseWidth)) {
        return 127; // Return center value (127) for invalid pulses
    }
    
    // Map pulse width from 1000-2000μs to 0-255
    // constrain() ensures we stay within bounds even if pulse is slightly outside normal range
    pulseWidth = constrain(pulseWidth, PPM_MIN_PULSE, PPM_MAX_PULSE);
    
    return map(pulseWidth, PPM_MIN_PULSE, PPM_MAX_PULSE, 0, 255);
}

// Check if pulse width is within valid PPM range
bool HandController::isPulseValid(uint16_t pulseWidth) {
    return (pulseWidth >= (PPM_MIN_PULSE - 100) && pulseWidth <= (PPM_MAX_PULSE + 100));
}