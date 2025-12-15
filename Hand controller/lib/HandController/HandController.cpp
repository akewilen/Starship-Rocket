
#include "HandController.h"
#include "Arduino.h"

// Static variable definitions
volatile uint32_t HandController::pulseStart[5] = {0, 0, 0, 0, 0};
volatile uint16_t HandController::pulseWidth[5] = {PPM_CENTER_PULSE, PPM_CENTER_PULSE, PPM_CENTER_PULSE, PPM_CENTER_PULSE, PPM_CENTER_PULSE};
volatile uint32_t HandController::lastPulseTime[5] = {0, 0, 0, 0, 0};
volatile bool HandController::pulseValid[5] = {false, false, false, false, false};

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
    
    setupPulseReading();
}

void HandController::readInputs() {
    uint32_t currentTime = millis();
    
    // Read PPM pulses and convert to 0-255 range (non-blocking)
    if (throttle) {
        uint16_t pulse = (currentTime - lastPulseTime[0] < PPM_TIMEOUT_MS && pulseValid[0]) ? pulseWidth[0] : PPM_CENTER_PULSE;
        *throttle = pulseToValue(pulse);
    }
    if (pitch) {
        uint16_t pulse = (currentTime - lastPulseTime[1] < PPM_TIMEOUT_MS && pulseValid[1]) ? pulseWidth[1] : PPM_CENTER_PULSE;
        *pitch = pulseToValue(pulse);
    }
    if (yaw) {
        uint16_t pulse = (currentTime - lastPulseTime[2] < PPM_TIMEOUT_MS && pulseValid[2]) ? pulseWidth[2] : PPM_CENTER_PULSE;
        *yaw = pulseToValue(pulse);
    }
    if (roll) {
        uint16_t pulse = (currentTime - lastPulseTime[3] < PPM_TIMEOUT_MS && pulseValid[3]) ? pulseWidth[3] : PPM_CENTER_PULSE;
        *roll = pulseToValue(pulse);
    }
    if (killSwitch) {
        uint16_t pulse = (currentTime - lastPulseTime[4] < PPM_TIMEOUT_MS && pulseValid[4]) ? pulseWidth[4] : PPM_CENTER_PULSE;
        *killSwitch = pulseToValue(pulse);
    }
}



void HandController::setupPulseReading() {
    // Attach interrupts for each channel
    attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), throttleISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PITCH_PIN), pitchISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(YAW_PIN), yawISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROLL_PIN), rollISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(KILL_SWITCH_PIN), killSwitchISR, CHANGE);
}

void HandController::handlePulse(uint8_t channel, uint8_t pin) {
    uint32_t currentTime = micros();
    
    if (digitalRead(pin) == HIGH) {
        // Rising edge - start timing
        pulseStart[channel] = currentTime;
    } else {
        // Falling edge - calculate pulse width
        if (pulseStart[channel] != 0) {
            uint32_t width = currentTime - pulseStart[channel];
            if (width >= PPM_MIN_PULSE - 100 && width <= PPM_MAX_PULSE + 100) {
                pulseWidth[channel] = width;
                lastPulseTime[channel] = millis();
                pulseValid[channel] = true;
            }
            pulseStart[channel] = 0;
        }
    }
}

// Interrupt service routines
void HandController::throttleISR() {
    handlePulse(0, THROTTLE_PIN);
}

void HandController::pitchISR() {
    handlePulse(1, PITCH_PIN);
}

void HandController::yawISR() {
    handlePulse(2, YAW_PIN);
}

void HandController::rollISR() {
    handlePulse(3, ROLL_PIN);
}

void HandController::killSwitchISR() {
    handlePulse(4, KILL_SWITCH_PIN);
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