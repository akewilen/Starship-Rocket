/*
 * Arduino Nano PPM Decoder
 * 
 * Reads 5 PPM channels from RC receiver and sends to Teensy via Serial
 * 
 * Pin Assignments:
 *   D3 - Throttle (CH1)
 *   D4 - Pitch (CH2)
 *   D5 - Yaw (CH3)
 *   D6 - Roll (CH4)
 *   D7 - Kill Switch (CH5)
 *   TX (D1) -> Teensy RX7 (Pin 28)
 *   GND -> Teensy GND (CRITICAL!)
 * 
 * Protocol: 8-byte packet
 *   [0xAA] [Throttle] [Pitch] [Yaw] [Roll] [KillSw] [Checksum] [0x55]
 */

#include <Arduino.h>

// Pin definitions
#define THROTTLE_PIN 4
#define PITCH_PIN 3
#define YAW_PIN 6
#define ROLL_PIN 5
#define KILL_SWITCH_PIN 7

// PPM constants
#define PPM_MIN 1000
#define PPM_MAX 2000
#define PPM_TIMEOUT 25000  // 25ms timeout for pulseIn

// Protocol constants
#define START_BYTE 0xAA
#define END_BYTE 0x55

// Channel values (0-255)
uint8_t throttle = 127;
uint8_t pitch = 127;
uint8_t yaw = 127;
uint8_t roll = 127;
uint8_t killSwitch = 210;  // Start in safe state (> 127)

// Convert PPM pulse (1000-2000us) to 0-255 value
uint8_t ppmToValue(uint16_t pulse) {
    if (pulse == 0) return 127;  // No signal = center
    if (pulse < PPM_MIN) return 0;
    if (pulse > PPM_MAX) return 255;
    return map(pulse, PPM_MIN, PPM_MAX, 0, 255);
}

// Read a single PPM channel
uint16_t readChannel(uint8_t pin) {
    return pulseIn(pin, HIGH, PPM_TIMEOUT);
}

// Send data packet to Teensy
void sendPacket() {
    uint8_t checksum = throttle ^ pitch ^ yaw ^ roll ^ killSwitch;
    
    Serial.write(START_BYTE);
    Serial.write(throttle);
    Serial.write(pitch);
    Serial.write(yaw);
    Serial.write(roll);
    Serial.write(killSwitch);
    Serial.write(checksum);
    Serial.write(END_BYTE);
}

void setup() {
    // Initialize serial at 115200 baud
    Serial.begin(115200);
    
    // Configure PPM input pins
    pinMode(THROTTLE_PIN, INPUT);
    pinMode(PITCH_PIN, INPUT);
    pinMode(YAW_PIN, INPUT);
    pinMode(ROLL_PIN, INPUT);
    pinMode(KILL_SWITCH_PIN, INPUT);
    
    // Brief delay to let everything stabilize
    delay(100);
    
    // Send a few initial packets with safe values
    for (int i = 0; i < 5; i++) {
        sendPacket();
        delay(20);
    }
}

void loop() {
    // Read all 5 PPM channels
    // This takes 15-25ms total - that's fine on the Nano!
    uint16_t pulseThrottle = readChannel(THROTTLE_PIN);
    uint16_t pulsePitch = readChannel(PITCH_PIN);
    uint16_t pulseYaw = readChannel(YAW_PIN);
    uint16_t pulseRoll = readChannel(ROLL_PIN);
    uint16_t pulseKill = readChannel(KILL_SWITCH_PIN);
    
    // Convert to 0-255 values
    // Only update if we got a valid pulse (non-zero)
    if (pulseThrottle > 0) {
        throttle = ppmToValue(pulseThrottle);
    } else {
        throttle = 0;  // No signal = throttle off for safety
    }
    if (pulsePitch > 0) pitch = ppmToValue(pulsePitch);
    if (pulseYaw > 0) yaw = ppmToValue(pulseYaw);
    if (pulseRoll > 0) roll = ppmToValue(pulseRoll);
    if (pulseKill > 0) killSwitch = ppmToValue(pulseKill);
    
    // Send packet to Teensy
    sendPacket();
    
    // Small delay to prevent flooding (~30-40Hz update rate)
    delay(5);
}