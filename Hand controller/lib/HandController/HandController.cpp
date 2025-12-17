#include "HandController.h"

void HandController::attach(uint8_t &throttleRef, double &refRollVal, double &refPitchVal, double &refYawRateVal, volatile bool &emergencyStopRef) {
    throttle = &throttleRef;
    refRoll = &refRollVal;
    refPitch = &refPitchVal;
    refYawRate = &refYawRateVal;
    emergencyStop = &emergencyStopRef;
    
    // Initialize with safe values
    *throttle = lastThrottle;
    *refRoll = 0.0;   // Center = 0 degrees
    *refPitch = 0.0;
    *refYawRate = 0.0;
    // emergencyStop starts as whatever the caller set it to
}

// Map 0-255 to -5 to +5 degrees with dead zone (120-134 -> 0)
double HandController::mapWithDeadZone(uint8_t value) {
    // Dead zone: 120-134 maps to 0
    if (value >= DEAD_ZONE_LOW && value <= DEAD_ZONE_HIGH) {
        return 0.0;
    }
    
    // Below dead zone: map 0-119 to -5 to ~0
    if (value < DEAD_ZONE_LOW) {
        // 0 -> -5.0, 119 -> small negative (just before dead zone)
        return ((double)value / DEAD_ZONE_LOW) * MAX_ANGLE_DEG - MAX_ANGLE_DEG;
    }
    
    // Above dead zone: map 135-255 to ~0 to +5
    // 135 -> small positive, 255 -> +5.0
    return ((double)(value - DEAD_ZONE_HIGH) / (255 - DEAD_ZONE_HIGH)) * MAX_ANGLE_DEG;
}

// Map 0-255 to -36 to +36 degrees/sec for yaw rate with dead zone (120-134 -> 0)
double HandController::mapYawRateWithDeadZone(uint8_t value) {
    const double MAX_YAW_RATE_DEG = 36.0;
    
    // Dead zone: 120-134 maps to 0
    if (value >= DEAD_ZONE_LOW && value <= DEAD_ZONE_HIGH) {
        return 0.0;
    }
    
    // Below dead zone: map 0-119 to -36 to ~0
    if (value < DEAD_ZONE_LOW) {
        // 0 -> -36.0, 119 -> small negative (just before dead zone)
        return ((double)value / DEAD_ZONE_LOW) * MAX_YAW_RATE_DEG - MAX_YAW_RATE_DEG;
    }
    
    // Above dead zone: map 135-255 to ~0 to +36
    // 135 -> small positive, 255 -> +36.0
    return ((double)(value - DEAD_ZONE_HIGH) / (255 - DEAD_ZONE_HIGH)) * MAX_YAW_RATE_DEG;
}

void HandController::init() {
    // Initialize Serial7 for receiving PPM data from Arduino Nano
    // RX7 is on pin 28
    PPM_SERIAL.begin(115200);
    
    // Clear any garbage in buffer
    while (PPM_SERIAL.available()) {
        PPM_SERIAL.read();
    }
    
    rxIndex = 0;
    inPacket = false;
}

void HandController::readInputs() {
    // Process all available serial data (non-blocking)
    while (PPM_SERIAL.available()) {
        uint8_t byte = PPM_SERIAL.read();
        
        if (!inPacket) {
            // Looking for start byte
            if (byte == PACKET_START) {
                inPacket = true;
                rxIndex = 0;
                rxBuffer[rxIndex++] = byte;
            }
        } else {
            // Collecting packet data
            rxBuffer[rxIndex++] = byte;
            
            // Check if we have a complete packet
            if (rxIndex >= PACKET_SIZE) {
                if (parsePacket()) {
                    // Successfully parsed - update outputs
                    // Scale throttle from 0-255 to 0-100
                    if (throttle) *throttle = (uint8_t)((lastThrottle * 100) / 255);
                    if (refRoll) *refRoll = mapWithDeadZone(lastRawRoll);
                    if (refPitch) *refPitch = mapWithDeadZone(lastRawPitch);
                    if (refYawRate) *refYawRate = mapYawRateWithDeadZone(lastRawYawRate);
                    
                    // Set emergencyStop based on kill switch threshold
                    if (emergencyStop) {
                        *emergencyStop = (lastKillSwitch < KILL_SWITCH_THRESHOLD);
                    }
                    
                    packetCount++;
                    lastPacketTime = millis();
                } else {
                    errorCount++;
                }
                
                // Reset for next packet
                inPacket = false;
                rxIndex = 0;
            }
        }
        
        // Safety: reset if buffer overflow
        if (rxIndex >= PACKET_SIZE) {
            inPacket = false;
            rxIndex = 0;
        }
    }
    
    // If no recent data, keep using cached values (already set)
    // This prevents sudden jumps if communication is temporarily lost
}

bool HandController::parsePacket() {
    // Packet format: [START] [Throttle] [Pitch] [Yaw] [Roll] [KillSw] [Checksum] [END]
    //                  0xAA     [1]       [2]    [3]   [4]     [5]       [6]      0x55
    
    // Verify start and end bytes
    if (rxBuffer[0] != PACKET_START || rxBuffer[7] != PACKET_END) {
        return false;
    }
    
    // Verify checksum (XOR of data bytes)
    uint8_t checksum = calculateChecksum(&rxBuffer[1], 5);
    if (checksum != rxBuffer[6]) {
        return false;
    }
    
    // All valid - extract raw values (0-255)
    // Packet format: [START][Throttle][Pitch][YawRate][Roll][KillSw][Checksum][END]
    lastThrottle = rxBuffer[1];
    lastRawPitch = rxBuffer[2];
    lastRawYawRate = rxBuffer[3];
    lastRawRoll = rxBuffer[4];
    lastKillSwitch = rxBuffer[5];
    
    return true;
}

uint8_t HandController::calculateChecksum(uint8_t* data, uint8_t len) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

void HandController::debugPPM() {
    Serial.println("=== HandController Serial Debug ===");
    
    Serial.print("Connection Status: ");
    if (isConnected()) {
        Serial.println("CONNECTED");
    } else {
        Serial.println("DISCONNECTED (no data in 500ms)");
    }
    
    Serial.print("Last Packet: ");
    Serial.print(millis() - lastPacketTime);
    Serial.println("ms ago");
    
    Serial.print("Packets Received: ");
    Serial.println(packetCount);
    
    Serial.print("Checksum Errors: ");
    Serial.println(errorCount);
    
    Serial.print("Raw Values (0-255): ");
    Serial.print("T:"); Serial.print(lastThrottle);
    Serial.print(" R:"); Serial.print(lastRawRoll);
    Serial.print(" P:"); Serial.print(lastRawPitch);
    Serial.print(" Y:"); Serial.print(lastRawYawRate);
    Serial.print(" K:"); Serial.println(lastKillSwitch);
    
    Serial.print("Mapped Values (deg): ");
    Serial.print("T:"); Serial.print((lastThrottle * 100) / 255, 1);
    Serial.print(" Roll:"); Serial.print(mapWithDeadZone(lastRawRoll), 2);
    Serial.print(" Pitch:"); Serial.print(mapWithDeadZone(lastRawPitch), 2);
    Serial.print(" YawRate:"); Serial.print(mapYawRateWithDeadZone(lastRawYawRate), 2);
    Serial.println();
    
    Serial.print("Emergency Stop: ");
    Serial.println(lastKillSwitch < KILL_SWITCH_THRESHOLD ? "ACTIVE" : "OFF");
    
    Serial.print("Serial7 Buffer: ");
    Serial.print(PPM_SERIAL.available());
    Serial.println(" bytes waiting");
    
    Serial.println("====================");
}





