#ifndef HANDCONTROLLER_H
#define HANDCONTROLLER_H

#include "stdint.h"
#include "Arduino.h"

// Serial communication protocol constants
#define PACKET_START 0xAA
#define PACKET_END 0x55
#define PACKET_SIZE 8

// Use Serial7 on Teensy 4.1 (RX on pin 28)
#define PPM_SERIAL Serial7

class HandController {
public:
    // Throttle is uint8_t (0-255)
    // Roll, Pitch, Yaw are double references in degrees (-5 to +5)
    // emergencyStop is set directly when killswitch < threshold
    void attach(uint8_t &throttleRef, double &refRoll, double &refPitch, double &refYawRate, volatile bool &emergencyStopRef);
    void init();
    void readInputs();
    void debugPPM(); // Debug method to check received values
    
    // Status info
    bool isConnected() { return lastPacketTime > 0 && (millis() - lastPacketTime) < 500; }
    unsigned long getLastPacketTime() { return lastPacketTime; }
    uint32_t getPacketCount() { return packetCount; }
    uint32_t getErrorCount() { return errorCount; }
    
private:
    bool parsePacket();
    uint8_t calculateChecksum(uint8_t* data, uint8_t len);
    double mapWithDeadZone(uint8_t value); // Maps 0-255 to -5 to +5 deg with dead zone
    
    uint8_t *throttle;
    double *refRoll;
    double *refPitch;
    double *refYawRate;
    volatile bool *emergencyStop;
    
    // Receive buffer
    uint8_t rxBuffer[PACKET_SIZE];
    uint8_t rxIndex = 0;
    bool inPacket = false;
    
    // Cached raw values from packet (0-255)
    uint8_t lastThrottle = 0;
    uint8_t lastRawRoll = 127;
    uint8_t lastRawPitch = 127;
    uint8_t lastRawYawRate = 127;
    uint8_t lastKillSwitch = 210; // Safe state (above 127)
    
    // Kill switch threshold
    static constexpr uint8_t KILL_SWITCH_THRESHOLD = 127;
    
    // Dead zone constants
    static constexpr uint8_t DEAD_ZONE_LOW = 120;
    static constexpr uint8_t DEAD_ZONE_HIGH = 134;
    static constexpr double MAX_ANGLE_DEG = 5.0;
    
    // Statistics
    unsigned long lastPacketTime = 0;
    uint32_t packetCount = 0;
    uint32_t errorCount = 0;
};

#endif
