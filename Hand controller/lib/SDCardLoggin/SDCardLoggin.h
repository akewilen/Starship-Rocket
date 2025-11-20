#ifndef SDCARDLOGGIN_H
#define SDCARDLOGGIN_H

#include "Arduino.h"
#include "SD.h"
#include "SPI.h"

// Teensy 4.1 built-in SD card uses SDIO interface
#define SD_CHIP_SELECT BUILTIN_SDCARD

// Logging configuration
#define LOG_FILENAME "FLIGHT.CSV"
#define LOG_BUFFER_SIZE 512

class SDCardLogger {
public:
    // Initialize SD card and create log file
    bool init();
    
    // Log data entry with timestamp
    bool logData(uint8_t throttle, uint8_t pitch, uint8_t yaw, uint8_t roll, 
                uint8_t killSwitch, bool emergencyStop = false, 
                float batteryVoltage = 0.0, float temperature = 0.0);
    
    // Log data entry with custom timestamp
    bool logDataWithCustomTime(unsigned long customTime, uint8_t throttle, uint8_t pitch, uint8_t yaw, uint8_t roll, 
                              uint8_t killSwitch, bool emergencyStop = false);
    
    // Log custom message
    bool logMessage(const String& message);
    
    // Flush buffer to SD card
    void flush();
    
    // Close current log file
    void close();
    
    // Get current log filename
    String getCurrentLogFile();
    
    // Check if SD card is ready
    bool isReady();
    
    // Get free space on SD card (in MB)
    uint32_t getFreeSpaceMB();
    
private:
    File logFile;
    String currentLogFileName;
    bool sdReady;
    unsigned long lastFlushTime;
    
    // Create unique filename with timestamp
    String generateLogFileName();
    
    // Write CSV header
    bool writeCSVHeader();
    
    // Get current timestamp string
    String getTimestamp();
};

// Global instance
extern SDCardLogger sdLogger;

#endif