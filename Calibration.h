/**
 * @file Calibration.h
 * @brief System calibration and EEPROM storage
 */

#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "EMGProcessor.h"
#include "Finger.h"

// EEPROM storage structure
struct CalibrationData {
    uint16_t magicNumber;         // To verify valid data
    float emgThresholds[NUM_EMG_CHANNELS];
    float emgOffsets[NUM_EMG_CHANNELS];
    int16_t fingerOffsets[NUM_FINGERS];
    float slipThreshold;
    uint8_t checksum;             // Simple validation
};

class Calibration {
public:
    Calibration();
    
    /**
     * @brief Initialize calibration module
     * @param emgProcessor Pointer to EMG processor
     * @param fingers Array of finger pointers
     */
    void begin(EMGProcessor* emgProcessor, Finger** fingers);
    
    /**
     * @brief Load calibration from EEPROM
     * @return True if valid data loaded
     */
    bool loadFromEEPROM();
    
    /**
     * @brief Save current calibration to EEPROM
     */
    void saveToEEPROM();
    
    /**
     * @brief Reset to default calibration
     */
    void resetToDefaults();
    
    /**
     * @brief Calibrate EMG resting levels
     */
    void calibrateEMGRest();
    
    /**
     * @brief Set EMG threshold for a channel
     */
    void setEMGThreshold(uint8_t channel, float threshold);
    
    /**
     * @brief Set EMG offset for a channel
     */
    void setEMGOffset(uint8_t channel, float offset);
    
    /**
     * @brief Set finger position offset
     */
    void setFingerOffset(uint8_t finger, int16_t offset);
    
    /**
     * @brief Set slip detection threshold
     */
    void setSlipThreshold(float threshold);
    
    /**
     * @brief Print current calibration values
     */
    void printCalibration();

private:
    EMGProcessor* _emgProcessor;
    Finger** _fingers;
    
    CalibrationData _calibData;
    
    // Magic number to identify valid calibration data
    static const uint16_t MAGIC_NUMBER = 0xCAB1;
    
    // Calculate simple checksum for data validation
    uint8_t calculateChecksum(const CalibrationData& data);
    
    // Apply calibration to components
    void applyCalibration();
};