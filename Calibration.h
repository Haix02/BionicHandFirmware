/**
 * @file Calibration.h
 * @brief User calibration and personalization
 * @version 3.0
 */

#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "Finger.h"
#include "EMGProcessor.h"

// Structure to store calibration data in EEPROM
struct CalibrationData {
    uint16_t magic;            // Magic number to check data validity
    float emgThresholds[NUM_EMG_CHANNELS];
    float emgOffsets[NUM_EMG_CHANNELS];
    int16_t fingerOffsets[NUM_FINGERS];
    float fsrScaling[NUM_FINGERS];
    float slipThreshold;
};

/**
 * @class Calibration
 * @brief Manages calibration data and personalization
 * 
 * V3 Enhancements:
 * - Extended calibration parameters
 * - EEPROM storage with verification
 * - Automated calibration procedures
 */
class Calibration {
public:
    /**
     * @brief Initialize calibration system with component pointers
     */
    Calibration(EMGProcessor* emgProcessor, Finger** fingers);
    
    /**
     * @brief Load calibration from EEPROM
     * @return true if valid data was loaded
     */
    bool loadFromEEPROM();
    
    /**
     * @brief Save current calibration to EEPROM
     */
    void saveToEEPROM();
    
    /**
     * @brief Restore default calibration settings
     */
    void restoreDefaults();
    
    /**
     * @brief Set EMG threshold for specific channel
     */
    void setEMGThreshold(uint8_t channel, float threshold);
    
    /**
     * @brief Set EMG offset for specific channel
     */
    void setEMGOffset(uint8_t channel, float offset);
    
    /**
     * @brief Set finger position offset
     */
    void setFingerOffset(uint8_t finger, int16_t offset);
    
    /**
     * @brief Set FSR scaling factor
     */
    void setFSRScaling(uint8_t finger, float scaling);
    
    /**
     * @brief Set slip detection threshold
     */
    void setSlipThreshold(float threshold);
    
    /**
     * @brief Get current calibration data
     */
    const CalibrationData& getData() const;
    
private:
    EMGProcessor* _emgProcessor;
    Finger** _fingers;
    
    CalibrationData _data;
    
    static constexpr uint16_t CALIBRATION_MAGIC = 0xC