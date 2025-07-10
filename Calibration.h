/**
 * @file Calibration.h
 * @brief EEPROM-based calibration storage for bionic hand system
 * @version 2.0
 * @author Haix02
 * @date 2025-07-10 23:13:16 UTC
 */

#pragma once

#include <Arduino.h>
#include "config.h"

// Forward declarations
class EMGProcessor;
class GraspManager;
class Finger;

// EEPROM configuration
#define EEPROM_START_ADDRESS 0
#define CALIBRATION_MAGIC 0xCAFE
#define CALIBRATION_VERSION 2

// Default calibration values
#define EMG_DEFAULT_OFFSET 512.0f
#define EMG_DEFAULT_THRESHOLD 0.05f
#define MAX_FINGERS 8

// Calibration data structure stored in EEPROM
struct CalibrationData {
    // Header
    uint16_t magicNumber;                    // Magic number for validation
    uint8_t version;                         // Calibration format version
    uint32_t timestamp;                      // Last calibration time
    
    // EMG calibration
    float emgOffsets[NUM_EMG_CHANNELS];      // DC offsets for each EMG channel
    float emgThresholds[NUM_EMG_CHANNELS];   // Activity thresholds for each channel
    
    // Finger calibration
    int16_t fingerOffsets[MAX_FINGERS];      // Position offsets for each finger
    
    // System parameters
    float graspForceMultiplier;              // Global force scaling factor
    float reflexSensitivity;                 // Reflex detection sensitivity
    
    // Reserved for future expansion
    uint8_t reserved[32];
    
    // Data integrity
    uint16_t checksum;                       // Data validation checksum
};

class Calibration {
public:
    Calibration();
    
    /**
     * @brief Initialize calibration module with system components
     * @param emgProcessor Pointer to EMG processor
     * @param graspManager Pointer to grasp manager
     * @param fingers Array of finger pointers
     * @param numFingers Number of fingers
     */
    void begin(EMGProcessor* emgProcessor, 
              GraspManager* graspManager,
              Finger** fingers, 
              uint8_t numFingers);
    
    /**
     * @brief Load calibration data from EEPROM
     * @return True if calibration loaded successfully
     */
    bool loadCalibration();
    
    /**
     * @brief Save current calibration data to EEPROM
     * @return True if calibration saved successfully
     */
    bool saveCalibration();
    
    /**
     * @brief Reset calibration to factory defaults
     */
    void resetCalibration();
    
    /**
     * @brief Perform EMG calibration sequence
     */
    void calibrateEMG();
    
    /**
     * @brief Set EMG channel threshold
     * @param channel EMG channel index (0-7)
     * @param threshold Activity threshold (0.0-1.0)
     */
    void setEMGThreshold(uint8_t channel, float threshold);
    
    /**
     * @brief Set EMG channel offset
     * @param channel EMG channel index (0-7)
     * @param offset DC offset value
     */
    void setEMGOffset(uint8_t channel, float offset);
    
    /**
     * @brief Set finger position offset
     * @param finger Finger index (0-3)
     * @param offset Position offset (-50 to +50)
     */
    void setFingerOffset(uint8_t finger, int16_t offset);
    
    /**
     * @brief Set global grasp force multiplier
     * @param multiplier Force scaling factor (0.1-2.0)
     */
    void setGraspForceMultiplier(float multiplier);
    
    /**
     * @brief Set reflex sensitivity
     * @param sensitivity Sensitivity factor (0.1-2.0)
     */
    void setReflexSensitivity(float sensitivity);
    
    /**
     * @brief Get calibration values
     */
    float getEMGThreshold(uint8_t channel) const;
    float getEMGOffset(uint8_t channel) const;
    int16_t getFingerOffset(uint8_t finger) const;
    float getGraspForceMultiplier() const;
    float getReflexSensitivity() const;
    
    /**
     * @brief Print current calibration to serial
     */
    void printCalibration() const;
    
    /**
     * @brief Check if calibration data is valid
     * @return True if calibration is valid
     */
    bool isCalibrationValid() const;
    
    /**
     * @brief Export calibration data for backup
     */
    void exportCalibration() const;
    
    /**
     * @brief Perform factory reset (erase all EEPROM data)
     */
    void factoryReset();
    
    /**
     * @brief Perform self-test of calibration system
     * @return True if all tests pass
     */
    bool performSelfTest() const;

private:
    // System component pointers
    EMGProcessor* _emgProcessor;
    GraspManager* _graspManager;
    Finger** _fingers;
    uint8_t _numFingers;
    bool _initialized;
    
    // Calibration data
    CalibrationData _calibData;
    
    // Internal methods
    void resetCalibrationData();
    void applyCalibration();
    uint16_t calculateChecksum(const CalibrationData& data) const;
};