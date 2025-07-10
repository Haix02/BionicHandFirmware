/**
 * @file SensorFusion.h
 * @brief Enhanced sensor fusion for context-aware grip control
 * @version 3.0
 */

#pragma once

#include <Arduino.h>
#include "config.h"
#include <Wire.h>

/**
 * @class SensorFusion
 * @brief Integrates multiple sensor inputs to create context-aware control
 * 
 * V3 Enhancements:
 * - IMU integration for hand orientation awareness
 * - FSR + IMU + EMG combined context detection
 * - Predictive slip detection based on hand orientation
 * - Activity/context classification 
 */
class SensorFusion {
public:
    SensorFusion();
    
    /**
     * @brief Initialize sensor fusion system
     */
    bool begin();
    
    /**
     * @brief Update all sensor readings and fusion algorithms
     */
    void update();
    
    /**
     * @brief Set EMG feature vector input
     */
    void setEMGFeatures(const float* emgFeatures, uint8_t numFeatures);
    
    /**
     * @brief Set FSR values for all fingers
     */
    void setFSRValues(const float* fsrValues, uint8_t numFSRs);
    
    /**
     * @brief Get current hand orientation (roll, pitch, yaw)
     */
    void getHandOrientation(float* roll, float* pitch, float* yaw);
    
    /**
     * @brief Get slip risk assessment (0-1 scale)
     */
    float getSlipRisk() const;
    
    /**
     * @brief Get recommended grip force scale based on context
     */
    float getRecommendedGripForce() const;
    
    /**
     * @brief Get current grip stability assessment (0-1 scale)
     */
    float getGripStability() const;
    
    /**
     * @brief Get detected activity context
     */
    uint8_t getActivityContext() const;
    
    /**
     * @brief Print debug information
     */
    void printDebug(Stream& s);
    
private:
    // IMU data
    bool _imuPresent;
    float _accelData[3];   // x, y, z in g
    float _gyroData[3];    // x, y, z in deg/s
    float _orientation[3]; // roll, pitch, yaw in degrees
    
    // FSR state
    float _fsrValues[NUM_FINGERS];
    float _totalGripForce;
    bool _stableGrip;
    
    // EMG state
    float _emgFeatures[NUM_EMG_CHANNELS];
    float _emgActivity;
    
    // Fusion outputs
    float _slipRisk;
    float _recommendedForceScale;
    float _gripStability;
    uint8_t _activityContext;
    
    // IMU management
    bool initIMU();
    void updateIMU();
    void calculateOrientation();
    
    // Context detection
    void assessSlipRisk();
    void assessGripStability();
    void classifyActivity();
    void calculateRecommendedForce();
};