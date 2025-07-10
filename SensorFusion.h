/**
 * @file SensorFusion.h
 * @brief Enhanced sensor fusion with IMU integration
 * @version 2.0
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// Optional IMU support
#ifdef USE_MPU6050
#include <MPU6050.h>
#endif

class SensorFusion {
public:
    SensorFusion();
    
    /**
     * @brief Initialize sensor fusion system
     * @return True if initialized successfully
     */
    bool begin();
    
    /**
     * @brief Update sensor readings and fusion algorithms
     */
    void update();
    
    /**
     * @brief Set EMG feature vector
     * @param emgFeatures Array of EMG feature values
     * @param numChannels Number of EMG channels
     */
    void setEMGFeatures(const float* emgFeatures, uint8_t numChannels);
    
    /**
     * @brief Set FSR values
     * @param fsrValues Array of FSR values (0.0-1.0)
     * @param numFingers Number of fingers
     */
    void setFSRValues(const float* fsrValues, uint8_t numFingers);
    
    /**
     * @brief Get current hand orientation
     * @param roll Pointer to store roll (X rotation) in degrees
     * @param pitch Pointer to store pitch (Y rotation) in degrees
     * @param yaw Pointer to store yaw (Z rotation) in degrees
     */
    void getOrientation(float* roll, float* pitch, float* yaw) const;
    
    /**
     * @brief Get slip risk assessment based on sensor fusion
     * @return Risk factor from 0.0 (no risk) to 1.0 (high risk)
     */
    float getSlipRisk() const;
    
    /**
     * @brief Get recommended grip force multiplier
     * @return Force multiplier from 0.5 (low force) to 2.0 (high force)
     */
    float getGripForceMultiplier() const;
    
    /**
     * @brief Get detected activity context
     * @return Activity context code (0=idle, 1=holding, 2=moving)
     */
    uint8_t getActivityContext() const;
    
    /**
     * @brief Check if an IMU is present and working
     * @return True if IMU is available
     */
    bool hasIMU() const;
    
    /**
     * @brief Print sensor fusion status
     * @param stream Stream to print to (e.g., Serial)
     */
    void printStatus(Stream& stream) const;

private:
    // IMU data
    bool _imuPresent;
    float _accel[3];    // X, Y, Z acceleration in g
    float _gyro[3];     // X, Y, Z rotation in deg/s
    float _orientation[3]; // roll, pitch, yaw in degrees
    
    // FSR data
    float _fsrValues[MAX_FINGERS];
    float _prevFsrValues[MAX_FINGERS];
    float _fsrDerivatives[MAX_FINGERS];
    uint8_t _numFingers;
    
    // EMG data
    float _emgFeatures[MAX_EMG_CHANNELS];
    uint8_t _numEmgChannels;
    
    // Fusion outputs
    float _slipRisk;
    float _gripForceMultiplier;
    uint8_t _activityContext;
    
    // Movement detection
    bool _isMoving;
    float _movementMagnitude;
    
    // Timing
    uint32_t _lastUpdateTime;
    
    // Internal methods
    bool initIMU();
    void updateIMU();
    void updateOrientation(float dt);
    void assessSlipRisk();
    void determineActivityContext();
    float calculateGripForceMultiplier() const;
    void detectMovement();
    
    #ifdef USE_MPU6050
    MPU6050 _mpu;
    #endif
};