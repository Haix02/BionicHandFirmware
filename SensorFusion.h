/**
 * @file SensorFusion.h
 * @brief Enhanced sensor fusion with IMU integration and slip detection
 * @version 2.0
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// MPU6050 IMU Address
#define MPU6050_ADDR 0x68

// Maximum number of sensors
#define MAX_FSR_COUNT 8
#define MAX_EMG_COUNT 8

class SensorFusion {
public:
    SensorFusion();
    
    /**
     * @brief Initialize sensor fusion
     * @return True if initialization successful
     */
    bool begin();
    
    /**
     * @brief Update sensor fusion with latest data
     */
    void update();
    
    /**
     * @brief Set FSR sensor values
     * @param values Array of FSR values (0.0-1.0)
     * @param count Number of FSR values
     */
    void setFSRValues(const float* values, uint8_t count);
    
    /**
     * @brief Set EMG sensor values
     * @param values Array of EMG values
     * @param count Number of EMG values
     */
    void setEMGValues(const float* values, uint8_t count);
    
    /**
     * @brief Get current orientation from IMU
     * @param roll Output roll angle in degrees
     * @param pitch Output pitch angle in degrees
     * @param yaw Output yaw angle in degrees
     */
    void getOrientation(float* roll, float* pitch, float* yaw) const;
    
    /**
     * @brief Get calculated slip risk
     * @return Risk factor from 0.0 (no risk) to 1.0 (high risk)
     */
    float getSlipRisk() const;
    
    /**
     * @brief Get force multiplier based on sensor fusion
     * @return Force multiplier factor (typically 0.8-2.0)
     */
    float getForceMultiplier() const;
    
    /**
     * @brief Check if IMU is present and working
     * @return True if IMU is available
     */
    bool hasIMU() const;
    
    /**
     * @brief Get FSR derivative for slip detection
     * @param index FSR index
     * @return Derivative value (negative indicates slip)
     */
    float getFsrDerivative(uint8_t index) const;
    
    /**
     * @brief Detect slip on specific FSR
     * @param index FSR index
     * @param threshold Negative threshold to consider slip
     * @return True if slip detected
     */
    bool detectSlip(uint8_t index, float threshold = -0.2f) const;

private:
    bool _imuPresent;       // Is IMU present and working
    
    // FSR data
    float _fsrValues[MAX_FSR_COUNT];
    float _fsrLastValues[MAX_FSR_COUNT];
    float _fsrDerivatives[MAX_FSR_COUNT];
    float _fsrFiltered[MAX_FSR_COUNT];
    uint8_t _fsrCount;
    
    // EMG data
    float _emgValues[MAX_EMG_COUNT];
    uint8_t _emgCount;
    
    // IMU data
    float _accel[3];        // x, y, z acceleration in g
    float _gyro[3];         // x, y, z rotation in deg/s
    float _orientation[3];  // roll, pitch, yaw in degrees
    
    // Fusion results
    float _slipRisk;        // 0.0-1.0 slip risk assessment
    float _forceMultiplier; // Force multiplier for grip
    
    uint32_t _lastUpdateTime;
    
    // Private methods
    bool initIMU();
    void updateIMU();
    void updateOrientation(float dt);
    void updateFsrDerivatives(float dt);
    void assessSlipRisk();
    void calculateForceMultiplier();
    
    // Helper function to map a value from one range to another
    float map(float x, float in_min, float in_max, float out_min, float out_max) const {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};