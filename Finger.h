/**
 * @file Finger.h
 * @brief Enhanced finger control with motion profiles and reflex response
 * @version 2.0
 */

#pragma once

#include <Arduino.h>
#include "config.h"

/**
 * @enum MotionProfileType
 * @brief Type of motion profile for finger movement
 */
enum class MotionProfileType {
    NONE,       // Direct control (no profile)
    TRAPEZOIDAL, // Trapezoidal velocity profile
    S_CURVE     // S-curve velocity profile
};

class Finger {
public:
    Finger(uint8_t servoPin, uint8_t fsrPin);
    
    // Original interface (maintained for compatibility)
    void begin();
    void update(float dt);
    
    void setTarget(uint16_t position);
    uint16_t getTarget() const;
    uint16_t getCurrentPosition() const;
    
    float getFSR() const;
    bool detectSlip();
    
    void setOffset(int16_t offset);
    
    // Enhanced methods
    
    /**
     * @brief Trigger a reflexive grip response
     * @param forceBoost Additional force to apply (0.0-1.0)
     */
    void reflexGrip(float forceBoost = 0.3f);
    
    /**
     * @brief Set motion profile for smoother movement
     */
    void setMotionProfile(MotionProfileType profile);
    
    /**
     * @brief Enable or disable soft contact detection
     */
    void enableSoftContact(bool enable);
    
    /**
     * @brief Set soft contact force threshold
     */
    void setSoftContactThreshold(float threshold);
    
    /**
     * @brief Configure PID parameters
     */
    void setPIDGains(float kp, float ki, float kd);

private:
    uint8_t _servoPin;
    uint8_t _fsrPin;
    
    // Position control
    uint16_t _targetPosition;
    uint16_t _currentPosition;
    int16_t _offset;
    
    // FSR state
    float _fsrValue;
    float _fsrDerivative;
    float _prevFSRValue;
    bool _slipDetected;
    
    // Enhanced motion control
    MotionProfileType _motionProfile;
    bool _softContactEnabled;
    float _softContactThreshold;
    
    // Timing parameters for motion profiles
    float _moveStartTime;
    uint16_t _moveStartPosition;
    float _moveDuration;
    
    // PID control
    float _pidKp;
    float _pidKi;
    float _pidKd;
    float _pidIntegral;
    float _pidLastError;
    
    // Internal methods
    void writePosition(uint16_t position);
    float calculateTrapezoidal(float t, float t_total);
    float calculateSCurve(float t, float t_total);
    void updateSoftContact(float dt);
    void updatePID(float dt);
};