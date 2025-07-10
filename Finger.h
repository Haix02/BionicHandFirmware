/**
 * @file Finger.h
 * @brief Enhanced finger actuator with motion profiling and PID control
 * @version 3.0
 */

#pragma once

#include <Arduino.h>
#include "config.h"

// Forward declarations for types that might be platform-specific
class PID;

/**
 * @enum MotionProfileType
 * @brief Types of motion profiles for biomimetic finger movement
 */
enum class MotionProfileType {
    None,       ///< No profiling (direct positioning)
    Trapezoidal,///< Trapezoidal velocity profile
    SCurve      ///< S-curve velocity profile (smoother)
};

/**
 * @enum FingerState
 * @brief Logical state of each finger.
 */
enum class FingerState : uint8_t {
    Idle,
    Moving,
    Gripping,
    Slipping,
    Error
};

/**
 * @class Finger
 * @brief Encapsulates finger actuator, FSR feedback, PID, and reflex logic.
 * 
 * V3 Enhancements:
 * - Motion profiling with trapezoidal or S-curve velocity
 * - PID control for accurate positioning
 * - Soft contact detection and grip adaptation
 * - Direct reflex control bypassing normal pipeline
 */
class Finger {
public:
    Finger(uint8_t pwmPin, uint8_t fsrPin);

    // Main actuator control interface
    void begin();
    void update(float dt_ms);

    void setTarget(uint16_t position);
    uint16_t getTarget() const;
    uint16_t getCurrentPosition() const;
    float getFSR() const;

    // Reflex: slip detection & response
    void sampleFSR(); ///< Call at 100Hz from ISR
    bool detectSlip();
    
    // V3: Enhanced reflexGrip with immediate response
    void reflexGrip();
    
    // V3: Motion profile configuration
    void setMotionProfile(MotionProfileType type);
    void setVelocityLimits(float maxVel, float maxAccel);
    
    // V3: Soft contact grip adaptation
    void enableSoftContact(bool enable);
    void setSoftContactThreshold(float threshold);
    
    // V3: PID tuning
    void setPIDGains(float kp, float ki, float kd);

    // Calibration
    void setOffset(int16_t offset);

    // State
    FingerState getState() const;
    void setState(FingerState st);

    // Debug
    void printDebug(Stream &s);

private:
    // Hardware pins
    uint8_t _pwmPin, _fsrPin;
    int16_t _offset;

    // Position feedback (if available)
    uint16_t _currentPosition;
    uint16_t _targetPosition;

    // PID control 
    float _pidErrorInt;
    float _pidLastError;
    float _pidKp, _pidKi, _pidKd;  // V3: Configurable PID gains
    PID* _pidController;           // V3: Enhanced PID controller

    // FSR feedback
    float _fsrVal;                 ///< Current FSR reading
    float _fsrHist[FSR_DERIV_WINDOW];
    uint8_t _fsrHistIdx;

    // Slip detection
    float _fsrDFdt;                ///< Derivative
    bool  _slipDetected;

    // V3: Motion profile parameters
    MotionProfileType _profileType;
    float _maxVelocity;
    float _maxAcceleration;
    float _profileStartTime;
    uint16_t _profileStartPos;
    bool _motionProfileActive;
    
    // V3: Soft contact parameters
    bool _softContactEnabled;
    float _softContactThreshold;
    float _currentVelocity;

    FingerState _state;

    void actuate(uint16_t pwm_val);
    void updateSlipDetection();
    
    // V3: Enhanced methods
    float calculateTrapezoidal(float t, float startPos, float endPos, float maxVel, float maxAccel);
    float calculateSCurve(float t, float startPos, float endPos, float maxVel, float maxAccel);
    float calculateVelocityScale();
    void resetMotionProfile();
};