/**
 * @file Finger.h
 * @brief Enhanced finger control with reflexive grip
 * @version 2.0
 */

#pragma once

#include <Arduino.h>
#include "config.h"

// Reflex parameters
#define REFLEX_DURATION_MS 400      // Duration of reflex grip (ms)
#define REFLEX_COOLDOWN_MS 300      // Minimum time between reflexes (ms)
#define REFLEX_BOOST_PERCENT 15     // Boost grip by 15% of remaining range
#define REFLEX_MIN_BOOST 10         // Minimum boost amount

//#define DEBUG_REFLEX              // Uncomment for debug output

class Finger {
public:
    Finger(uint8_t pwmPin, uint8_t fsrPin);
    
    /**
     * @brief Initialize pins and starting position
     */
    void begin();
    
    /**
     * @brief Update finger state (call periodically)
     * @param dt Time delta in seconds
     */
    void update(float dt);
    
    /**
     * @brief Trigger reflexive grip response
     * Temporarily increases grip force to prevent slip
     */
    void reflexGrip();
    
    /**
     * @brief Set target position
     * @param position Target position (0-180)
     */
    void setTarget(uint16_t position);
    
    /**
     * @brief Get current target position
     * @return Target position
     */
    uint16_t getTarget() const;
    
    /**
     * @brief Get current actual position
     * @return Current position
     */
    uint16_t getCurrentPosition() const;
    
    /**
     * @brief Set position offset for calibration
     * @param offset Position offset
     */
    void setOffset(int16_t offset);
    
    /**
     * @brief Get current FSR value
     * @return FSR value (0.0-1.0)
     */
    float getFSR() const;
    
    /**
     * @brief Get FSR derivative
     * @return FSR derivative (dF/dt)
     */
    float getFsrDerivative() const;

private:
    // Hardware pins
    uint8_t _pwmPin;
    uint8_t _fsrPin;
    
    // Position control
    uint16_t _currentPosition;
    uint16_t _targetPosition;
    int16_t _offset;
    
    // FSR feedback
    float _fsrValue;
    float _fsrLastValue;
    float _fsrDerivative;
    
    // Reflex state
    bool _reflexActive;
    uint32_t _reflexStartTime;
    uint32_t _lastReflexTime;
    uint16_t _preReflexTarget;
    
    // Private methods
    void updatePosition(float dt);
    void writePosition(uint16_t position);
};