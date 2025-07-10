/**
 * @file Finger.h
 * @brief Enhanced finger control with reflexive grip capability
 * @version 2.0
 * @author Haix02
 * @date 2025-07-10
 */

#pragma once

#include <Arduino.h>
#include "config.h"

// Reflex configuration parameters
#define REFLEX_BOOST_PERCENT_MIN 10     // Minimum boost percentage
#define REFLEX_BOOST_PERCENT_MAX 20     // Maximum boost percentage
#define REFLEX_DURATION_MS 400          // Duration to hold reflex position (ms)
#define REFLEX_COOLDOWN_MS 250          // Minimum time between reflexes (ms)
#define REFLEX_RETURN_TIME_MS 100       // Time to gradually return to baseline (ms)

// Uncomment for debug output
//#define DEBUG_REFLEX

class Finger {
public:
    Finger(uint8_t pwmPin, uint8_t fsrPin);
    
    /**
     * @brief Initialize finger hardware
     */
    void begin();
    
    /**
     * @brief Update finger state (call periodically)
     * @param dt Time delta in seconds
     */
    void update(float dt);
    
    /**
     * @brief Trigger immediate reflexive grip response
     * Increases grip force by 10-20% to counter slip
     */
    void reflexGrip();
    
    /**
     * @brief Set target position
     * @param position Target position (0-180 degrees)
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
     * @brief Get current FSR reading
     * @return FSR value (0.0-1.0)
     */
    float getFSR() const;
    
    /**
     * @brief Get FSR rate of change
     * @return FSR derivative (dF/dt)
     */
    float getFsrDerivative() const;
    
    /**
     * @brief Check if reflex is currently active
     * @return True if reflex response is active
     */
    bool isReflexActive() const;

private:
    // Hardware pins
    uint8_t _pwmPin;
    uint8_t _fsrPin;
    
    // Position control
    uint16_t _currentPosition;
    uint16_t _targetPosition;
    uint16_t _maxPosition;
    int16_t _offset;
    
    // FSR feedback
    float _fsrValue;
    float _fsrLastValue;
    float _fsrDerivative;
    
    // Reflex state machine
    enum ReflexState {
        REFLEX_IDLE,        // No reflex active
        REFLEX_ACTIVE,      // Reflex boost applied
        REFLEX_RETURNING    // Gradually returning to baseline
    };
    
    ReflexState _reflexState;
    uint32_t _reflexStartTime;      // When reflex was triggered
    uint32_t _lastReflexTime;       // Last reflex activation (for cooldown)
    uint16_t _preReflexTarget;      // Target before reflex
    uint16_t _reflexTarget;         // Target during reflex
    uint16_t _boostAmount;          // Amount of boost applied
    
    // Private methods
    void updatePosition(float dt);
    void updateReflexState();
    void writePosition(uint16_t position);
    void readFSR();
    uint16_t calculateReflexBoost();
};