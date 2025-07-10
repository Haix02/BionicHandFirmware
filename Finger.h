/**
 * @file Finger.h
 * @brief Enhanced finger actuator control with reflexive grip
 * @version 2.0
 */

#pragma once

#include <Arduino.h>
#include "config.h"

// Reflex parameters
#define REFLEX_BOOST_PERCENT 15     // Increase grip by 15% of remaining range
#define REFLEX_MIN_BOOST 5          // Minimum boost amount
#define REFLEX_DURATION_MS 400      // Duration of reflex grip (milliseconds)

//#define DEBUG_REFLEX              // Uncomment for debug output

class Finger {
public:
    Finger(uint8_t pwmPin, uint8_t fsrPin);
    
    void begin();
    void update(float dt);
    
    // Position control methods
    void setTarget(uint16_t position);
    uint16_t getTarget() const;
    uint16_t getCurrentPosition() const;
    void setOffset(int16_t offset);
    
    // Sensory feedback methods
    float getFSR() const;
    float getFsrDerivative() const;
    
    // Reflex control
    void reflexGrip();
    bool isReflexActive() const;

private:
    // Hardware pins
    uint8_t _pwmPin;
    uint8_t _fsrPin;
    
    // Position control
    uint16_t _currentPosition;
    uint16_t _targetPosition;
    uint16_t _minPosition;
    uint16_t _maxPosition;
    int16_t _offset;
    
    // FSR feedback
    float _fsrValue;
    float _lastFsrValue;
    float _fsrDerivative;
    
    // Reflex state
    bool _reflexActive;
    uint32_t _reflexStartTime;
    uint16_t _preReflexPosition;
    uint16_t _reflexBoostAmount;
    
    // Private methods
    void readFSR();
    void updatePosition(float dt);
    void writePosition(uint16_t position);
};