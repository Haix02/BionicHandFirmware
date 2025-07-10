/**
 * @file Finger.cpp
 * @brief Enhanced finger control with reflexive grip
 * @version 2.0
 */

#include "Finger.h"

// Constructor implementation
Finger::Finger(uint8_t pwmPin, uint8_t fsrPin) 
    : _pwmPin(pwmPin), 
      _fsrPin(fsrPin),
      _currentPosition(FINGER_POS_MIN),
      _targetPosition(FINGER_POS_MIN),
      _offset(0),
      _fsrValue(0.0f),
      _fsrLastValue(0.0f),
      _fsrDerivative(0.0f),
      _reflexActive(false),
      _reflexStartTime(0),
      _preReflexTarget(0),
      _lastReflexTime(0)
{
    // Constructor implementation
}

void Finger::begin() {
    // Initialize pins
    pinMode(_pwmPin, OUTPUT);
    pinMode(_fsrPin, INPUT);
    
    // Set initial position
    setTarget(FINGER_POS_MIN);
    _currentPosition = FINGER_POS_MIN;
    
    // Write initial position to servo/motor
    writePosition(_currentPosition);
}

void Finger::update(float dt) {
    // Read FSR value
    uint16_t rawValue = analogRead(_fsrPin);
    _fsrValue = rawValue / 1023.0f; // Normalize to 0.0-1.0
    
    // Calculate FSR derivative for slip detection
    _fsrDerivative = (_fsrValue - _fsrLastValue) / dt;
    _fsrLastValue = _fsrValue;
    
    // Handle reflex timeout if active
    if (_reflexActive) {
        uint32_t elapsedTime = millis() - _reflexStartTime;
        
        // Check if reflex duration has elapsed
        if (elapsedTime >= REFLEX_DURATION_MS) {
            _reflexActive = false;
            
            // Gradually return to original target
            setTarget(_preReflexTarget);
            
            #ifdef DEBUG_REFLEX
            Serial.print(F("Releasing reflex on finger "));
            Serial.println(_pwmPin);
            #endif
        }
    }
    
    // Update position based on target
    updatePosition(dt);
    
    // Apply current position to servo/motor
    writePosition(_currentPosition);
}

void Finger::reflexGrip() {
    uint32_t currentTime = millis();
    
    // Check cooldown to prevent reflex spam
    if (currentTime - _lastReflexTime < REFLEX_COOLDOWN_MS) {
        return;
    }
    
    // Record reflex activation time
    _lastReflexTime = currentTime;
    _reflexStartTime = currentTime;
    
    // Store pre-reflex target for later restoration
    _preReflexTarget = _targetPosition;
    
    // Calculate boost amount (percentage of remaining range)
    uint16_t currentPos = _currentPosition;
    uint16_t remainingRange = FINGER_POS_MAX - currentPos;
    uint16_t boostAmount = remainingRange * REFLEX_BOOST_PERCENT / 100;
    
    // Ensure minimum boost amount
    if (boostAmount < REFLEX_MIN_BOOST) {
        boostAmount = REFLEX_MIN_BOOST;
    }
    
    // Calculate new position with clamping to max
    uint16_t newPosition = constrain(currentPos + boostAmount, currentPos, FINGER_POS_MAX);
    
    // Apply new position immediately
    _targetPosition = newPosition;
    _currentPosition = newPosition; // Immediate update for faster response
    _reflexActive = true;
    
    #ifdef DEBUG_REFLEX
    Serial.print(F("Reflex grip on finger "));
    Serial.print(_pwmPin);
    Serial.print(F(" from "));
    Serial.print(currentPos);
    Serial.print(F(" to "));
    Serial.println(newPosition);
    #endif
    
    // Apply to servo/motor directly
    writePosition(_currentPosition);
}

void Finger::updatePosition(float dt) {
    // Simple position control with rate limiting
    // In a real implementation, consider PID or motion profiles
    
    // Calculate position error
    int32_t error = _targetPosition - _currentPosition;
    
    // Apply speed limit (degrees per second)
    float maxSpeed = 400.0f;
    float maxDelta = maxSpeed * dt;
    
    // Calculate change with speed limit
    int32_t delta = constrain(error, -maxDelta, maxDelta);
    
    // Update position
    _currentPosition = constrain(_currentPosition + delta, FINGER_POS_MIN, FINGER_POS_MAX);
}

void Finger::writePosition(uint16_t position) {
    // Apply offset
    int32_t adjustedPos = position + _offset;
    
    // Clamp to valid range
    adjustedPos = constrain(adjustedPos, FINGER_POS_MIN, FINGER_POS_MAX);
    
    // Map to PWM range and write to pin
    // This assumes servo range is 0-180 mapped to PWM 0-255
    uint8_t pwmValue = map(adjustedPos, 0, 180, 0, 255);
    analogWrite(_pwmPin, pwmValue);
}

void Finger::setTarget(uint16_t position) {
    _targetPosition = constrain(position, FINGER_POS_MIN, FINGER_POS_MAX);
}

uint16_t Finger::getTarget() const {
    return _targetPosition;
}

uint16_t Finger::getCurrentPosition() const {
    return _currentPosition;
}

void Finger::setOffset(int16_t offset) {
    _offset = offset;
}

float Finger::getFSR() const {
    return _fsrValue;
}

float Finger::getFsrDerivative() const {
    return _fsrDerivative;
}