/**
 * @file Finger.cpp
 * @brief Enhanced finger actuator control with reflexive grip
 * @version 2.0
 */

#include "Finger.h"

Finger::Finger(uint8_t pwmPin, uint8_t fsrPin) 
    : _pwmPin(pwmPin),
      _fsrPin(fsrPin),
      _currentPosition(0),
      _targetPosition(0),
      _minPosition(FINGER_POS_MIN),
      _maxPosition(FINGER_POS_MAX),
      _offset(0),
      _fsrValue(0.0f),
      _lastFsrValue(0.0f),
      _fsrDerivative(0.0f),
      _reflexActive(false),
      _reflexStartTime(0),
      _preReflexPosition(0),
      _reflexBoostAmount(0)
{
    // Constructor implementation
}

void Finger::begin() {
    // Initialize pins
    pinMode(_pwmPin, OUTPUT);
    pinMode(_fsrPin, INPUT);
    
    // Set initial position
    setTarget(_minPosition);
}

void Finger::update(float dt) {
    // Read FSR value
    readFSR();
    
    // Calculate FSR derivative for slip detection
    _fsrDerivative = (_fsrValue - _lastFsrValue) / dt;
    _lastFsrValue = _fsrValue;
    
    // Check if reflex is active and handle timeout
    if (_reflexActive) {
        uint32_t reflexDuration = millis() - _reflexStartTime;
        
        // Release reflex after REFLEX_DURATION_MS
        if (reflexDuration >= REFLEX_DURATION_MS) {
            _reflexActive = false;
            
            // Gradually return to pre-reflex target
            setTarget(_preReflexPosition);
            
            #ifdef DEBUG_REFLEX
            Serial.print(F("Reflex released on finger "));
            Serial.println(_pwmPin);
            #endif
        }
    }
    
    // Update position based on target and constraints
    updatePosition(dt);
    
    // Apply current position to servo/motor
    writePosition(_currentPosition);
}

void Finger::updatePosition(float dt) {
    // Simple position control with smoothing
    // In a real implementation, you might use PID or motion profiles
    
    const float maxSpeed = 400.0f; // degrees per second
    
    // Calculate position error
    int32_t error = _targetPosition - _currentPosition;
    
    // Apply speed limit
    float maxDelta = maxSpeed * dt;
    int32_t delta = constrain(error, -maxDelta, maxDelta);
    
    // Update position with limit
    _currentPosition = constrain(_currentPosition + delta, _minPosition, _maxPosition);
}

void Finger::reflexGrip() {
    // Reflex response to counter object slippage
    if (_reflexActive) {
        // Don't allow multiple reflexes too close together
        return;
    }
    
    // Store pre-reflex position and time
    _preReflexPosition = _targetPosition;
    _reflexStartTime = millis();
    
    // Calculate boost amount (percentage of remaining range)
    uint16_t remainingRange = _maxPosition - _currentPosition;
    _reflexBoostAmount = remainingRange * REFLEX_BOOST_PERCENT / 100;
    
    // Ensure minimum boost amount
    if (_reflexBoostAmount < REFLEX_MIN_BOOST) {
        _reflexBoostAmount = REFLEX_MIN_BOOST;
    }
    
    // Calculate new target with safety limits
    uint16_t reflexTarget = _currentPosition + _reflexBoostAmount;
    reflexTarget = constrain(reflexTarget, _currentPosition, _maxPosition);
    
    // Apply reflex position immediately
    _targetPosition = reflexTarget;
    _reflexActive = true;
    
    #ifdef DEBUG_REFLEX
    Serial.print(F("Reflex triggered on finger "));
    Serial.print(_pwmPin);
    Serial.print(F(" from "));
    Serial.print(_currentPosition);
    Serial.print(F(" to "));
    Serial.println(reflexTarget);
    #endif
    
    // For a more immediate response, we could also directly update the position:
    // _currentPosition = reflexTarget;
    // writePosition(_currentPosition);
}

void Finger::readFSR() {
    // Read analog value from FSR
    int rawValue = analogRead(_fsrPin);
    
    // Convert to normalized range 0.0-1.0
    _fsrValue = rawValue / 1023.0f;
}

void Finger::setTarget(uint16_t position) {
    // Apply position constraints
    _targetPosition = constrain(position, _minPosition, _maxPosition);
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

void Finger::writePosition(uint16_t position) {
    // Apply offset
    int32_t adjustedPos = position + _offset;
    
    // Ensure position is within valid range
    adjustedPos = constrain(adjustedPos, _minPosition, _maxPosition);
    
    // Write to servo/motor
    // This implementation assumes servo-style control
    // Modify for your specific actuator type
    analogWrite(_pwmPin, map(adjustedPos, 0, 180, 0, 255));
}

float Finger::getFSR() const {
    return _fsrValue;
}

float Finger::getFsrDerivative() const {
    return _fsrDerivative;
}

bool Finger::isReflexActive() const {
    return _reflexActive;
}