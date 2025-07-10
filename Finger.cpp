/**
 * @file Finger.cpp
 * @brief Enhanced finger control with reflexive grip capability
 * @version 2.0
 * @author Haix02
 * @date 2025-07-10
 */

#include "Finger.h"

Finger::Finger(uint8_t pwmPin, uint8_t fsrPin) 
    : _pwmPin(pwmPin), 
      _fsrPin(fsrPin),
      _currentPosition(FINGER_POS_MIN),
      _targetPosition(FINGER_POS_MIN),
      _maxPosition(FINGER_POS_MAX),
      _offset(0),
      _fsrValue(0.0f),
      _fsrLastValue(0.0f),
      _fsrDerivative(0.0f),
      _reflexState(REFLEX_IDLE),
      _reflexStartTime(0),
      _lastReflexTime(0),
      _preReflexTarget(FINGER_POS_MIN),
      _reflexTarget(FINGER_POS_MIN),
      _boostAmount(0)
{
    // Constructor initialization complete
}

void Finger::begin() {
    // Initialize hardware pins
    pinMode(_pwmPin, OUTPUT);
    pinMode(_fsrPin, INPUT);
    
    // Set initial position
    _currentPosition = FINGER_POS_MIN;
    _targetPosition = FINGER_POS_MIN;
    
    // Write initial position to actuator
    writePosition(_currentPosition);
    
    #ifdef DEBUG_REFLEX
    Serial.print(F("Finger initialized on PWM pin "));
    Serial.print(_pwmPin);
    Serial.print(F(", FSR pin "));
    Serial.println(_fsrPin);
    #endif
}

void Finger::update(float dt) {
    // Read FSR sensor
    readFSR();
    
    // Calculate FSR derivative for slip detection
    _fsrDerivative = (_fsrValue - _fsrLastValue) / dt;
    _fsrLastValue = _fsrValue;
    
    // Update reflex state machine
    updateReflexState();
    
    // Update position based on current target
    updatePosition(dt);
    
    // Apply position to actuator
    writePosition(_currentPosition);
}

void Finger::reflexGrip() {
    uint32_t currentTime = millis();
    
    // Check cooldown period to prevent reflex spam
    if (currentTime - _lastReflexTime < REFLEX_COOLDOWN_MS) {
        #ifdef DEBUG_REFLEX
        Serial.println(F("Reflex blocked - cooldown active"));
        #endif
        return;
    }
    
    // Don't trigger if already in reflex state
    if (_reflexState != REFLEX_IDLE) {
        #ifdef DEBUG_REFLEX
        Serial.println(F("Reflex blocked - already active"));
        #endif
        return;
    }
    
    // Record timing for state machine
    _reflexStartTime = currentTime;
    _lastReflexTime = currentTime;
    
    // Store current target for later restoration
    _preReflexTarget = _targetPosition;
    
    // Calculate boost amount (10-20% of remaining range)
    _boostAmount = calculateReflexBoost();
    
    // Calculate new reflex target with safety clamping
    _reflexTarget = _currentPosition + _boostAmount;
    _reflexTarget = constrain(_reflexTarget, _currentPosition, _maxPosition);
    
    // Apply reflex target immediately for fast response
    _targetPosition = _reflexTarget;
    _currentPosition = _reflexTarget; // Immediate response
    
    // Enter reflex active state
    _reflexState = REFLEX_ACTIVE;
    
    #ifdef DEBUG_REFLEX
    Serial.print(F("REFLEX TRIGGERED - Pin "));
    Serial.print(_pwmPin);
    Serial.print(F(" | Boost: "));
    Serial.print(_boostAmount);
    Serial.print(F(" | From "));
    Serial.print(_preReflexTarget);
    Serial.print(F(" to "));
    Serial.println(_reflexTarget);
    #endif
    
    // Immediately write new position for fastest response
    writePosition(_currentPosition);
}

void Finger::updateReflexState() {
    uint32_t currentTime = millis();
    uint32_t elapsedTime = currentTime - _reflexStartTime;
    
    switch (_reflexState) {
        case REFLEX_IDLE:
            // No reflex active - normal operation
            break;
            
        case REFLEX_ACTIVE:
            // Check if reflex duration has elapsed
            if (elapsedTime >= REFLEX_DURATION_MS) {
                // Transition to returning state
                _reflexState = REFLEX_RETURNING;
                _reflexStartTime = currentTime; // Reset timer for return phase
                
                #ifdef DEBUG_REFLEX
                Serial.print(F("REFLEX RETURNING - Pin "));
                Serial.println(_pwmPin);
                #endif
            }
            break;
            
        case REFLEX_RETURNING:
            // Gradually return to pre-reflex target
            if (elapsedTime >= REFLEX_RETURN_TIME_MS) {
                // Return phase complete
                _targetPosition = _preReflexTarget;
                _reflexState = REFLEX_IDLE;
                
                #ifdef DEBUG_REFLEX
                Serial.print(F("REFLEX COMPLETE - Pin "));
                Serial.print(_pwmPin);
                Serial.print(F(" | Returned to "));
                Serial.println(_preReflexTarget);
                #endif
            } else {
                // Interpolate between reflex target and pre-reflex target
                float progress = (float)elapsedTime / REFLEX_RETURN_TIME_MS;
                progress = constrain(progress, 0.0f, 1.0f);
                
                // Smooth easing function (ease-out)
                progress = 1.0f - (1.0f - progress) * (1.0f - progress);
                
                uint16_t interpolatedTarget = _reflexTarget + 
                    (int32_t)((_preReflexTarget - _reflexTarget) * progress);
                
                _targetPosition = interpolatedTarget;
            }
            break;
    }
}

uint16_t Finger::calculateReflexBoost() {
    // Calculate boost based on remaining range to maximum
    uint16_t currentPos = _currentPosition;
    uint16_t remainingRange = _maxPosition - currentPos;
    
    // Use dynamic boost percentage based on current position
    // More boost when far from max, less when close to max
    float boostPercent;
    if (remainingRange > 50) {
        boostPercent = REFLEX_BOOST_PERCENT_MAX; // 20% when plenty of range
    } else if (remainingRange > 20) {
        boostPercent = REFLEX_BOOST_PERCENT_MIN + 
                      ((REFLEX_BOOST_PERCENT_MAX - REFLEX_BOOST_PERCENT_MIN) * 
                       (remainingRange - 20) / 30.0f);
    } else {
        boostPercent = REFLEX_BOOST_PERCENT_MIN; // 10% when close to max
    }
    
    // Calculate boost amount
    uint16_t boost = (uint16_t)(remainingRange * boostPercent / 100.0f);
    
    // Ensure minimum boost for effectiveness
    if (boost < 5 && remainingRange >= 5) {
        boost = 5;
    }
    
    // Ensure we don't exceed maximum position
    if (currentPos + boost > _maxPosition) {
        boost = _maxPosition - currentPos;
    }
    
    return boost;
}

void Finger::updatePosition(float dt) {
    // Simple position control with rate limiting
    if (_currentPosition == _targetPosition) {
        return; // Already at target
    }
    
    // Calculate position error
    int32_t error = _targetPosition - _currentPosition;
    
    // Apply speed limit for smooth movement (except during reflex)
    float maxSpeed = (_reflexState == REFLEX_ACTIVE) ? 1000.0f : 300.0f; // deg/s
    float maxDelta = maxSpeed * dt;
    
    // Calculate change with speed limit
    int32_t delta = constrain(error, -maxDelta, maxDelta);
    
    // Update current position
    _currentPosition = constrain(_currentPosition + delta, FINGER_POS_MIN, _maxPosition);
}

void Finger::writePosition(uint16_t position) {
    // Apply calibration offset
    int32_t adjustedPos = position + _offset;
    
    // Clamp to valid range with safety margin
    adjustedPos = constrain(adjustedPos, FINGER_POS_MIN, _maxPosition);
    
    // Convert to PWM value and write to pin
    // Assumes servo control - modify for other actuator types
    uint8_t pwmValue = map(adjustedPos, 0, 180, 0, 255);
    analogWrite(_pwmPin, pwmValue);
}

void Finger::readFSR() {
    // Read analog value from FSR
    uint16_t rawValue = analogRead(_fsrPin);
    
    // Convert to normalized range 0.0-1.0
    _fsrValue = rawValue / 1023.0f;
}

void Finger::setTarget(uint16_t position) {
    // Only update target if not in active reflex
    if (_reflexState == REFLEX_IDLE) {
        _targetPosition = constrain(position, FINGER_POS_MIN, _maxPosition);
    } else {
        // Store as pre-reflex target for later use
        _preReflexTarget = constrain(position, FINGER_POS_MIN, _maxPosition);
    }
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

bool Finger::isReflexActive() const {
    return (_reflexState != REFLEX_IDLE);
}