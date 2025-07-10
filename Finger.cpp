/**
 * @file Finger.cpp
 * @brief Enhanced finger control implementation
 * @version 2.0
 */

#include "Finger.h"

Finger::Finger(uint8_t servoPin, uint8_t fsrPin)
    : _servoPin(servoPin), _fsrPin(fsrPin),
      _targetPosition(0), _currentPosition(0), _offset(0),
      _fsrValue(0.0f), _fsrDerivative(0.0f), _prevFSRValue(0.0f), _slipDetected(false),
      _motionProfile(MotionProfileType::NONE),
      _softContactEnabled(false), _softContactThreshold(0.5f),
      _moveStartTime(0), _moveStartPosition(0), _moveDuration(0),
      _pidKp(2.0f), _pidKi(0.1f), _pidKd(0.05f),
      _pidIntegral(0.0f), _pidLastError(0.0f)
{
}

void Finger::begin() {
    pinMode(_servoPin, OUTPUT);
    pinMode(_fsrPin, INPUT);
    
    // Initialize servo at current position
    writePosition(_currentPosition);
}

void Finger::update(float dt) {
    // Read FSR
    _fsrValue = analogRead(_fsrPin) / 1023.0f;
    
    // Calculate FSR derivative for internal slip detection
    _fsrDerivative = (_fsrValue - _prevFSRValue) / dt;
    _prevFSRValue = _fsrValue;
    
    // Update slip detection state
    _slipDetected = (_fsrDerivative < -0.2f && _fsrValue > 0.1f);
    
    // Motion control based on selected profile
    if (_motionProfile == MotionProfileType::NONE) {
        // Simple PID position control
        updatePID(dt);
    } else {
        // Use motion profile
        float elapsed = (millis() - _moveStartTime) / 1000.0f; // Seconds
        float progress;
        
        if (elapsed >= _moveDuration) {
            // Motion complete
            _currentPosition = _targetPosition;
        } else {
            if (_motionProfile == MotionProfileType::TRAPEZOIDAL) {
                progress = calculateTrapezoidal(elapsed, _moveDuration);
            } else {
                progress = calculateSCurve(elapsed, _moveDuration);
            }
            
            // Calculate new position based on progress
            int32_t distance = _targetPosition - _moveStartPosition;
            _currentPosition = _moveStartPosition + (distance * progress);
        }
    }
    
    // Apply soft contact behavior if enabled
    if (_softContactEnabled) {
        updateSoftContact(dt);
    }
    
    // Apply output
    writePosition(_currentPosition + _offset);
}

void Finger::setTarget(uint16_t position) {
    position = constrain(position, 0, 180); // Assume servo range 0-180
    
    if (position == _targetPosition) {
        return; // No change needed
    }
    
    // If using motion profiles, initialize movement
    if (_motionProfile != MotionProfileType::NONE) {
        _moveStartTime = millis();
        _moveStartPosition = _currentPosition;
        
        // Calculate move duration based on distance
        float distance = abs((int32_t)position - (int32_t)_currentPosition);
        _moveDuration = max(0.5f, distance / 100.0f); // 0.5-1.8 seconds depending on distance
    }
    
    _targetPosition = position;
}

uint16_t Finger::getTarget() const {
    return _targetPosition;
}

uint16_t Finger::getCurrentPosition() const {
    return _currentPosition;
}

float Finger::getFSR() const {
    return _fsrValue;
}

bool Finger::detectSlip() {
    return _slipDetected;
}

void Finger::setOffset(int16_t offset) {
    _offset = offset;
}

void Finger::reflexGrip(float forceBoost) {
    // Apply immediate force increase for reflex response
    uint16_t currentForce = _currentPosition;
    uint16_t maxForce = 180; // Maximum position (fully closed)
    
    // Calculate boost amount (percentage of remaining range)
    uint16_t boostAmount = (maxForce - currentForce) * forceBoost;
    
    // Apply immediate position change - overrides any motion profile
    _currentPosition = constrain(currentForce + boostAmount, 0, 180);
    _targetPosition = _currentPosition;
    
    // Immediately write position to servo
    writePosition(_currentPosition + _offset);
}

void Finger::setMotionProfile(MotionProfileType profile) {
    _motionProfile = profile;
}

void Finger::enableSoftContact(bool enable) {
    _softContactEnabled = enable;
}

void Finger::setSoftContactThreshold(float threshold) {
    _softContactThreshold = constrain(threshold, 0.1f, 0.9f);
}

void Finger::setPIDGains(float kp, float ki, float kd) {
    _pidKp = kp;
    _pidKi = ki;
    _pidKd = kd;
    _pidIntegral = 0.0f; // Reset integral term
}

void Finger::writePosition(uint16_t position) {
    // Ensure position is within valid range
    position = constrain(position, 0, 180);
    
    // Write to servo - implemention depends on your servo library
    analogWrite(_servoPin, map(position, 0, 180, 0, 255));
    // Or use Servo library with servo.write(position)
}

float Finger::calculateTrapezoidal(float t, float t_total) {
    // Implement trapezoidal profile
    // 25% accel, 50% constant velocity, 25% decel
    
    const float t_accel = t_total * 0.25f;
    const float t_decel = t_total * 0.25f;
    const float t_const = t_total - t_accel - t_decel;
    
    if (t < t_accel) {
        // Acceleration phase
        return 0.5f * (t / t_accel) * (t / t_accel);
    } else if (t < (t_accel + t_const)) {
        // Constant velocity phase
        return 0.5f + (t - t_accel) / t_total;
    } else if (t < t_total) {
        // Deceleration phase
        float td = (t - t_accel - t_const) / t_decel;
        return 1.0f - 0.5f * (1.0f - td) * (1.0f - td);
    } else {
        // Past end of movement
        return 1.0f;
    }
}

float Finger::calculateSCurve(float t, float t_total) {
    // Implement S-curve velocity profile using sine functions
    
    if (t <= 0) return 0.0f;
    if (t >= t_total) return 1.0f;
    
    // Use sinusoidal easing for smooth S-curve
    return (1.0f - cos(t / t_total * PI)) / 2.0f;
}

void Finger::updateSoftContact(float dt) {
    // Implement soft contact behavior
    // Slow down movement when approaching contact threshold
    
    if (_fsrValue >= _softContactThreshold) {
        // Calculate slow-down factor based on FSR pressure
        float pressure_factor = map(_fsrValue, _softContactThreshold, 1.0f, 1.0f, 0.1f);
        pressure_factor = constrain(pressure_factor, 0.1f, 1.0f);
        
        // If target is higher than current (closing), slow down movement
        if (_targetPosition > _currentPosition) {
            // Apply movement damping factor
            int32_t diff = _targetPosition - _currentPosition;
            uint16_t damped_diff = diff * pressure_factor * dt * 5.0f;
            _currentPosition += damped_diff;
        }
    }
}

void Finger::updatePID(float dt) {
    // Basic PID implementation
    float error = _targetPosition - _currentPosition;
    
    // Proportional term
    float p_term = _pidKp * error;
    
    // Integral term with anti-windup
    _pidIntegral += error * dt;
    _pidIntegral = constrain(_pidIntegral, -100.0f, 100.0f);
    float i_term = _pidKi * _pidIntegral;
    
    // Derivative term
    float error_rate = (error - _pidLastError) / dt;
    float d_term = _pidKd * error_rate;
    _pidLastError = error;
    
    // Calculate total command
    float command = p_term + i_term + d_term;
    
    // Convert command to position change
    int16_t position_change = command * dt * 10.0f;
    _currentPosition += position_change;
    _currentPosition = constrain(_currentPosition, 0, 180);
}