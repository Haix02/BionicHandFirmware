/**
 * @file Finger.cpp
 * @brief Enhanced finger actuator implementation
 * @version 3.0
 */

#include "Finger.h"
#include <PID_v1.h> // PID Library for Teensy

Finger::Finger(uint8_t pwmPin, uint8_t fsrPin)
    : _pwmPin(pwmPin), _fsrPin(fsrPin), _offset(0), _currentPosition(0),
      _targetPosition(0), _pidErrorInt(0.0f), _pidLastError(0.0f),
      _pidKp(FINGER_PID_KP), _pidKi(FINGER_PID_KI), _pidKd(FINGER_PID_KD),
      _fsrVal(0), _fsrHistIdx(0), _fsrDFdt(0), _slipDetected(false),
      _profileType(MotionProfileType::None), 
      _maxVelocity(400.0), _maxAcceleration(800.0),
      _profileStartTime(0), _profileStartPos(0), _motionProfileActive(false),
      _softContactEnabled(true), _softContactThreshold(0.3), _currentVelocity(0),
      _state(FingerState::Idle)
{
    memset(_fsrHist, 0, sizeof(_fsrHist));
    
    // Initialize PID controller
    _pidController = new PID(&_currentPosition, &_currentVelocity, &_targetPosition, 
                            _pidKp, _pidKi, _pidKd, DIRECT);
}

void Finger::begin() {
    pinMode(_pwmPin, OUTPUT);
    pinMode(_fsrPin, INPUT);
    
    // Configure PID controller
    _pidController->SetMode(AUTOMATIC);
    _pidController->SetOutputLimits(-_maxVelocity, _maxVelocity);
    _pidController->SetSampleTime(10); // 10ms update rate
    
    // Initialize finger position
    actuate(_currentPosition);
}

void Finger::update(float dt_ms) {
    // V3: Enhanced update with motion profiling and soft contact
    float dt_sec = dt_ms / 1000.0f;
    
    if (_motionProfileActive) {
        // Calculate elapsed time since motion started
        float elapsedTime = (millis() - _profileStartTime) / 1000.0f;
        
        // Calculate new position based on profile type
        uint16_t newPos = _currentPosition;
        
        switch (_profileType) {
            case MotionProfileType::Trapezoidal:
                newPos = calculateTrapezoidal(elapsedTime, _profileStartPos, 
                                            _targetPosition, _maxVelocity, _maxAcceleration);
                break;
                
            case MotionProfileType::SCurve:
                newPos = calculateSCurve(elapsedTime, _profileStartPos, 
                                       _targetPosition, _maxVelocity, _maxAcceleration);
                break;
                
            default:
                // Direct positioning (legacy behavior)
                float error = _targetPosition - _currentPosition;
                float command = _pidKp * error + _pidKi * _pidErrorInt + _pidKd * (_pidLastError - error) / dt_ms;
                _pidLastError = error;
                _pidErrorInt += error * dt_sec;
                
                // Apply position update
                newPos = _currentPosition + command * dt_sec;
                break;
        }
        
        // Soft contact adaptation - slow down when approaching contact
        if (_softContactEnabled && _fsrVal > _softContactThreshold) {
            // Calculate velocity scaling based on force
            float velocityScale = calculateVelocityScale();
            
            // Apply velocity scaling
            float delta = newPos - _currentPosition;
            newPos = _currentPosition + delta * velocityScale;
            
            // If stable contact reached, stop motion profile
            if (_fsrVal > 0.7f) {
                _motionProfileActive = false;
                setState(FingerState::Gripping);
            }
        }
        
        // Update position
        _currentPosition = constrain(newPos, FINGER_POS_MIN, FINGER_POS_MAX);
        
        // Check if motion profile is complete
        if (abs(_currentPosition - _targetPosition) < 2) {
            _motionProfileActive = false;
            setState(FingerState::Idle);
        }
    }
    else {
        // If no active profile, use PID for position maintenance
        _pidController->Compute();
        _currentPosition += _currentVelocity * dt_sec;
        _currentPosition = constrain(_currentPosition, FINGER_POS_MIN, FINGER_POS_MAX);
    }
    
    // Apply position offset and send to actuator
    uint16_t actuatorPos = _currentPosition + _offset;
    actuate(actuatorPos);
}

float Finger::calculateTrapezoidal(float t, float startPos, float endPos, float maxVel, float maxAccel) {
    float dist = endPos - startPos;
    float dir = dist > 0 ? 1.0f : -1.0f;
    dist = abs(dist);
    
    // Time to reach maximum velocity
    float t_accel = maxVel / maxAccel;
    
    // Distance covered during acceleration/deceleration
    float d_accel = 0.5f * maxAccel * t_accel * t_accel;
    
    // Check if we can reach maximum velocity
    if (2 * d_accel < dist) {
        // Trapezoidal profile (accel, cruise, decel)
        float t_cruise = (dist - 2 * d_accel) / maxVel;
        float t_total = 2 * t_accel + t_cruise;
        
        if (t < t_accel) {
            // Acceleration phase
            return startPos + dir * 0.5f * maxAccel * t * t;
        }
        else if (t < t_accel + t_cruise) {
            // Cruise phase
            return startPos + dir * (d_accel + maxVel * (t - t_accel));
        }
        else if (t < t_total) {
            // Deceleration phase
            float t_decel = t - (t_accel + t_cruise);
            return startPos + dir * (dist - 0.5f * maxAccel * (t_total - t) * (t_total - t));
        }
        else {
            // Motion complete
            return endPos;
        }
    }
    else {
        // Triangular profile (no cruise phase)
        // Recalculate maximum velocity
        maxVel = sqrt(maxAccel * dist);
        t_accel = maxVel / maxAccel;
        float t_total = 2 * t_accel;
        
        if (t < t_accel) {
            // Acceleration phase
            return startPos + dir * 0.5f * maxAccel * t * t;
        }
        else if (t < t_total) {
            // Deceleration phase
            return startPos + dir * (dist - 0.5f * maxAccel * (t_total - t) * (t_total - t));
        }
        else {
            // Motion complete
            return endPos;
        }
    }
}

float Finger::calculateSCurve(float t, float startPos, float endPos, float maxVel, float maxAccel) {
    float dist = endPos - startPos;
    float dir = dist > 0 ? 1.0f : -1.0f;
    dist = abs(dist);
    
    // For S-curve, we need jerk limiting
    // Simplified S-curve implementation
    float jerk = maxAccel * 2.0f; // Jerk parameter
    float t_jerk = maxAccel / jerk;
    float t_accel = maxVel / maxAccel + t_jerk;
    
    // Calculate total distance and time
    float d_accel = maxVel * t_accel - 0.5f * maxVel * t_jerk;
    
    if (2 * d_accel < dist) {
        // S-curve with cruise phase
        float t_cruise = (dist - 2 * d_accel) / maxVel;
        float t_total = 2 * t_accel + t_cruise;
        
        if (t < t_jerk) {
            // Initial jerk-limited acceleration
            return startPos + dir * (jerk * t * t * t / 6.0f);
        }
        else if (t < t_accel - t_jerk) {
            // Constant acceleration
            return startPos + dir * (0.5f * maxAccel * (t - t_jerk/2) * (t - t_jerk/2) + jerk * t_jerk * t_jerk * t_jerk / 6.0f);
        }
        else if (t < t_accel) {
            // Final jerk-limited acceleration
            float t_phase = t - (t_accel - t_jerk);
            return startPos + dir * (d_accel - maxVel * t_jerk / 2.0f + maxVel * t_phase - jerk * (t_jerk - t_phase) * (t_jerk - t_phase) * (t_jerk - t_phase) / 6.0f);
        }
        else if (t < t_accel + t_cruise) {
            // Cruise phase
            return startPos + dir * (d_accel + maxVel * (t - t_accel));
        }
        else if (t < t_total) {
            // Deceleration (mirror of acceleration)
            float t_decel = t - (t_accel + t_cruise);
            float t_remaining = t_total - t;
            
            // Mirror acceleration calculations for deceleration
            if (t_decel < t_jerk) {
                // Initial jerk-limited deceleration
                return endPos - dir * (jerk * t_decel * t_decel * t_decel / 6.0f);
            }
            else if (t_decel < t_accel - t_jerk) {
                // Constant deceleration
                return endPos - dir * (0.5f * maxAccel * (t_decel - t_jerk/2) * (t_decel - t_jerk/2) + jerk * t_jerk * t_jerk * t_jerk / 6.0f);
            }
            else {
                // Final jerk-limited deceleration
                float t_phase = t_decel - (t_accel - t_jerk);
                return endPos - dir * (d_accel - maxVel * t_jerk / 2.0f + maxVel * t_phase - jerk * (t_jerk - t_phase) * (t_jerk - t_phase) * (t_jerk - t_phase) / 6.0f);
            }
        }
        else {
            // Motion complete
            return endPos;
        }
    }
    else {
        // Simplified case for short movements
        // Use a simpler curve for small movements
        return startPos + dir * dist * (1 - cos(M_PI * t / (2 * t_accel))) / 2.0f;
    }
}

float Finger::calculateVelocityScale() {
    if (!_softContactEnabled || _fsrVal < _softContactThreshold) {
        return 1.0f;
    }
    
    // Scale velocity based on force feedback
    float forceRange = 0.7f - _softContactThreshold;
    float normalizedForce = (_fsrVal - _softContactThreshold) / forceRange;
    normalizedForce = constrain(normalizedForce, 0.0f, 1.0f);
    
    // Exponential slowdown for natural soft landing
    // 1.0 -> 0.2 as force increases
    return 1.0f - 0.8f * normalizedForce;
}

void Finger::setTarget(uint16_t position) {
    _targetPosition = constrain(position, FINGER_POS_MIN, FINGER_POS_MAX);
    setState(FingerState::Moving);
    
    // V3: Initialize motion profile
    if (_profileType != MotionProfileType::None) {
        _profileStartTime = millis();
        _profileStartPos = _currentPosition;
        _motionProfileActive = true;
    }
}

uint16_t Finger::getTarget() const {
    return _targetPosition;
}

uint16_t Finger::getCurrentPosition() const {
    return _currentPosition;
}

float Finger::getFSR() const {
    return _fsrVal;
}

void Finger::sampleFSR() {
    // Called at 100Hz from ISR
    _fsrVal = analogRead(_fsrPin) / 1024.0f;

    // FSR slip detection history
    _fsrHist[_fsrHistIdx] = _fsrVal;
    _fsrHistIdx = (_fsrHistIdx + 1) % FSR_DERIV_WINDOW;

    updateSlipDetection();
}

void Finger::updateSlipDetection() {
    // Compute dF/dt over the history window
    uint8_t oldest = (_fsrHistIdx + 1) % FSR_DERIV_WINDOW;
    float df = _fsrVal - _fsrHist[oldest];
    float dt = (float)FSR_DERIV_WINDOW * (1000.0f / FSR_SAMPLE_HZ); // ms
    _fsrDFdt = df / (dt / 1000.0f); // per second

    _slipDetected = (_fsrDFdt < FSR_SLIP_DFDT_THRESH);
    if (_slipDetected)
        _state = FingerState::Slipping;
}

bool Finger::detectSlip() {
    return _slipDetected;
}

void Finger::reflexGrip() {
    // V3: Enhanced reflex with direct motor control
    // This bypasses motion profiling for immediate response
    
    // Calculate immediate increase in position
    uint16_t reflexPos = _currentPosition + 10; // +10 units immediate increase
    reflexPos = constrain(reflexPos, _currentPosition, FINGER_POS_MAX);
    
    // Directly actuate without motion profile
    _currentPosition = reflexPos;
    actuate(_currentPosition + _offset);
    
    // Update state
    _state = FingerState::Gripping;
    
    // Reset slip detection
    _slipDetected = false;
}

void Finger::setMotionProfile(MotionProfileType type) {
    _profileType = type;
}

void Finger::setVelocityLimits(float maxVel, float maxAccel) {
    _maxVelocity = maxVel;
    _maxAcceleration = maxAccel;
    
    // Update PID controller limits
    if (_pidController) {
        _pidController->SetOutputLimits(-_maxVelocity, _maxVelocity);
    }
}

void Finger::enableSoftContact(bool enable) {
    _softContactEnabled = enable;
}

void Finger::setSoftContactThreshold(float threshold) {
    _softContactThreshold = constrain(threshold, 0.1f, 0.8f);
}

void Finger::setPIDGains(float kp, float ki, float kd) {
    _pidKp = kp;
    _pidKi = ki;
    _pidKd = kd;
    
    if (_pidController) {
        _pidController->SetTunings(kp, ki, kd);
    }
}

void Finger::setOffset(int16_t offset) {
    _offset = offset;
}

FingerState Finger::getState() const {
    return _state;
}

void Finger::setState(FingerState st) {
    _state = st;
}

void Finger::actuate(uint16_t pwm_val) {
    analogWrite(_pwmPin, pwm_val); // Adapt for actuator type
}

void Finger::printDebug(Stream &s) {
    s.print(F("FSR:")); s.print(_fsrVal, 3); s.print(F(" dF/dt:")); s.print(_fsrDFdt, 3);
    s.print(F(" Pos:")); s.print(_currentPosition);
    s.print(F(" Target:")); s.print(_targetPosition);
    s.print(F(" Profile:")); s.print((int)_profileType);
    s.print(F(" State:")); s.print((int)_state);
}

void Finger::resetMotionProfile() {
    _motionProfileActive = false;
}