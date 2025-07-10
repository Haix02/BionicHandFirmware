/**
 * @file ReflexEngine.cpp
 * @brief Implementation of the high-speed reflex detection system
 * @version 3.0
 */

#include "ReflexEngine.h"

// Static instance for ISR callback
ReflexEngine* ReflexEngine::_instance = nullptr;

ReflexEngine::ReflexEngine() 
    : _fingers(nullptr), _enabled(false), 
      _slipThreshold(FSR_SLIP_DFDT_THRESH),
      _reflexCount(0), _totalResponseTime(0), 
      _lastSlipDetectTime(0), _avgResponseTime(0.0f)
{
    _instance = this; // Set singleton instance for ISR
    
    // Initialize FSR history
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
        _fsrHistoryIdx[i] = 0;
        for (uint8_t j = 0; j < FSR_DERIV_WINDOW; j++) {
            _fsrHistory[i][j] = 0.0f;
        }
    }
}

void ReflexEngine::begin(Finger** fingers) {
    _fingers = fingers;
    
    // Initialize and start high-frequency timer for reflex detection
    _reflexTimer.begin(reflexISR, 1000); // 1000us = 1ms = 1kHz
    
    _enabled = true;
}

void ReflexEngine::enable(bool enabled) {
    _enabled = enabled;
}

void ReflexEngine::setSlipThreshold(float threshold) {
    // Ensure threshold is negative (for slip detection)
    _slipThreshold = threshold < 0 ? threshold : -threshold;
}

void ReflexEngine::getReflexStats(uint32_t* reflexCount, float* avgResponseTime) {
    *reflexCount = _reflexCount;
    *avgResponseTime = _avgResponseTime;
}

void ReflexEngine::update() {
    // Update called from main loop
    // Mainly for monitoring, actual reflex happens in ISR
}

void ReflexEngine::reflexISR() {
    // Static ISR that calls instance method
    if (_instance && _instance->_enabled) {
        _instance->processFSRReadings();
    }
}

void ReflexEngine::processFSRReadings() {
    // High-speed FSR processing for slip detection
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
        // Read latest FSR value directly (bypass class interface for speed)
        float fsrValue = analogRead(FSR_PINS[i]) / 1024.0f;
        
        // Store in history buffer
        uint8_t idx = _fsrHistoryIdx[i];
        _fsrHistory[i][idx] = fsrValue;
        _fsrHistoryIdx[i] = (idx + 1) % FSR_DERIV_WINDOW;
        
        // Check for slip
        float slipRate = 0.0f;
        if (detectSlip(i, &slipRate)) {
            triggerReflex(i, slipRate);
        }
    }
}

bool ReflexEngine::detectSlip(uint8_t fingerIdx, float* slipRate) {
    // Calculate force derivative (dF/dt)
    uint8_t currentIdx = _fsrHistoryIdx[fingerIdx];
    uint8_t oldestIdx = (currentIdx + 1) % FSR_DERIV_WINDOW;
    
    float currentForce = _fsrHistory[fingerIdx][currentIdx];
    float oldestForce = _fsrHistory[fingerIdx][oldestIdx];
    
    // Skip detection if force is too small (not gripping anything)
    if (currentForce < 0.05f) {
        return false;
    }
    
    // Calculate rate of change (negative = slip)
    float dt = FSR_DERIV_WINDOW * (1.0f / 1000.0f); // Time window in seconds
    *slipRate = (currentForce - oldestForce) / dt;
    
    // Detect slip when force decreases rapidly
    if (*slipRate < _slipThreshold) {
        _lastSlipDetectTime = micros();
        return true;
    }
    
    return false;
}

void ReflexEngine::triggerReflex(uint8_t fingerIdx, float slipRate) {
    // Neuromorphic-inspired immediate reflex response
    if (_fingers && _fingers[fingerIdx]) {
        // Access finger object and trigger reflexive grip
        _fingers[fingerIdx]->reflexGrip();
        
        // Update reflex statistics
        uint32_t responseTime = micros() - _lastSlipDetectTime;
        _reflexCount++;
        _totalResponseTime += responseTime;
        _avgResponseTime = (float)_totalResponseTime / _reflexCount;
    }
}