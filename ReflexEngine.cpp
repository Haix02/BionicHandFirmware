/**
 * @file ReflexEngine.cpp
 * @brief High-speed slip detection and reflex response
 * @version 2.0
 */

#include "ReflexEngine.h"

// Static instance pointer for ISR callback
ReflexEngine* ReflexEngine::_instance = nullptr;

ReflexEngine::ReflexEngine()
    : _fingers(nullptr),
      _numFingers(0),
      _fsrReadFunc(nullptr),
      _enabled(false),
      _slipThreshold(-0.2f),
      _forceBoost(0.2f),
      _lowPowerMode(false),
      _reflexCount(0),
      _lastReflexTime(0),
      _totalResponseTime(0),
      _avgResponseTime(0.0f)
{
    _instance = this;
    
    // Initialize FSR history
    for (uint8_t i = 0; i < MAX_FINGERS; i++) {
        for (uint8_t j = 0; j < FSR_HISTORY_SIZE; j++) {
            _fsrHistory[i][j] = 0.0f;
        }
        _fsrHistoryIndex[i] = 0;
        _slipDetected[i] = false;
        _reflexActive[i] = false;
        _slipStartTime[i] = 0;
    }
}

void ReflexEngine::begin(Finger** fingers, uint8_t numFingers) {
    _fingers = fingers;
    _numFingers = min(numFingers, (uint8_t)MAX_FINGERS);
    _enabled = true;
}

void ReflexEngine::setFSRReadFunction(FSRReadFunction func) {
    _fsrReadFunc = func;
}

void ReflexEngine::enable(bool enabled) {
    _enabled = enabled;
}

void ReflexEngine::setSlipThreshold(float threshold) {
    // Ensure threshold is negative (for slip detection)
    _slipThreshold = (threshold < 0) ? threshold : -threshold;
}

void ReflexEngine::setForceBoost(float boost) {
    _forceBoost = constrain(boost, 0.1f, 1.0f);
}

void ReflexEngine::setLowPowerMode(bool enabled) {
    _lowPowerMode = enabled;
}

void ReflexEngine::update() {
    // This is called at 1kHz from IntervalTimer
    if (!_enabled || !_fingers || !_fsrReadFunc) return;
    
    // Use less processing in low power mode
    if (_lowPowerMode && (micros() % 2 == 1)) return; // Skip every other call
    
    // Record start time for response measurement
    uint32_t startTime = micros();
    
    // Check each finger for slip
    for (uint8_t i = 0; i < _numFingers; i++) {
        // Skip if finger doesn't exist
        if (!_fingers[i]) continue;
        
        // Skip if a reflex was recently triggered on this finger
        if (_reflexActive[i]) {
            uint32_t elapsedTime = millis() - _lastReflexTime;
            if (elapsedTime >= REFLEX_COOLDOWN_MS) {
                _reflexActive[i] = false;
            } else {
                continue;
            }
        }
        
        // Read FSR value
        float fsrValue = _fsrReadFunc(i);
        