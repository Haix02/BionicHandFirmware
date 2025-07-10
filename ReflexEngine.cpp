/**
 * @file ReflexEngine.cpp
 * @brief High-speed slip detection and reflexive grip
 * @version 2.0
 */

#include "ReflexEngine.h"

// Static instance pointer for ISR callback
ReflexEngine* ReflexEngine::_instance = nullptr;

ReflexEngine::ReflexEngine()
    : _fingers(nullptr),
      _numFingers(0),
      _fsrReadFunc(nullptr),
      _enabled(true),
      _slipThreshold(REFLEX_SLIP_THRESHOLD),
      _lowPowerMode(false),
      _reflexCount(0),
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
        _lastReflexTime[i] = 0;
        _slipDetected[i] = false;
    }
}

void ReflexEngine::begin(Finger** fingers, uint8_t numFingers) {
    _fingers = fingers;
    _numFingers = min(numFingers, (uint8_t)MAX_FINGERS);
}

void ReflexEngine::setFSRReadFunction(FSRReadFunction func) {
    _fsrReadFunc = func;
}

void ReflexEngine::enable(bool enabled) {
    _enabled = enabled;
}

void ReflexEngine::setSlipThreshold(float threshold) {
    // Ensure threshold is negative
    _slipThreshold = (threshold < 0) ? threshold : -threshold;
}

void ReflexEngine::setLowPowerMode(bool enabled) {
    _lowPowerMode = enabled;
}

void ReflexEngine::update() {
    // This is called from IntervalTimer at 1kHz
    if (!_enabled || !_fingers || !_fsrReadFunc) return;
    
    // In low power mode, skip some updates to save energy
    if (_lowPowerMode && (micros() % 4 == 0)) return;
    
    // Store start time for response measurement
    uint32_t startTime = micros();
    
    // Process each finger
    for (uint8_t i = 0; i < _numFingers; i++) {
        if (!_fingers[i]) continue;
        
        // Read FSR value
        float fsrValue = _fsrReadFunc(i);
        
        // Store in history buffer
        uint8_t histIdx = _fsrHistoryIndex[i];
        _fsrHistory[i][histIdx] = fsrValue;
        
        // Advance history index
        _fsrHistoryIndex[i] = (histIdx + 1) % FSR_HISTORY_SIZE;
        
        // Check for slip
        if (detectSlip(i)) {
            // Trigger reflex response
            triggerReflex(i);
        }
    }
}

bool ReflexEngine::detectSlip(uint8_t fingerIdx) {
    // Calculate time since last reflex
    uint32_t timeSinceReflex = millis() - _lastReflexTime[fingerIdx];
    
    // Skip if reflex was recently triggered
    if (timeSinceReflex < REFLEX_COOLDOWN_MS) {
        return false;
    }
    
    // Calculate derivative from history buffer
    uint8_t newestIdx = (_fsrHistoryIndex[fingerIdx] == 0) ? 
                        FSR_HISTORY_SIZE - 1 : 
                        _fsrHistoryIndex[fingerIdx] - 1;
                        
    uint8_t oldestIdx = _fsrHistoryIndex[fingerIdx];
    
    float newest = _fsrHistory[fingerIdx][newestIdx];
    float oldest = _fsrHistory[fingerIdx][oldestIdx];
    
    // Skip if contact force is too low
    if (newest < REFLEX_MIN_FORCE) {
        return false;
    }
    
    // Calculate rate of change (negative = slip)
    float timeWindow = FSR_HISTORY_SIZE / 1000.0f; // Window in seconds (assuming 1kHz)
    float derivative = (newest - oldest) / timeWindow;
    
    // Detect slip (negative derivative beyond threshold)
    if (derivative < _slipThreshold) {
        _slipDetected[fingerIdx] = true;
        return true;
    }
    
    return false;
}

void ReflexEngine::triggerReflex(uint8_t fingerIdx) {
    if (!_fingers[fingerIdx]) return;
    
    // Record reflex time
    uint32_t now = millis();
    _lastReflexTime[fingerIdx] = now;
    
    // Calculate response time
    uint32_t responseTime = micros() - _slipStartTime;
    
    // Update statistics
    _reflexCount++;
    _totalResponseTime += responseTime;
    _avgResponseTime = (float)_totalResponseTime / _reflexCount / 1000.0f; // In ms
    
    // Trigger reflex grip
    _fingers[fingerIdx]->reflexGrip();
    
    #ifdef DEBUG_REFLEX
    Serial.print(F("Slip detected on finger "));
    Serial.print(fingerIdx);
    Serial.print(F(", response time: "));
    Serial.print(responseTime / 1000.0f, 2);
    Serial.println(F(" ms"));
    #endif
}

void ReflexEngine::triggerManualReflex(uint8_t fingerIdx) {
    if (fingerIdx >= _numFingers || !_fingers[fingerIdx]) return;
    
    // Trigger reflex manually
    _fingers[fingerIdx]->reflexGrip();
    
    // Record for statistics
    _reflexCount++;
    _lastReflexTime[fingerIdx] = millis();
    
    Serial.print(F("Manual reflex triggered on finger "));
    Serial.println(fingerIdx);
}

void ReflexEngine::getReflexStats(uint32_t* count, float* avgTime) const {
    if (count) *count = _reflexCount;
    if (avgTime) *avgTime = _avgResponseTime;
}