/**
 * @file ReflexEngine.cpp
 * @brief Implementation of high-speed reflex detection system
 * @version 1.0
 */

#include "ReflexEngine.h"

// Static instance pointer for ISR
ReflexEngine* ReflexEngine::_instance = nullptr;

ReflexEngine::ReflexEngine() 
    : _fingers(nullptr), 
      _numFingers(0),
      _enabled(false),
      _slipThreshold(-0.2f),
      _forceBoost(0.3f),
      _reflexCount(0),
      _totalResponseTime(0),
      _avgResponseTime(0.0f),
      _lastSlipDetectTime(0)
{
    _instance = this;
    
    // Initialize FSR history arrays
    for (uint8_t i = 0; i < MAX_FINGERS; i++) {
        _fsrHistoryIdx[i] = 0;
        for (uint8_t j = 0; j < FSR_HISTORY_LEN; j++) {
            _fsrHistory[i][j] = 0.0f;
        }
    }
}

void ReflexEngine::begin(Finger** fingers, uint8_t numFingers) {
    _fingers = fingers;
    _numFingers = min(numFingers, (uint8_t)MAX_FINGERS);
    
    // Start the high-frequency timer (1000 Hz = 1000 µs)
    _reflexTimer.begin(_reflexISRWrapper, 1000);
    _enabled = true;
}

void ReflexEngine::enable(bool enabled) {
    _enabled = enabled;
}

void ReflexEngine::setSlipThreshold(float threshold) {
    // Ensure threshold is negative (for slip detection)
    _slipThreshold = (threshold < 0) ? threshold : -threshold;
}

void ReflexEngine::setForceBoost(float boost) {
    _forceBoost = constrain(boost, 0.0f, 1.0f);
}

void ReflexEngine::getReflexStats(uint32_t* count, float* avgResponse) {
    if (count) *count = _reflexCount;
    if (avgResponse) *avgResponse = _avgResponseTime;
}

void ReflexEngine::update() {
    // This method is called from main loop
    // Nothing to do here as the main work happens in the ISR
    // Could add debug output or other non-time-critical functions
}

void ReflexEngine::_reflexISRWrapper() {
    // Static method called by IntervalTimer
    if (_instance && _instance->_enabled) {
        _instance->reflexISR();
    }
}

void ReflexEngine::reflexISR() {
    // This runs at 1kHz for rapid slip detection
    for (uint8_t i = 0; i < _numFingers; i++) {
        // Skip if finger doesn't exist
        if (!_fingers[i]) continue;
        
        // Get latest FSR reading
        float fsrVal = _fingers[i]->getFSR();
        
        // Store in history buffer (circular)
        _fsrHistory[i][_fsrHistoryIdx[i]] = fsrVal;
        _fsrHistoryIdx[i] = (_fsrHistoryIdx[i] + 1) % FSR_HISTORY_LEN;
        
        // Check for slip
        if (detectSlip(i)) {
            triggerReflex(i);
        }
    }
}

bool ReflexEngine::detectSlip(uint8_t fingerIdx, float* slipRate) {
    // Get newest and oldest FSR values
    uint8_t newest = (_fsrHistoryIdx[fingerIdx] == 0) ? 
                      FSR_HISTORY_LEN - 1 : _fsrHistoryIdx[fingerIdx] - 1;
    uint8_t oldest = _fsrHistoryIdx[fingerIdx];
    
    float newestVal = _fsrHistory[fingerIdx][newest];
    float oldestVal = _fsrHistory[fingerIdx][oldest];
    
    // Skip detection if FSR value is too low (not gripping anything)
    if (newestVal < 0.05f) {
        return false;
    }
    
    // Calculate rate of change (negative = slip)
    float dt = FSR_HISTORY_LEN / 1000.0f; // Time window in seconds (1kHz sampling)
    float rate = (newestVal - oldestVal) / dt;
    
    if (slipRate) *slipRate = rate;
    
    // Detect slip when force decreases rapidly
    if (rate < _slipThreshold) {
        _lastSlipDetectTime = micros();
        return true;
    }
    
    return false;
}

void ReflexEngine::triggerReflex(uint8_t fingerIdx) {
    if (_fingers[fingerIdx]) {
        // Call the reflexGrip method on the finger
        _fingers[fingerIdx]->reflexGrip(_forceBoost);
        
        // Update statistics
        uint32_t responseTime = micros() - _lastSlipDetectTime;
        _reflexCount++;
        _totalResponseTime += responseTime;
        _avgResponseTime = (float)_totalResponseTime / (float)_reflexCount / 1000.0f; // Convert to ms
    }
}

void ReflexEngine::triggerManualReflex(uint8_t fingerIdx) {
    if (fingerIdx < _numFingers && _fingers[fingerIdx]) {
        _fingers[fingerIdx]->reflexGrip(_forceBoost);
    }
}