/**
 * @file Finger.cpp
 * @brief Implementation of finger control with reflex grip capability
 */

// Existing implementation
// ...

/**
 * @brief Implement reflexive grip response to counter object slippage
 * @details Temporarily increases grip force by a percentage of remaining range
 */
void Finger::reflexGrip() {
    // Record time of reflex (for potential debouncing)
    uint32_t currentTime = millis();
    
    // Don't allow multiple reflexes too close together (debouncing)
    if (currentTime - _lastReflexTime < REFLEX_DEBOUNCE_MS) {
        return;
    }
    _lastReflexTime = currentTime;
    
    // Calculate current grip and maximum possible positions
    uint16_t currentPos = getCurrentPosition();
    uint16_t maxPos = _maxPosition;
    
    // Calculate boost amount (percentage of remaining range)
    uint16_t remainingRange = (maxPos > currentPos) ? (maxPos - currentPos) : 0;
    uint16_t boostAmount = remainingRange * REFLEX_BOOST_PERCENT / 100;
    
    // Ensure minimum boost (even when close to max)
    if (boostAmount < REFLEX_MIN_BOOST) {
        boostAmount = REFLEX_MIN_BOOST;
    }
    
    // Calculate new position with safety limit
    uint16_t newPosition = min(currentPos + boostAmount, maxPos);
    
    // Store original target for later restoration if needed
    _preReflexTarget = _targetPosition;
    
    // Apply new position immediately
    _currentPosition = newPosition;
    
    // Update target if reflex requires more force than current target
    if (_targetPosition < newPosition) {
        _targetPosition = newPosition;
    }
    
    // Apply to servo/motor directly
    writeServo(_currentPosition);
    
    // Set reflex active flag
    _reflexActive = true;
    
    #ifdef DEBUG_REFLEX
    Serial.print(F("REFLEX: Finger "));
    Serial.print(_fingerIndex);
    Serial.print(F(" boosted from "));
    Serial.print(currentPos);
    Serial.print(F(" to "));
    Serial.println(newPosition);
    #endif
}

// Add to existing update() method:
void Finger::update(float dt) {
    // Existing update code...
    
    // Handle reflex timeout if active
    if (_reflexActive) {
        if (millis() - _lastReflexTime > REFLEX_DURATION_MS) {
            _reflexActive = false;
            
            // Gradually return to original target if needed
            if (_targetPosition > _preReflexTarget) {
                _targetPosition = _preReflexTarget;
            }
        }
    }
    
    // Rest of existing update code...
}