/**
 * @file Finger.cpp
 * @brief Finger class implementation with reflexGrip method
 */

#include "Finger.h"

// Existing Finger implementation (unchanged)

/**
 * @brief Trigger reflexive grip response to prevent slip
 * @details Temporarily increases grip force/angle by a fixed amount
 */
void Finger::reflexGrip() {
  // Don't allow reflexes too frequently (debounce)
  if (millis() - _lastReflexTime < 300) {
    return;
  }
  
  // Record reflex time
  _lastReflexTime = millis();
  
  // Calculate reflex boost amount (15% of remaining range)
  uint16_t currentPos = getCurrentPosition();
  uint16_t remainingRange = _maxPosition - currentPos;
  uint16_t boostAmount = remainingRange * 0.15;
  
  // Ensure minimum boost even when near limit
  if (boostAmount < 5) {
    boostAmount = 5;
  }
  
  // Calculate new position with safety limit
  uint16_t reflexPosition = min(currentPos + boostAmount, _maxPosition);
  
  // Apply the new position immediately
  _currentPosition = reflexPosition;
  
  // Also update target to match (prevents immediate movement back)
  if (_targetPosition < _currentPosition) {
    _targetPosition = _currentPosition;
  }
  
  // Directly apply new position to servo
  analogWrite(_servoPin, map(_currentPosition, 0, 180, 0, 255));
  // Note: Adjust mapping values based on your servo configuration
  
  // Could also use: servo.write(_currentPosition);
}