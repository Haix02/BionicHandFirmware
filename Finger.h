/**
 * @file Finger.h
 * @brief Finger class definition with added reflex grip functionality
 */

#pragma once

#include <Arduino.h>

class Finger {
public:
  // Existing declarations (unchanged)
  Finger(uint8_t servoPin, uint8_t fsrPin);
  void begin();
  void update();
  void setTarget(uint16_t position);
  uint16_t getTarget() const;
  uint16_t getCurrentPosition() const;
  float getFSR() const;
  
  /**
   * @brief Trigger reflexive grip response to prevent slip
   * @details Temporarily increases grip force/angle by a fixed amount
   */
  void reflexGrip();
  
  // Other existing methods (unchanged)
  
private:
  // Existing members (unchanged)
  uint8_t _servoPin;
  uint8_t _fsrPin;
  uint16_t _currentPosition;
  uint16_t _targetPosition;
  uint16_t _maxPosition;
  bool _isMoving;
  uint32_t _lastReflexTime; // New: track last reflex activation time
  
  // Other existing private members and methods (unchanged)
};