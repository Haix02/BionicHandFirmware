/**
 * @file GraspManager.cpp
 * @brief Multi-finger grasp pattern coordination
 * @version 2.0
 */

#include "GraspManager.h"

GraspManager::GraspManager()
    : _fingers(nullptr),
      _numFingers(0),
      _currentGraspMode("open"),
      _graspForce(1.0f)
{
}

void GraspManager::begin(Finger** fingers, uint8_t numFingers) {
    _fingers = fingers;
    _numFingers = numFingers;
}

void GraspManager::update(float dt) {
    // Continuous updates if needed
    // This could adapt grasp based on sensor input
    // Currently empty as grasps are event-driven
}

void GraspManager::executeGrasp(String mode) {
    // Convert to lowercase for case-insensitive comparison
    mode.toLowerCase();
    _currentGraspMode = mode;
    
    Serial.print(F("Executing grasp: "));
    Serial.println(mode);
    
    // Execute appropriate grasp pattern
    if (mode == "open") {
        executeOpenHand();
    }
    else if (mode == "close") {
        executeCloseHand();
    }
    else if (mode == "power") {
        executePowerGrip();
    }
    else if (mode == "precision") {
        executePrecisionGrip();
    }
    else if (mode == "spherical") {
        executeSphericalGrip();
    }
    else if (mode == "tripod") {
        executeTripodGrip();
    }
    else if (mode == "lateral") {
        executeLateralGrip();
    }
    else if (mode == "hook") {
        executeHookGrip();
    }
    else if (mode == "point") {
        executePointingGesture();
    }
    else {
        // Unknown grip type - default to open
        Serial.println(F("Unknown grasp type, defaulting to open"));
        executeOpenHand();
    }
}

void GraspManager::executeOpenHand() {
    if (!_fingers || _numFingers == 0) return;
    
    // Set all fingers to minimum position (open)
    for (uint8_t i = 0; i < _numFingers; i++) {
        _fingers[i]->setTarget(FINGER_POS_MIN);
    }
}

void GraspManager::executeCloseHand() {
    if (!_fingers || _numFingers == 0) return;
    
    // Calculate force-scaled position
    uint16_t targetPos = FINGER_POS_MIN + 
                         (FINGER_POS_MAX - FINGER_POS_MIN) * _graspForce;
    
    // Set all fingers to closed position
    for (uint8_t i = 0; i < _numFingers; i++) {
        _fingers[i]->setTarget(targetPos);
    }
}

void GraspManager::executePowerGrip() {
    if (!_fingers || _numFingers == 0) return;
    
    // Power grip: strong full closure of all fingers
    // Thumb position (typically opposed to fingers)
    uint16_t thumbPos = FINGER_POS_MIN + 
                       (FINGER_POS_MAX - FINGER_POS_MIN) * 0.9f * _graspForce;
    
    // Other fingers - fully closed
    uint16_t fingerPos = FINGER_POS_MIN + 
                        (FINGER_POS_MAX - FINGER_POS_MIN) * _graspForce;
    
    // Apply positions
    // Assuming finger 0 is thumb
    if (_numFingers > 0) _fingers[0]->setTarget(thumbPos);
    
    // Remaining fingers
    for (uint8_t i = 1; i < _numFingers; i++) {
        _fingers[i]->setTarget(fingerPos);
    }
}

void GraspManager::executePrecisionGrip() {
    if (!_fingers || _numFingers < 2) return;
    
    // Precision grip: thumb and index finger only
    uint16_t range = FINGER_POS_MAX - FINGER_POS_MIN;
    
    // Thumb and index finger at ~70% closure
    uint16_t thumbIndexPos = FINGER_POS_MIN + range * 0.7f * _graspForce;
    
    // Other fingers curled out of the way
    uint16_t otherFingerPos = FINGER_POS_MIN + range * 0.9f * _graspForce;
    
    // Apply positions
    // Assuming finger 0 is thumb, finger 1 is index
    _fingers[0]->setTarget(thumbIndexPos);  // Thumb
    _fingers[1]->setTarget(thumbIndexPos);  // Index
    
    // Curl remaining fingers
    for (uint8_t i = 2; i < _numFingers; i++) {
        _fingers[i]->setTarget(otherFingerPos);
    }
}

void GraspManager::executeSphericalGrip() {
    if (!_fingers || _numFingers == 0) return;
    
    // Spherical grip: curved hand shape for holding round objects
    uint16_t range = FINGER_POS_MAX - FINGER_POS_MIN;
    
    // Target positions for spherical grip - ~60% closed
    uint16_t midPos = FINGER_POS_MIN + range * 0.6f * _graspForce;
    
    // Set all fingers to mid position
    for (uint8_t i = 0; i < _numFingers; i++) {
        _fingers[i]->setTarget(midPos);
    }
}

void GraspManager::executeTripodGrip() {
    if (!_fingers || _numFingers < 3) return;
    
    // Tripod grip: thumb, index, middle form three-point contact
    uint16_t range = FINGER_POS_MAX - FINGER_POS_MIN;
    
    // Tripod fingers at ~70% closure
    uint16_t tripodPos = FINGER_POS_MIN + range * 0.7f * _graspForce;
    
    // Other fingers curled out of the way
    uint16_t otherFingerPos = FINGER_POS_MIN + range * 0.9f * _graspForce;
    
    // Apply positions
    // Assuming finger 0 is thumb, 1 is index, 2 is middle
    _fingers[0]->setTarget(tripodPos);  // Thumb
    _fingers[1]->setTarget(tripodPos);  // Index
    _fingers[2]->setTarget(tripodPos);  // Middle
    
    // Curl remaining fingers
    for (uint8_t i = 3; i < _numFingers; i++) {
        _fingers[i]->setTarget(otherFingerPos);
    }
}

void GraspManager::executeLateralGrip() {
    if (!_fingers || _numFingers < 2) return;
    
    // Lateral grip (key grip): thumb presses against side of index
    uint16_t range = FINGER_POS_MAX - FINGER_POS_MIN;
    
    // Thumb at medium-high position
    uint16_t thumbPos = FINGER_POS_MIN + range * 0.7f * _graspForce;
    
    // Index at medium position
    uint16_t indexPos = FINGER_POS_MIN + range * 0.5f * _graspForce;
    
    // Other fingers curled
    uint16_t otherFingerPos = FINGER_POS_MIN + range * 0.85f * _graspForce;
    
    // Apply positions
    _fingers[0]->setTarget(thumbPos);  // Thumb
    _fingers[1]->setTarget(indexPos);  // Index
    
    // Curl remaining fingers
    for (uint8_t i = 2; i < _numFingers; i++) {
        _fingers[i]->setTarget(otherFingerPos);
    }
}

void GraspManager::executeHookGrip() {
    if (!_fingers || _numFingers == 0) return;
    
    // Hook grip: fingers curled, thumb relaxed
    uint16_t range = FINGER_POS_MAX - FINGER_POS_MIN;
    
    // Thumb stays open
    uint16_t thumbPos = FINGER_POS_MIN + range * 0.2f * _graspForce;
    
    // Other fingers fully curled
    uint16_t fingerPos = FINGER_POS_MIN + range * 0.95f * _graspForce;
    
    // Apply positions
    if (_numFingers > 0) _fingers[0]->setTarget(thumbPos);  // Thumb
    
    // Curl other fingers
    for (uint8_t i = 1; i < _numFingers; i++) {
        _fingers[i]->setTarget(fingerPos);
    }
}

void GraspManager::executePointingGesture() {
    if (!_fingers || _numFingers < 2) return;
    
    // Pointing gesture: index extended, others curled
    uint16_t range = FINGER_POS_MAX - FINGER_POS_MIN;
    
    // Index extended
    uint16_t indexPos = FINGER_POS_MIN;
    
    // Other fingers curled
    uint16_t otherFingerPos = FINGER_POS_MIN + range * 0.9f * _graspForce;
    
    // Apply positions
    for (uint8_t i = 0; i < _numFingers; i++) {
        if (i == 1) { // Index finger
            _fingers[i]->setTarget(indexPos);
        } else {
            _fingers[i]->setTarget(otherFingerPos);
        }
    }
}

void GraspManager::setGraspForce(float force) {
    // Constrain force to range [0, 1]
    _graspForce = constrain(force, 0.0f, 1.0f);
    
    // Re-apply current grasp with new force
    executeGrasp(_currentGraspMode);
}

float GraspManager::getGraspForce() const {
    return _graspForce;
}

String GraspManager::getCurrentGraspMode() const {
    return _currentGraspMode;
}