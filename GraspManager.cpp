/**
 * @file GraspManager.cpp
 * @brief Coordinates multi-finger grasp patterns
 * @version 2.0
 */

#include "GraspManager.h"

GraspManager::GraspManager()
    : _fingers(nullptr),
      _numFingers(0),
      _currentGraspMode("open"),
      _graspForce(1.0f),
      _transitionSpeed(1.0f)
{
    // Constructor implementation
}

void GraspManager::begin(Finger** fingers, uint8_t numFingers) {
    _fingers = fingers;
    _numFingers = numFingers;
}

void GraspManager::update() {
    // This method can be called in the main loop
    // Add any continuous grasp adjustments here
    
    // Example: Maintain current grasp mode
    // This could be expanded to react to sensor input
}

void GraspManager::executeGrasp(String mode) {
    // Normalize mode string
    mode.toLowerCase();
    _currentGraspMode = mode;
    
    if (mode == "open") {
        executeOpenHand();
    }
    else if (mode == "close") {
        executeCloseHand();
    }
    else if (mode == "power") {
        executePowerGrasp();
    }
    else if (mode == "precision") {
        executePrecisionGrasp();
    }
    else if (mode == "spherical") {
        executeSphericalGrasp();
    }
    else if (mode == "tripod") {
        executeTripodGrasp();
    }
    else if (mode == "lateral") {
        executeLateralGrasp();
    }
    else if (mode == "hook") {
        executeHookGrasp();
    }
    else if (mode == "point") {
        executePointGesture();
    }
    else if (mode == "thumbs_up") {
        executeThumbsUpGesture();
    }
    else {
        // Unknown grasp mode - default to open
        executeOpenHand();
    }
    
    Serial.print(F("Executing grasp: "));
    Serial.println(mode);
}

void GraspManager::executeOpenHand() {
    if (!_fingers) return;
    
    // Set all fingers to minimum position
    for (uint8_t i = 0; i < _numFingers; i++) {
        _fingers[i]->setTarget(FINGER_POS_MIN);
    }
}

void GraspManager::executeCloseHand() {
    if (!_fingers) return;
    
    // Scale max position by grip force
    uint16_t targetPos = FINGER_POS_MIN + 
                         (FINGER_POS_MAX - FINGER_POS_MIN) * _graspForce;
    
    // Set all fingers to maximum position
    for (uint8_t i = 0; i < _numFingers; i++) {
        _fingers[i]->setTarget(targetPos);
    }
}

void GraspManager::executePowerGrasp() {
    if (!_fingers || _numFingers < 4) return;
    
    // Power grasp: all fingers curl around object
    // Different positions for anatomically correct hand pose
    
    // Scale positions by grip force
    uint16_t range = FINGER_POS_MAX - FINGER_POS_MIN;
    
    // Thumb position (typically 90% flexion in power grasp)
    uint16_t thumbPos = FINGER_POS_MIN + range * 0.9f * _graspForce;
    
    // Other fingers progressively more flexed
    uint16_t indexPos = FINGER_POS_MIN + range * 0.85f * _graspForce;
    uint16_t middlePos = FINGER_POS_MIN + range * 0.95f * _graspForce;
    uint16_t ringPos = FINGER_POS_MIN + range * 0.95f * _graspForce;
    
    // Set finger positions
    _fingers[0]->setTarget(thumbPos);    // Thumb
    _fingers[1]->setTarget(indexPos);    // Index
    _fingers[2]->setTarget(middlePos);   // Middle
    _fingers[3]->setTarget(ringPos);     // Ring/pinky
}

void GraspManager::executePrecisionGrasp() {
    if (!_fingers || _numFingers < 3) return;
    
    // Precision grasp: thumb and index finger form pincer
    // Other fingers curl out of the way
    
    uint16_t range = FINGER_POS_MAX - FINGER_POS_MIN;
    
    // Thumb and index at ~60% flexion
    uint16_t thumbPos = FINGER_POS_MIN + range * 0.6f * _graspForce;
    uint16_t indexPos = FINGER_POS_MIN + range * 0.6f * _graspForce;
    
    // Other fingers curl out of the way
    uint16_t middlePos = FINGER_POS_MIN + range * 0.9f * _graspForce;
    uint16_t ringPos = FINGER_POS_MIN + range * 0.95f * _graspForce;
    
    // Set finger positions
    _fingers[0]->setTarget(thumbPos);    // Thumb
    _fingers[1]->setTarget(indexPos);    // Index
    _fingers[2]->setTarget(middlePos);   // Middle
    if (_numFingers > 3) {
        _fingers[3]->setTarget(ringPos);     // Ring/pinky
    }
}

void GraspManager::executeSphericalGrasp() {
    if (!_fingers || _numFingers < 4) return;
    
    // Spherical grasp: fingers form hemisphere around object
    // Each finger at a different flexion angle
    
    uint16_t range = FINGER_POS_MAX - FINGER_POS_MIN;
    
    // Different flex angles for spherical grasp
    uint16_t thumbPos = FINGER_POS_MIN + range * 0.7f * _graspForce;
    uint16_t indexPos = FINGER_POS_MIN + range * 0.6f * _graspForce;
    uint16_t middlePos = FINGER_POS_MIN + range * 0.7f * _graspForce;
    uint16_t ringPos = FINGER_POS_MIN + range * 0.8f * _graspForce;
    
    // Set finger positions
    _fingers[0]->setTarget(thumbPos);    // Thumb
    _fingers[1]->setTarget(indexPos);    // Index
    _fingers[2]->setTarget(middlePos);   // Middle
    _fingers[3]->setTarget(ringPos);     // Ring/pinky
}

void GraspManager::executeTripodGrasp() {
    if (!_fingers || _numFingers < 3) return;
    
    // Tripod grasp: thumb, index, middle form three-point contact
    // Ring/pinky curl out of the way
    
    uint16_t range = FINGER_POS_MAX - FINGER_POS_MIN;
    
    // Thumb, index and middle at medium flexion
    uint16_t thumbPos = FINGER_POS_MIN + range * 0.65f * _graspForce;
    uint16_t indexPos = FINGER_POS_MIN + range * 0.65f * _graspForce;
    uint16_t middlePos = FINGER_POS_MIN + range * 0.65f * _graspForce;
    uint16_t ringPos = FINGER_POS_MIN + range * 0.9f * _graspForce;
    
    // Set finger positions
    _fingers[0]->setTarget(thumbPos);    // Thumb
    _fingers[1]->setTarget(indexPos);    // Index
    _fingers[2]->setTarget(middlePos);   // Middle
    if (_numFingers > 3) {
        _fingers[3]->setTarget(ringPos);     // Ring/pinky
    }
}

void GraspManager::executeLateralGrasp() {
    if (!_fingers || _numFingers < 2) return;
    
    // Lateral/key grip: thumb presses against side of index
    // Other fingers curl to different positions
    
    uint16_t range = FINGER_POS_MAX - FINGER_POS_MIN;
    
    // Thumb at medium-high flexion
    uint16_t thumbPos = FINGER_POS_MIN + range * 0.7f * _graspForce;
    
    // Index only partially flexed
    uint16_t indexPos = FINGER_POS_MIN + range * 0.5f * _graspForce;
    
    // Other fingers out of the way
    uint16_t middlePos = FINGER_POS_MIN + range * 0.7f * _graspForce;
    uint16_t ringPos = FINGER_POS_MIN + range * 0.8f * _graspForce;
    
    // Set finger positions
    _fingers[0]->setTarget(thumbPos);    // Thumb
    _fingers[1]->setTarget(indexPos);    // Index
    if (_numFingers > 2) {
        _fingers[2]->setTarget(middlePos);   // Middle
    }
    if (_numFingers > 3) {
        _fingers[3]->setTarget(ringPos);     // Ring/pinky
    }
}

void GraspManager::executeHookGrasp() {
    if (!_fingers || _numFingers < 4) return;
    
    // Hook grasp: fingers curl like a hook, thumb stays open
    
    uint16_t range = FINGER_POS_MAX - FINGER_POS_MIN;
    
    // Thumb stays relatively open
    uint16_t thumbPos = FINGER_POS_MIN + range * 0.2f * _graspForce;
    
    // Other fingers curl strongly
    uint16_t indexPos = FINGER_POS_MIN + range * 0.9f * _graspForce;
    uint16_t middlePos = FINGER_POS_MIN + range * 0.95f * _graspForce;
    uint16_t ringPos = FINGER_POS_MIN + range * 0.95f * _graspForce;
    
    // Set finger positions
    _fingers[0]->setTarget(thumbPos);    // Thumb
    _fingers[1]->setTarget(indexPos);    // Index
    _fingers[2]->setTarget(middlePos);   // Middle
    _fingers[3]->setTarget(ringPos);     // Ring/pinky
}

void GraspManager::executePointGesture() {
    if (!_fingers || _numFingers < 4) return;
    
    // Point gesture: index extended, other fingers curled
    
    uint16_t range = FINGER_POS_MAX - FINGER_POS_MIN;
    
    // Thumb partially flexed
    uint16_t thumbPos = FINGER_POS_MIN + range * 0.7f * _graspForce;
    
    // Index extended
    uint16_t indexPos = FINGER_POS_MIN;
    
    // Other fingers fully curled
    uint16_t middlePos = FINGER_POS_MIN + range * 0.95f * _graspForce;
    uint16_t ringPos = FINGER_POS_MIN + range * 0.95f * _graspForce;
    
    // Set finger positions
    _fingers[0]->setTarget(thumbPos);    // Thumb
    _fingers[1]->setTarget(indexPos);    // Index
    _fingers[2]->setTarget(middlePos);   // Middle
    _fingers[3]->setTarget(ringPos);     // Ring/pinky
}

void GraspManager::executeThumbsUpGesture() {
    if (!_fingers || _numFingers < 4) return;
    
    // Thumbs up gesture: thumb extended, other fingers curled
    
    uint16_t range = FINGER_POS_MAX - FINGER_POS_MIN;
    
    // Thumb extended
    uint16_t thumbPos = FINGER_POS_MIN;
    
    // Other fingers curled
    uint16_t indexPos = FINGER_POS_MIN + range * 0.9f * _graspForce;
    uint16_t middlePos = FINGER_POS_MIN + range * 0.95f * _graspForce;
    uint16_t ringPos = FINGER_POS_MIN + range * 0.95f * _graspForce;
    
    // Set finger positions
    _fingers[0]->setTarget(thumbPos);    // Thumb
    _fingers[1]->setTarget(indexPos);    // Index
    _fingers[2]->setTarget(middlePos);   // Middle
    _fingers[3]->setTarget(ringPos);     // Ring/pinky
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

void GraspManager::setTransitionSpeed(float speed) {
    _transitionSpeed = constrain(speed, 0.1f, 1.0f);
}