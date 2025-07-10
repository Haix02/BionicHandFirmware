/**
 * @file GraspManager.cpp
 * @brief Implementation of enhanced grasp coordination
 * @version 3.0
 */
#include "GraspManager.h"

GraspManager::GraspManager(Finger* fingers[NUM_FINGERS])
    : _currentGrasp(GraspType::Open), _gripForce(1.0f),
      _naturalTiming(true), _transitionSpeed(0.8f), _gripChangeTime(0)
{
    for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
        _fingers[i] = fingers[i];
        _customGrip[i] = FINGER_POS_MIN;
    }
    
    // Calculate natural finger cascade delays
    calculateFingerDelays();
}

void GraspManager::setGraspType(GraspType grasp) {
    _currentGrasp = grasp;
    _gripChangeTime = millis(); // Record time of grip change
}

GraspType GraspManager::getGraspType() const {
    return _currentGrasp;
}

void GraspManager::update(const float emgFeature[NUM_EMG_CHANNELS], float forceScale) {
    // Apply force scaling to current grip
    executeGrasp(forceScale);
}

void GraspManager::executeGrasp(float forceScale) {
    // Apply grasp with force scaling
    // Force scale adjusts the overall grip strength
    float effectiveForce = _gripForce * forceScale;
    
    // Implement appropriate grip pattern
    switch (_currentGrasp) {
        case GraspType::Open:
            openHand();
            break;
        case GraspType::Close:
            closeHand();
            break;
        case GraspType::Power:
            powerGrip();
            break;
        case GraspType::Spherical:
            sphericalGrip();
            break;
        case GraspType::Pinch:
            pinchGrip();
            break;
        case GraspType::Hook:
            hookGrip();
            break;
        case GraspType::Lateral:
            lateralGrip();
            break;
        case GraspType::Custom:
            customGrip();
            break;
        default:
            openHand();
            break;
    }
}

void GraspManager::openHand() {
    // Set all fingers to open position
    for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
        if (_naturalTiming) {
            // Apply natural finger cascade timing
            delay(_fingerDelays[i]);
        }
        _fingers[i]->setTarget(FINGER_POS_MIN);
    }
}

void GraspManager::closeHand() {
    // Calculate force-scaled closing position
    uint16_t closePos = FINGER_POS_MIN + _gripForce * (FINGER_POS_MAX - FINGER_POS_MIN);
    
    // Set all fingers to closed position
    for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
        if (_naturalTiming) {
            // Apply natural finger cascade timing
            delay(_fingerDelays[i]);
        }
        
        // Calculate finger-specific force
        float fingerForce = calculateFingerForce(i, _gripForce);
        uint16_t fingerPos = FINGER_POS_MIN + fingerForce * (FINGER_POS_MAX - FINGER_POS_MIN);
        
        _fingers[i]->setTarget(fingerPos);
    }
}

void GraspManager::powerGrip() {
    // Power grip - all fingers curl with strong force
    // Thumb opposes other fingers
    
    // Calculate finger positions for power grip
    // In a real power grip, fingers curl with different angles
    float positions[NUM_FINGERS];
    
    // Thumb position (typically less flexed than other fingers in power grip)
    positions[0] = 0.8f;
    
    // Other fingers progressively more curled
    // Index finger typically less curled than pinky in power grip
    positions[1] = 0.85f;  // Index
    positions[2] = 0.90f;  // Middle
    positions[3] = 0.95f;  // Ring/pinky
    
    // Apply grip positions with force scaling
    for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
        if (_naturalTiming) {
            delay(_fingerDelays[i]);
        }
        
        float fingerForce = calculateFingerForce(i, _gripForce);
        uint16_t pos = FINGER_POS_MIN + positions[i] * fingerForce * (FINGER_POS_MAX - FINGER_POS_MIN);
        _fingers[i]->setTarget(pos);
    }
}

void GraspManager::sphericalGrip() {
    // Spherical grip - fingers form cup around spherical object
    
    // Calculate finger positions for spherical grip
    float positions[NUM_FINGERS];
    
    // In spherical grip, thumb opposes and fingers are more spread out
    positions[0] = 0.75f;  // Thumb
    positions[1] = 0.65f;  // Index - less curled
    positions[2] = 0.70f;  // Middle
    positions[3] = 0.75f;  // Ring/pinky
    
    // Apply grip positions with force scaling
    for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
        if (_naturalTiming) {
            delay(_fingerDelays[i]);
        }
        
        float fingerForce = calculateFingerForce(i, _gripForce);
        uint16_t pos = FINGER_POS_MIN + positions[i] * fingerForce * (FINGER_POS_MAX - FINGER_POS_MIN);
        _fingers[i]->setTarget(pos);
    }
}

void GraspManager::pinchGrip() {
    // Pinch grip - thumb and index for precision grip
    
    // Calculate finger positions for pinch grip
    float positions[NUM_FINGERS];
    
    // Thumb and index oppose each other, other fingers curl out of the way
    positions[0] = 0.6f;   // Thumb
    positions[1] = 0.6f;   // Index
    positions[2] = 0.9f;   // Middle (curls out of way)
    positions[3] = 0.95f;  // Ring/pinky (curls out of way)
    
    // Apply grip positions with force scaling
    for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
        if (_naturalTiming) {
            delay(_fingerDelays[i]);
        }
        
        float fingerForce = calculateFingerForce(i, _gripForce);
        uint16_t pos = FINGER_POS_MIN + positions[i] * fingerForce * (FINGER_POS_MAX - FINGER_POS_MIN);
        _fingers[i]->setTarget(pos);
    }
}

void GraspManager::hookGrip() {
    // Hook grip - fingers curl, thumb doesn't participate
    
    // Calculate finger positions for hook grip
    float positions[NUM_FINGERS];
    
    // Thumb stays open, other fingers curl strongly
    positions[0] = 0.0f;   // Thumb stays open
    positions[1] = 0.95f;  // Index curls strongly
    positions[2] = 0.95f;  // Middle curls strongly
    positions[3] = 0.95f;  // Ring/pinky curls strongly
    
    // Apply grip positions with force scaling
    for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
        if (_naturalTiming) {
            delay(_fingerDelays[i]);
        }
        
        float fingerForce = calculateFingerForce(i, _gripForce);
        uint16_t pos = FINGER_POS_MIN + positions[i] * fingerForce * (FINGER_POS_MAX - FINGER_POS_MIN);
        _fingers[i]->setTarget(pos);
    }
}

void GraspManager::lateralGrip() {
    // Lateral/key grip - thumb presses against side of index
    
    // Calculate finger positions for lateral grip
    float positions[NUM_FINGERS];
    
    // Thumb opposes side of index, other fingers curl partially
    positions[0] = 0.7f;   // Thumb 
    positions[1] = 0.5f;   // Index (partially curled)
    positions[2] = 0.8f;   // Middle (more curled)
    positions[3] = 0.8f;   // Ring/pinky (more curled)
    
    // Apply grip positions with force scaling
    for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
        if (_naturalTiming) {
            delay(_fingerDelays[i]);
        }
        
        float fingerForce = calculateFingerForce(i, _gripForce);
        uint16_t pos = FINGER_POS_MIN + positions[i] * fingerForce * (FINGER_POS_MAX - FINGER_POS_MIN);
        _fingers[i]->setTarget(pos);
    }
}

void GraspManager::customGrip() {
    // Apply custom grip values
    for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
        if (_naturalTiming) {
            delay(_fingerDelays[i]);
        }
        
        float fingerForce = calculateFingerForce(i, _gripForce);
        uint16_t range = FINGER_POS_MAX - FINGER_POS_MIN;
        uint16_t scaled = _customGrip[i] - FINGER_POS_MIN;
        uint16_t pos = FINGER_POS_MIN + scaled * fingerForce;
        _fingers[i]->setTarget(pos);
    }
}

void GraspManager::setCustomGrip(uint16_t target[NUM_FINGERS]) {
    for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
        _customGrip[i] = constrain(target[i], FINGER_POS_MIN, FINGER_POS_MAX);
    }
}

void GraspManager::calculateFingerDelays() {
    // Natural finger cascade timing
    // Different fingers naturally move with slight delays in human hand
    _fingerDelays[0] = 0;    // Thumb (no delay)
    _fingerDelays[1] = 10;   // Index finger (small delay)
    _fingerDelays[2] = 20;   // Middle finger
    _fingerDelays[3] = 35;   // Ring/pinky (longest delay)
}

float GraspManager::calculateFingerForce(uint8_t fingerIdx, float baseForce) {
    // Allow for finger-specific force adjustments
    // This can be used to simulate different finger strengths
    
    // Default implementation just returns base force
    return baseForce;
    
    // Could be expanded to adjust per finger:
    // static const float fingerStrength[NUM_FINGERS] = {1.0f, 0.9f, 1.0f, 0.8f};
    // return baseForce * fingerStrength[fingerIdx];
}

void GraspManager::setGripForce(float force) {
    _gripForce = constrain(force, 0.0f, 1.0f);
}

float GraspManager::getGripForce() const {
    return _gripForce;
}

void GraspManager::enableNaturalTiming(bool enable) {
    _naturalTiming = enable;
}

void GraspManager::setGripTransitionSpeed(float speed) {
    _transitionSpeed = constrain(speed, 0.1f, 1.0f);
    
    // Apply to finger motion profiles
    for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
        float scaledVelocity = MAX_VELOCITY_DEG_PER_SEC * _transitionSpeed;
        float scaledAccel = MAX_ACCELERATION_DEG_PER_SEC2 * _transitionSpeed;
        _fingers[i]->setVelocityLimits(scaledVelocity, scaledAccel);
    }
}

void GraspManager::printDebug(Stream &s) {
    s.print(F("Grasp:")); s.print((int)_currentGrasp);
    s.print(F(" Force:")); s.print(_gripForce, 2);
    s.print(F(" Fingers:"));
    for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
        s.print(F(" ")); s.print(_fingers[i]->getCurrentPosition());
    }
}