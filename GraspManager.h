/**
 * @file GraspManager.h
 * @brief Enhanced grasp coordination with biomimetic grasp synergies
 * @version 3.0
 */

#pragma once

#include <Arduino.h>
#include "config.h"
#include "Finger.h"

/**
 * @enum GraspType
 * @brief Supported grasp types.
 */
enum class GraspType : uint8_t {
    Open,       ///< Open hand
    Close,      ///< Close hand
    Power,      ///< Power grip (cylindrical objects)
    Spherical,  ///< Spherical grip (round objects)
    Pinch,      ///< Precision pinch (small objects)
    Hook,       ///< Hook grip (carrying)
    Lateral,    ///< Key/lateral grip
    Custom      ///< Custom grip pattern
};

/**
 * @class GraspManager
 * @brief Multi-finger coordination and gesture/grip logic.
 * 
 * V3 Enhancements:
 * - Biomimetic grip synergies
 * - Context-aware grip adaptation
 * - Variable grip force based on sensor fusion
 */
class GraspManager {
public:
    GraspManager(Finger* fingers[NUM_FINGERS]);

    // Core functionality
    void setGraspType(GraspType grasp);
    GraspType getGraspType() const;
    
    // V3: Enhanced update with context-aware force control
    void update(const float emgFeature[NUM_EMG_CHANNELS], float forceScale = 1.0f);
    
    // Debug information
    void printDebug(Stream &s);

    // Custom grip mapping
    void setCustomGrip(uint16_t target[NUM_FINGERS]);
    
    // V3: Grip force control
    void setGripForce(float force); // 0.0-1.0 scale
    float getGripForce() const;
    
    // V3: Finger cascade timing control
    void enableNaturalTiming(bool enable);
    
    // V3: Grip transition speed
    void setGripTransitionSpeed(float speed); // 0.0-1.0 scale

private:
    Finger* _fingers[NUM_FINGERS];
    GraspType _currentGrasp;
    uint16_t _customGrip[NUM_FINGERS];
    
    // V3: Enhanced grip control
    float _gripForce;           ///< Overall grip force (0.0-1.0)
    bool _naturalTiming;        ///< Enable natural finger cascade
    float _transitionSpeed;     ///< Speed of grip transitions
    uint32_t _gripChangeTime;   ///< Time when grip was last changed
    
    // Finger timing delays for natural cascade (in ms)
    uint8_t _fingerDelays[NUM_FINGERS];
    
    // Grip implementation methods
    void executeGrasp(float forceScale);
    void openHand();
    void closeHand();
    void powerGrip();
    void sphericalGrip();
    void pinchGrip();
    void hookGrip();
    void lateralGrip();
    void customGrip();
    
    // V3: Helper methods
    void calculateFingerDelays();
    float calculateFingerForce(uint8_t fingerIdx, float baseForce);
};