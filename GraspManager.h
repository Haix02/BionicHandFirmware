/**
 * @file GraspManager.h
 * @brief Coordinates multi-finger grasp patterns
 * @version 2.0
 */

#pragma once

#include <Arduino.h>
#include "config.h"
#include "Finger.h"

class GraspManager {
public:
    GraspManager();
    
    /**
     * @brief Initialize with array of finger pointers
     */
    void begin(Finger** fingers, uint8_t numFingers);
    
    /**
     * @brief Update grasp behavior (call periodically)
     */
    void update();
    
    /**
     * @brief Execute a specific grasp pattern
     * @param mode Grasp mode ("power", "precision", "spherical", etc.)
     */
    void executeGrasp(String mode);
    
    /**
     * @brief Set grip force for all grasps
     * @param force Force level from 0.0 (min) to 1.0 (max)
     */
    void setGraspForce(float force);
    
    /**
     * @brief Get current grip force
     * @return Force level from 0.0 to 1.0
     */
    float getGraspForce() const;
    
    /**
     * @brief Get current grasp mode
     * @return String indicating current grasp mode
     */
    String getCurrentGraspMode() const;
    
    /**
     * @brief Set transition speed for grasp changes
     * @param speed Speed factor from 0.1 (slow) to 1.0 (fast)
     */
    void setTransitionSpeed(float speed);

private:
    Finger** _fingers;     // Array of finger pointers
    uint8_t _numFingers;   // Number of fingers in array
    
    String _currentGraspMode; // Current active grasp mode
    float _graspForce;     // Force level for grasps
    float _transitionSpeed; // Speed factor for transitions
    
    // Grasp pattern implementations
    void executeOpenHand();
    void executeCloseHand();
    void executePowerGrasp();
    void executePrecisionGrasp();
    void executeSphericalGrasp();
    void executeTripodGrasp();
    void executeLateralGrasp();
    void executeHookGrasp();
    void executePointGesture();
    void executeThumbsUpGesture();
};