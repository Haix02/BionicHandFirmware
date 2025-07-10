/**
 * @file GraspManager.h
 * @brief Multi-finger grasp pattern coordination
 * @version 2.0
 */

#pragma once

#include <Arduino.h>
#include "Finger.h"
#include "config.h"

class GraspManager {
public:
    GraspManager();
    
    /**
     * @brief Initialize with finger array
     * @param fingers Array of finger pointers
     * @param numFingers Number of fingers in array
     */
    void begin(Finger** fingers, uint8_t numFingers);
    
    /**
     * @brief Update method for continuous adjustments
     * @param dt Time delta in seconds
     */
    void update(float dt);
    
    /**
     * @brief Execute a specific grasp pattern
     * @param mode Grasp mode name
     */
    void executeGrasp(String mode);
    
    /**
     * @brief Set grip force level
     * @param force Force level (0.0-1.0)
     */
    void setGraspForce(float force);
    
    /**
     * @brief Get current grip force level
     * @return Force level (0.0-1.0)
     */
    float getGraspForce() const;
    
    /**
     * @brief Get current grasp mode
     * @return Grasp mode name
     */
    String getCurrentGraspMode() const;

private:
    // Finger array
    Finger** _fingers;
    uint8_t _numFingers;
    
    // Grasp state
    String _currentGraspMode;
    float _graspForce;
    
    // Grasp implementations
    void executeOpenHand();
    void executeCloseHand();
    void executePowerGrip();
    void executePrecisionGrip();
    void executeSphericalGrip();
    void executeTripodGrip();
    void executeLateralGrip();
    void executeHookGrip();
    void executePointingGesture();
};