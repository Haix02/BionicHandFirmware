/**
 * @file FingerModel.h
 * @brief Anatomically-inspired finger model with 3-joint kinematics
 * @version 3.0
 */

#pragma once

#include <Arduino.h>
#include "config.h"

/**
 * @struct JointAngles
 * @brief Represents the three anatomical joint angles in a finger.
 */
struct JointAngles {
    float mcp; ///< Metacarpophalangeal joint angle (degrees)
    float pip; ///< Proximal interphalangeal joint angle (degrees)
    float dip; ///< Distal interphalangeal joint angle (degrees)
    
    JointAngles() : mcp(0), pip(0), dip(0) {}
    
    JointAngles(float m, float p, float d) : 
        mcp(constrain(m, 0.0f, 90.0f)), 
        pip(constrain(p, 0.0f, 110.0f)), 
        dip(constrain(d, 0.0f, 80.0f)) {}
};

/**
 * @struct JointOffsets
 * @brief Calibration offsets for each joint.
 */
struct JointOffsets {
    float mcp;
    float pip;
    float dip;
    
    JointOffsets() : mcp(0), pip(0), dip(0) {}
};

/**
 * @class FingerModel
 * @brief Models the 3-joint kinematics of a human finger with anatomical constraints.
 * 
 * This class provides biomechanically accurate joint angle calculations based on the
 * anatomy of the human hand, including natural coupling between joints that occurs
 * due to tendon mechanics.
 */
class FingerModel {
public:
    FingerModel();
    
    /**
     * @brief Calculate joint angles based on a single actuator position.
     * @param actuatorPosition normalized position (0.0-1.0)
     * @return JointAngles struct with anatomically-coupled joint positions
     */
    JointAngles calculateJointAngles(float actuatorPosition) const;
    
    /**
     * @brief Apply joint offsets for personalization/calibration.
     */
    void setJointOffsets(const JointOffsets& offsets);
    
    /**
     * @brief Get current joint offsets.
     */
    JointOffsets getJointOffsets() const;
    
    /**
     * @brief Get anatomically-normalized finger flexion (0.0 = fully extended, 1.0 = fully flexed)
     */
    float getFlexionAmount(const JointAngles& angles) const;
    
    /**
     * @brief Translates joint angles to a single actuator position.
     * @param angles The joint angles to convert
     * @return Normalized actuator position (0.0-1.0)
     */
    float jointAnglesToActuatorPosition(const JointAngles& angles) const;
    
    /**
     * @brief Set the tendon coupling ratio between DIP and PIP joints
     * @param ratio Typical anatomical value is ~0.67
     */
    void setDipToPipRatio(float ratio);

private:
    JointOffsets _offsets;
    float _dipToPipRatio;  // Anatomical coupling ratio (typically ~0.67)
    
    // Joint ranges
    static constexpr float MCP_MIN = 0.0f;
    static constexpr float MCP_MAX = 90.0f;
    static constexpr float PIP_MIN = 0.0f;
    static constexpr float PIP_MAX = 110.0f;
    static constexpr float DIP_MIN = 0.0f;
    static constexpr float DIP_MAX = 80.0f;
};