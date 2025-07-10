/**
 * @file FingerModel.cpp
 * @brief Implementation of the anatomically-inspired finger model
 * @version 3.0
 */

#include "FingerModel.h"

FingerModel::FingerModel() : _dipToPipRatio(0.67f) {
    // Initialize with zero offsets
    _offsets = JointOffsets();
}

JointAngles FingerModel::calculateJointAngles(float actuatorPosition) const {
    // Clamp actuator position to valid range
    actuatorPosition = constrain(actuatorPosition, 0.0f, 1.0f);
    
    // Calculate primary joint angles - MCP drives the system
    float mcp = MCP_MIN + actuatorPosition * (MCP_MAX - MCP_MIN) + _offsets.mcp;
    
    // PIP follows MCP but with physiological bias (greater range)
    // In real fingers, PIP flexes more than MCP in most grips
    float pip_ratio = 1.2f; // PIP moves ~20% more than MCP
    float pip = PIP_MIN + actuatorPosition * pip_ratio * (PIP_MAX - PIP_MIN) + _offsets.pip;
    
    // DIP is anatomically coupled to PIP via tendon mechanics
    // _dipToPipRatio simulates this coupling (~0.67 matches human anatomy)
    float dip = DIP_MIN + (pip - PIP_MIN) * _dipToPipRatio + _offsets.dip;
    
    // Ensure we're within anatomical limits
    mcp = constrain(mcp, MCP_MIN, MCP_MAX);
    pip = constrain(pip, PIP_MIN, PIP_MAX);
    dip = constrain(dip, DIP_MIN, DIP_MAX);
    
    return JointAngles(mcp, pip, dip);
}

void FingerModel::setJointOffsets(const JointOffsets& offsets) {
    _offsets = offsets;
}

JointOffsets FingerModel::getJointOffsets() const {
    return _offsets;
}

float FingerModel::getFlexionAmount(const JointAngles& angles) const {
    // Calculate normalized flexion based on weighted sum of joint angles
    // This gives a single 0.0-1.0 value representing overall finger flexion
    
    // Weights reflect contribution to functional grasp (MCP most important)
    const float MCP_WEIGHT = 0.5f;
    const float PIP_WEIGHT = 0.35f;
    const float DIP_WEIGHT = 0.15f;
    
    float mcp_norm = (angles.mcp - MCP_MIN) / (MCP_MAX - MCP_MIN);
    float pip_norm = (angles.pip - PIP_MIN) / (PIP_MAX - PIP_MIN);
    float dip_norm = (angles.dip - DIP_MIN) / (DIP_MAX - DIP_MIN);
    
    return MCP_WEIGHT * mcp_norm + PIP_WEIGHT * pip_norm + DIP_WEIGHT * dip_norm;
}

float FingerModel::jointAnglesToActuatorPosition(const JointAngles& angles) const {
    // Inverse kinematics - convert joint angles back to actuator position
    // We primarily use MCP for this calculation since it's most directly driven
    
    // Calculate normalized MCP position
    float mcp_norm = (angles.mcp - _offsets.mcp - MCP_MIN) / (MCP_MAX - MCP_MIN);
    mcp_norm = constrain(mcp_norm, 0.0f, 1.0f);
    
    // Also check PIP for verification (should be correlated)
    float pip_norm = (angles.pip - _offsets.pip - PIP_MIN) / (PIP_MAX - PIP_MIN) / 1.2f;
    pip_norm = constrain(pip_norm, 0.0f, 1.0f);
    
    // Weighted average, favoring MCP
    return mcp_norm * 0.8f + pip_norm * 0.2f;
}

void FingerModel::setDipToPipRatio(float ratio) {
    _dipToPipRatio = constrain(ratio, 0.5f, 0.8f);  // Limit to anatomically feasible range
}