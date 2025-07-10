/**
 * @file CommandInterface.h
 * @brief Serial command interface for bionic hand control
 * @version 2.0
 * @author Haix02
 * @date 2025-07-10 23:11:47 UTC
 */

#pragma once

#include <Arduino.h>
#include "config.h"

// Forward declarations
class EMGProcessor;
class GraspManager;
class ReflexEngine;
class PowerMonitor;
class Calibration;
class Finger;

class CommandInterface {
public:
    CommandInterface();
    
    /**
     * @brief Initialize command interface with system components
     * @param emgProcessor Pointer to EMG processor
     * @param graspManager Pointer to grasp manager
     * @param reflexEngine Pointer to reflex engine
     * @param powerMonitor Pointer to power monitor (optional)
     * @param calibration Pointer to calibration module (optional)
     * @param fingers Array of finger pointers
     */
    void begin(EMGProcessor* emgProcessor, 
              GraspManager* graspManager,
              ReflexEngine* reflexEngine,
              PowerMonitor* powerMonitor = nullptr,
              Calibration* calibration = nullptr,
              Finger** fingers = nullptr);
    
    /**
     * @brief Update method - call frequently to process serial input
     */
    void update();

private:
    // System component pointers
    EMGProcessor* _emgProcessor;
    GraspManager* _graspManager;
    ReflexEngine* _reflexEngine;
    PowerMonitor* _powerMonitor;
    Calibration* _calibration;
    Finger** _fingers;
    
    // Command buffer
    char _cmdBuffer[CMD_BUFFER_SIZE];
    uint8_t _cmdIndex;
    
    // Command processing methods
    void processCommand(const char* cmd);
    
    // Individual command implementations
    void cmdHelp();
    void cmdEmgPrint();
    void cmdForceSlip(int fingerIndex);
    void cmdSetGrasp(const char* graspType);
    void cmdCalibrate();
    void cmdPowerStatus();
    void cmdSystemStatus();
    void cmdVersion();
    void cmdReset();
    void cmdFingerControl(const char* cmd);
};