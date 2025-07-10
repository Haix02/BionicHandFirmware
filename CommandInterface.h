/**
 * @file CommandInterface.h
 * @brief Enhanced serial command interface
 * @version 3.0
 */

#pragma once

#include <Arduino.h>
#include "config.h"
#include "EMGProcessor.h"
#include "Finger.h"
#include "GraspManager.h"
#include "ReflexEngine.h"
#include "SensorFusion.h"
#include "Calibration.h"

/**
 * @class CommandInterface
 * @brief Provides serial command parsing and system control
 * 
 * V3 Enhancements:
 * - Advanced debugging and monitoring commands
 * - Motion profile control
 * - Reflex testing and statistics
 * - Enhanced EMG data visualization
 */
class CommandInterface {
public:
    CommandInterface();
    
    /**
     * @brief Initialize with system component pointers
     */
    void begin(EMGProcessor* emgProcessor, Finger** fingers, 
              GraspManager* graspManager, ReflexEngine* reflexEngine = nullptr, 
              SensorFusion* sensorFusion = nullptr, Calibration* calibration = nullptr);
    
    /**
     * @brief Process serial input
     */
    void processSerial();
    
    /**
     * @brief Process a specific command
     */
    bool processCommand(const String& command);
    
    /**
     * @brief Print help information
     */
    void printHelp();
    
private:
    // System components
    EMGProcessor* _emgProcessor;
    Finger** _fingers;
    GraspManager* _graspManager;
    ReflexEngine* _reflexEngine;
    SensorFusion* _sensorFusion;
    Calibration* _calibration;
    
    // Command buffer
    String _commandBuffer;
    
    // V3: Enhanced command processing
    void handleGripCommand(const String& params);
    void handleFingerCommand(const String& params);
    void handleEMGCommand(const String& params);
    void handleProfileCommand(const String& params);
    void handleCalibrationCommand(const String& params);
    void handleReflexCommand(const String& params);
    void handleSensorCommand(const String& params);
    void handleSystemCommand(const String& params);
    void handleDebugCommand(const String& params);
    
    // V3: EMG visualization
    void printEMGVisual();
    void printEMGStats();
};