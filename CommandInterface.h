/**
 * @file CommandInterface.h
 * @brief Enhanced serial command interface
 * @version 2.0
 */

#pragma once

#include <Arduino.h>
#include "config.h"

// Forward declarations
class EMGProcessor;
class Finger;
class GraspManager;
class ReflexEngine;
class SensorFusion;
class PowerMonitor;

class CommandInterface {
public:
    CommandInterface();
    
    /**
     * @brief Initialize with system components
     */
    void begin(
        EMGProcessor* emgProcessor,
        Finger** fingers,
        uint8_t numFingers,
        GraspManager* graspManager = nullptr,
        ReflexEngine* reflexEngine = nullptr,
        SensorFusion* sensorFusion = nullptr,
        PowerMonitor* powerMonitor = nullptr
    );
    
    /**
     * @brief Process incoming serial data
     */
    void update();
    
    /**
     * @brief Process a specific command
     * @param cmd Command string to process
     * @return True if command was recognized and processed
     */
    bool processCommand(const char* cmd);
    
    /**
     * @brief Print help information
     */
    void printHelp();

private:
    EMGProcessor* _emgProcessor;
    Finger** _fingers;
    uint8_t _numFingers;
    GraspManager* _graspManager;
    ReflexEngine* _reflexEngine;
    SensorFusion* _sensorFusion;
    PowerMonitor* _powerMonitor;
    
    // Command buffer
    static constexpr uint8_t CMD_BUF_LEN = 64;
    char _cmdBuffer[CMD_BUF_LEN];
    uint8_t _cmdIndex;
    
    // Command handlers
    void cmdHelp();
    void cmdStatus();
    void cmdEmgFeatures();
    void cmdReflex(int fingerIdx);
    void cmdGraspProfile(const char* profile);
    void cmdPowerStatus();
    void cmdMotionProfile(const char* profile);
    void cmdSoftContact(bool enable);
    void cmdSlipThreshold(float threshold);
};