/**
 * @file CommandInterface.cpp
 * @brief Serial command interface implementation
 * @version 2.0
 */

#include "CommandInterface.h"
#include "EMGProcessor.h"
#include "GraspManager.h"
#include "ReflexEngine.h"
#include "SensorFusion.h"
#include "PowerMonitor.h"
#include "Calibration.h"

CommandInterface::CommandInterface()
    : _emgProcessor(nullptr),
      _graspManager(nullptr),
      _reflexEngine(nullptr),
      _sensorFusion(nullptr),
      _powerMonitor(nullptr),
      _calibration(nullptr),
      _cmdIndex(0)
{
    memset(_cmdBuffer, 0, CMD_BUFFER_SIZE);
}

void CommandInterface::begin(EMGProcessor* emgProcessor, 
                           GraspManager* graspManager,
                           ReflexEngine* reflexEngine,
                           SensorFusion* sensorFusion,
                           PowerMonitor* powerMonitor,
                           Calibration* calibration)
{
    _emgProcessor = emgProcessor;
    _graspManager = graspManager;
    _reflexEngine = reflexEngine;
    _sensorFusion = sensorFusion;
    _powerMonitor = powerMonitor;
    _calibration = calibration;
    
    Serial.println(F("Command interface ready"));
    Serial.println(F("Type 'HELP' for available commands"));
    Serial.print(F("> "));
}

void CommandInterface::update() {
    // Process incoming serial data
    while (Serial.available()) {
        char c = Serial.read();
        
        // Handle backspace
        if (c == '\b' || c == 127) {
            if (_cmdIndex > 0) {
                _cmdIndex--;
                _cmdBuffer[_cmdIndex] = '\0';
                Serial.print(F("\b \b")); // Erase character on terminal
            }
            continue;
        }
        
        // Process on newline
        if (c == '\n' || c == '\r') {
            Serial.println(); // Finish line
            
            // Process command if buffer is not empty
            if (_cmdIndex > 0) {
                _cmdBuffer[_cmdIndex] = '\0'; // Null-terminate
                processCommand(_cmdBuffer);
                
                // Clear buffer
                _cmdIndex = 0;
                memset(_cmdBuffer, 0, CMD_BUFFER_SIZE);
            }
            
            // Show prompt
            Serial.print(F("> "));
            continue;
        }
        
        // Echo character
        Serial.write(c);
        
        // Add character to buffer if not full
        if (_cmdIndex < CMD_BUFFER_SIZE - 1) {
            _cmdBuffer[_cmdIndex++] = c;
        }
    }
}

void CommandInterface::processCommand(const char* cmd) {
    // Convert to uppercase for case-insensitive comparison
    char upperCmd[CMD_BUFFER_SIZE];
    strncpy(upperCmd, cmd, CMD_BUFFER_SIZE);
    upperCmd[CMD_BUFFER_SIZE-1] = '\0';
    
    for (uint8_t i = 0; i < strlen(upperCmd); i++) {
        upperCmd[i] = toupper(upperCmd[i]);
    }
    
    // Process commands
    if (strcmp(upperCmd, "HELP") == 0) {
        printHelp();
    }
    else if (strcmp(upperCmd, "EMG_PRINT") == 0) {
        printEMGFeatures();
    }
    else if (strncmp(upperCmd, "SET_GRASP", 9) == 0) {
        // Extract grasp type
        const char* graspType = cmd + 10; // Skip "SET_GRASP "
        setGrasp(graspType);
    }
    else if (strncmp(upperCmd, "FORCE_SLIP", 10) == 0) {
        // Extract finger index
        int fingerIndex = atoi(cmd + 11); // Skip "FORCE_SLIP "
        forceSlip(fingerIndex);
    }
    else if (strcmp(upperCmd, "CALIBRATE") == 0) {
        calibrate();
    }
    else if (strcmp(upperCmd, "SAVE_CALIBRATION") == 0) {
        saveCalibration();
    }
    else if (strcmp(upperCmd, "LOAD_CALIBRATION") == 0) {
        loadCalibration();
    }
    else if (strcmp(upperCmd, "RESET_CALIBRATION") == 0) {
        resetCalibration();
    }
    else if (strcmp(upperCmd, "POWER_STATUS") == 0) {
        printPowerStatus();
    }
    else if (strcmp(upperCmd, "SYSTEM_STATUS") == 0) {
        printSystemStatus();
    }
    else {
        Serial.print(F("Unknown command: "));
        Serial.println(cmd);
        Serial.println(F("Type 'HELP' for available commands"));
    }
}

void CommandInterface::printHelp() {
    Serial.println(F("Available Commands:"));
    Serial.println(F("  HELP - Show this help message"));
    Serial.println(F("  EMG_PRINT - Display EMG features"));
    Serial.println(F("  SET_GRASP <type> - Set grasp pattern"));
    Serial.println(F("    Types: open, close, power, precision, spherical, tripod, lateral, hook"));
    Serial.println(F("  FORCE_SLIP <index> - Trigger reflex on finger (0-3)"));
    Serial.println(F("  CALIBRATE - Run calibration sequence"));
    Serial.println(F("  SAVE_CALIBRATION - Save calibration to EEPROM"));
    Serial.println(F("  LOAD_CALIBRATION - Load calibration from EEPROM"));
    Serial.println(F("  RESET_CALIBRATION - Reset to default calibration"));
    Serial.println(F("  POWER_STATUS - Display battery status"));
    Serial.println(F("  SYSTEM_STATUS - Display complete system status"));
}

void CommandInterface::printEMGFeatures() {
    if (!_emgProcessor) {
        Serial.println(F("EMG processor not initialized"));
        return;
    }
    
    Serial.println(F("EMG Features:"));
    
    // Print normalized amplitudes
    Serial.println(F("Normalized Amplitudes:"));
    float features[NUM_EMG_CHANNELS];
    _emgProcessor->getFeatureVector(features);
    
    for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
        Serial.print(F("  Ch"));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.print(features[i], 3);
        
        // Visual bar graph
        Serial.print(F(" |"));
        int barLength = features[i] * 30;
        for (int j = 0; j < 30; j++) {
            Serial.print(j < barLength ? '#' : ' ');
        }
        Serial.println(F("|"));
    }
    
    // Print extended features
    Serial.println(F("Extended Features:"));
    for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
        Serial.print(F("  Ch"));
        Serial.print(i);
        Serial.print(F(": RMS="));
        Serial.print(_emgProcessor->emgFeatures[i*EMG_FEATURES_PER_CHANNEL+0], 3);
        Serial.print(F(", ZC="));
        Serial.print(_emgProcessor->emgFeatures[i*EMG_FEATURES_PER_CHANNEL+1], 3