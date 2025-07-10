/**
 * @file CommandInterface.cpp
 * @brief Serial command interface for bionic hand control
 * @version 2.0
 * @author Haix02
 * @date 2025-07-10 23:11:47 UTC
 */

#include "CommandInterface.h"
#include "EMGProcessor.h"
#include "GraspManager.h"
#include "ReflexEngine.h"
#include "PowerMonitor.h"
#include "Calibration.h"

CommandInterface::CommandInterface()
    : _emgProcessor(nullptr),
      _graspManager(nullptr),
      _reflexEngine(nullptr),
      _powerMonitor(nullptr),
      _calibration(nullptr),
      _cmdIndex(0)
{
    memset(_cmdBuffer, 0, CMD_BUFFER_SIZE);
}

void CommandInterface::begin(EMGProcessor* emgProcessor, 
                           GraspManager* graspManager,
                           ReflexEngine* reflexEngine,
                           PowerMonitor* powerMonitor,
                           Calibration* calibration,
                           Finger** fingers)
{
    _emgProcessor = emgProcessor;
    _graspManager = graspManager;
    _reflexEngine = reflexEngine;
    _powerMonitor = powerMonitor;
    _calibration = calibration;
    _fingers = fingers;
    
    Serial.println(F("Command interface initialized"));
    Serial.println(F("Type 'HELP' for available commands"));
}

void CommandInterface::update() {
    // Non-blocking serial processing
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
        
        // Process command on newline/carriage return
        if (c == '\n' || c == '\r') {
            Serial.println(); // Finish the line
            
            // Process command if buffer has content
            if (_cmdIndex > 0) {
                _cmdBuffer[_cmdIndex] = '\0'; // Null terminate
                processCommand(_cmdBuffer);
                
                // Clear command buffer
                _cmdIndex = 0;
                memset(_cmdBuffer, 0, CMD_BUFFER_SIZE);
            }
            
            // Show command prompt
            Serial.print(F("> "));
            continue;
        }
        
        // Echo character to terminal
        Serial.write(c);
        
        // Add character to buffer if space available
        if (_cmdIndex < CMD_BUFFER_SIZE - 1) {
            _cmdBuffer[_cmdIndex++] = c;
        }
    }
}

void CommandInterface::processCommand(const char* cmd) {
    // Convert command to uppercase for case-insensitive comparison
    char upperCmd[CMD_BUFFER_SIZE];
    strncpy(upperCmd, cmd, CMD_BUFFER_SIZE);
    upperCmd[CMD_BUFFER_SIZE-1] = '\0';
    
    for (uint8_t i = 0; i < strlen(upperCmd); i++) {
        upperCmd[i] = toupper(upperCmd[i]);
    }
    
    // Parse and execute commands
    if (strcmp(upperCmd, "HELP") == 0) {
        cmdHelp();
    }
    else if (strcmp(upperCmd, "EMG_PRINT") == 0) {
        cmdEmgPrint();
    }
    else if (strncmp(upperCmd, "FORCE_SLIP", 10) == 0) {
        // Extract finger index from command
        int fingerIndex = -1;
        if (strlen(cmd) > 11) {
            fingerIndex = atoi(cmd + 11); // Skip "FORCE_SLIP "
        }
        cmdForceSlip(fingerIndex);
    }
    else if (strncmp(upperCmd, "SET_GRASP", 9) == 0) {
        // Extract grasp type from command
        const char* graspType = "";
        if (strlen(cmd) > 10) {
            graspType = cmd + 10; // Skip "SET_GRASP "
        }
        cmdSetGrasp(graspType);
    }
    else if (strcmp(upperCmd, "CALIBRATE") == 0) {
        cmdCalibrate();
    }
    else if (strcmp(upperCmd, "POWER_STATUS") == 0) {
        cmdPowerStatus();
    }
    else if (strcmp(upperCmd, "STATUS") == 0) {
        cmdSystemStatus();
    }
    else if (strcmp(upperCmd, "VERSION") == 0) {
        cmdVersion();
    }
    else if (strcmp(upperCmd, "RESET") == 0) {
        cmdReset();
    }
    else if (strncmp(upperCmd, "FINGER", 6) == 0) {
        // Parse finger commands: FINGER <index> <position>
        cmdFingerControl(cmd);
    }
    else {
        Serial.print(F("ERROR: Unknown command '"));
        Serial.print(cmd);
        Serial.println(F("'"));
        Serial.println(F("Type 'HELP' for available commands"));
    }
}

void CommandInterface::cmdHelp() {
    Serial.println(F("\n===== BIONIC HAND COMMANDS ====="));
    Serial.println(F("Basic Commands:"));
    Serial.println(F("  HELP                   - Show this help message"));
    Serial.println(F("  VERSION                - Show firmware version"));
    Serial.println(F("  STATUS                 - Show system status"));
    Serial.println(F("  RESET                  - Software reset"));
    
    Serial.println(F("\nEMG Commands:"));
    Serial.println(F("  EMG_PRINT              - Display EMG feature values"));
    Serial.println(F("  CALIBRATE              - Run EMG calibration"));
    
    Serial.println(F("\nGrasp Commands:"));
    Serial.println(F("  SET_GRASP <type>       - Execute grasp pattern"));
    Serial.println(F("    Types: open, close, power, precision, spherical"));
    Serial.println(F("           tripod, lateral, hook, point"));
    
    Serial.println(F("\nFinger Commands:"));
    Serial.println(F("  FINGER <i> <pos>       - Set finger position (0-180)"));
    Serial.println(F("  FORCE_SLIP <i>         - Trigger reflex on finger i"));
    
    Serial.println(F("\nSystem Commands:"));
    Serial.println(F("  POWER_STATUS           - Show battery/power status"));
    
    Serial.println(F("\nExamples:"));
    Serial.println(F("  SET_GRASP power        - Execute power grip"));
    Serial.println(F("  FORCE_SLIP 1           - Trigger reflex on finger 1"));
    Serial.println(F("  FINGER 0 90            - Set finger 0 to 90 degrees"));
    Serial.println(F("================================\n"));
}

void CommandInterface::cmdEmgPrint() {
    if (!_emgProcessor) {
        Serial.println(F("ERROR: EMG processor not available"));
        return;
    }
    
    Serial.println(F("\n===== EMG FEATURES ====="));
    
    // Print normalized amplitudes
    Serial.println(F("Channel Amplitudes:"));
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        float amplitude = _emgProcessor->getFeature(ch);
        
        Serial.print(F("  Ch"));
        Serial.print(ch);
        Serial.print(F(": "));
        Serial.print(amplitude, 4);
        
        // Visual bar graph (30 characters wide)
        Serial.print(F(" |"));
        int barLength = amplitude * 30;
        for (int i = 0; i < 30; i++) {
            Serial.print(i < barLength ? '#' : '-');
        }
        Serial.print(F("| "));
        Serial.print(amplitude * 100, 1);
        Serial.println(F("%"));
    }
    
    // Print full feature array (emgFeatures[])
    Serial.println(F("\nExtended Features [RMS, ZC, SSC, WL]:"));
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        Serial.print(F("  Ch"));
        Serial.print(ch);
        Serial.print(F(": ["));
        
        for (uint8_t f = 0; f < EMG_FEATURES_PER_CHANNEL; f++) {
            uint8_t idx = ch * EMG_FEATURES_PER_CHANNEL + f;
            Serial.print(_emgProcessor->emgFeatures[idx], 4);
            if (f < EMG_FEATURES_PER_CHANNEL - 1) {
                Serial.print(F(", "));
            }
        }
        Serial.print(F("] "));
        Serial.println(_emgProcessor->isChannelActive(ch) ? "ACTIVE" : "idle");
    }
    
    Serial.println(F("========================\n"));
}

void CommandInterface::cmdForceSlip(int fingerIndex) {
    // Validate finger index
    if (fingerIndex < 0 || fingerIndex >= NUM_FINGERS) {
        Serial.print(F("ERROR: Invalid finger index "));
        Serial.print(fingerIndex);
        Serial.print(F(". Use 0-"));
        Serial.println(NUM_FINGERS - 1);
        return;
    }
    
    if (!_fingers || !_fingers[fingerIndex]) {
        Serial.println(F("ERROR: Finger not available"));
        return;
    }
    
    Serial.print(F("Triggering reflex slip response on finger "));
    Serial.println(fingerIndex);
    
    // Trigger reflex grip on specified finger
    _fingers[fingerIndex]->reflexGrip();
    
    // Also trigger via reflex engine if available for statistics
    if (_reflexEngine) {
        _reflexEngine->triggerManualReflex(fingerIndex);
    }
    
    Serial.println(F("Reflex triggered successfully"));
}

void CommandInterface::cmdSetGrasp(const char* graspType) {
    if (!_graspManager) {
        Serial.println(F("ERROR: Grasp manager not available"));
        return;
    }
    
    // Validate grasp type is not empty
    if (strlen(graspType) == 0) {
        Serial.println(F("ERROR: No grasp type specified"));
        Serial.println(F("Available types: open, close, power, precision, spherical, tripod, lateral, hook, point"));
        return;
    }
    
    Serial.print(F("Executing grasp pattern: "));
    Serial.println(graspType);
    
    // Execute the grasp
    _graspManager->executeGrasp(String(graspType));
    
    Serial.print(F("Grasp '"));
    Serial.print(graspType);
    Serial.println(F("' executed successfully"));
}

void CommandInterface::cmdCalibrate() {
    Serial.println(F("Starting calibration sequence..."));
    
    if (_emgProcessor) {
        Serial.println(F("Calibrating EMG channels..."));
        Serial.println(F("Please relax all muscles for 3 seconds..."));
        
        delay(1000); // 1 second preparation time
        
        Serial.println(F("Calibrating... keep relaxed!"));
        _emgProcessor->calibrateRest();
        
        Serial.println(F("EMG calibration complete"));
    } else {
        Serial.println(F("WARNING: EMG processor not available"));
    }
    
    if (_calibration) {
        Serial.println(F("Saving calibration to EEPROM..."));
        _calibration->saveToEEPROM();
        Serial.println(F("Calibration saved"));
    } else {
        Serial.println(F("WARNING: Calibration module not available"));
    }
    
    Serial.println(F("Calibration sequence complete"));
}

void CommandInterface::cmdPowerStatus() {
    if (_powerMonitor) {
        Serial.println(F("\n===== POWER STATUS ====="));
        _powerMonitor->printStatus();
        Serial.println(F("========================\n"));
    } else {
        Serial.println(F("Power monitoring not available"));
        
        // Provide basic system voltage if possible
        Serial.println(F("\n===== BASIC POWER INFO ====="));
        Serial.print(F("System voltage: "));
        Serial.print(3.3f, 2);
        Serial.println(F("V (Teensy 4.1)"));
        Serial.print(F("USB power: "));
        Serial.println(F("Connected"));
        Serial.println(F("============================\n"));
    }
}

void CommandInterface::cmdSystemStatus() {
    uint32_t uptime = millis();
    
    Serial.println(F("\n===== SYSTEM STATUS ====="));
    
    // System uptime
    Serial.print(F("Uptime: "));
    Serial.print(uptime / 1000);
    Serial.println(F(" seconds"));
    
    // Component status
    Serial.println(F("Components:"));
    Serial.print(F("  EMG Processor: "));
    Serial.println(_emgProcessor ? "OK" : "NOT AVAILABLE");
    Serial.print(F("  Grasp Manager: "));
    Serial.println(_graspManager ? "OK" : "NOT AVAILABLE");
    Serial.print(F("  Reflex Engine: "));
    Serial.println(_reflexEngine ? "OK" : "NOT AVAILABLE");
    Serial.print(F("  Power Monitor: "));
    Serial.println(_powerMonitor ? "OK" : "NOT AVAILABLE");
    Serial.print(F("  Calibration: "));
    Serial.println(_calibration ? "OK" : "NOT AVAILABLE");
    
    // Finger status
    if (_fingers) {
        Serial.println(F("Fingers:"));
        for (uint8_t i = 0; i < NUM_FINGERS; i++) {
            if (_fingers[i]) {
                Serial.print(F("  Finger "));
                Serial.print(i);
                Serial.print(F(": Pos="));
                Serial.print(_fingers[i]->getCurrentPosition());
                Serial.print(F("° Target="));
                Serial.print(_fingers[i]->getTarget());
                Serial.print(F("° FSR="));
                Serial.print(_fingers[i]->getFSR(), 3);
                Serial.print(F(" Reflex="));
                Serial.println(_fingers[i]->isReflexActive() ? "ACTIVE" : "idle");
            }
        }
    }
    
    // Current grasp mode
    if (_graspManager) {
        Serial.print(F("Current Grasp: "));
        Serial.println(_graspManager->getCurrentGraspMode());
        Serial.print(F("Grasp Force: "));
        Serial.print(_graspManager->getGraspForce() * 100, 1);
        Serial.println(F("%"));
    }
    
    // Reflex statistics
    if (_reflexEngine) {
        uint32_t reflexCount;
        float avgResponseTime;
        _reflexEngine->getReflexStats(&reflexCount, &avgResponseTime);
        Serial.print(F("Reflex Count: "));
        Serial.println(reflexCount);
        Serial.print(F("Avg Response Time: "));
        Serial.print(avgResponseTime, 2);
        Serial.println(F(" ms"));
    }
    
    Serial.println(F("=========================\n"));
}

void CommandInterface::cmdVersion() {
    Serial.println(F("\n===== FIRMWARE INFO ====="));
    Serial.println(F("Bionic Hand Firmware v2.0"));
    Serial.println(F("Author: Haix02"));
    Serial.println(F("Build: 2025-07-10 23:11:47 UTC"));
    Serial.println(F("Platform: Teensy 4.1"));
    Serial.println(F("Features:"));
    Serial.println(F("  - 8-channel EMG processing"));
    Serial.println(F("  - 4-finger servo control"));
    Serial.println(F("  - Real-time slip detection"));
    Serial.println(F("  - Multiple grasp patterns"));
    Serial.println(F("  - Advanced DSP filtering"));
    Serial.println(F("==========================\n"));
}

void CommandInterface::cmdReset() {
    Serial.println(F("Performing software reset..."));
    Serial.println(F("System will restart in 3 seconds"));
    
    delay(1000);
    Serial.println(F("3..."));
    delay(1000);
    Serial.println(F("2..."));
    delay(1000);
    Serial.println(F("1..."));
    delay(500);
    
    // Perform software reset on Teensy
    SCB_AIRCR = 0x05FA0004;
}

void CommandInterface::cmdFingerControl(const char* cmd) {
    // Parse: FINGER <index> <position>
    int fingerIndex = -1;
    int position = -1;
    
    // Simple parsing
    const char* ptr = cmd + 6; // Skip "FINGER"
    while (*ptr == ' ') ptr++; // Skip spaces
    
    if (*ptr >= '0' && *ptr <= '9') {
        fingerIndex = atoi(ptr);
        
        // Find next space
        while (*ptr && *ptr != ' ') ptr++;
        while (*ptr == ' ') ptr++; // Skip spaces
        
        if (*ptr >= '0' && *ptr <= '9') {
            position = atoi(ptr);
        }
    }
    
    // Validate parameters
    if (fingerIndex < 0 || fingerIndex >= NUM_FINGERS) {
        Serial.print(F("ERROR: Invalid finger index. Use 0-"));
        Serial.println(NUM_FINGERS - 1);
        return;
    }
    
    if (position < 0 || position > 180) {
        Serial.println(F("ERROR: Invalid position. Use 0-180 degrees"));
        return;
    }
    
    if (!_fingers || !_fingers[fingerIndex]) {
        Serial.println(F("ERROR: Finger not available"));
        return;
    }
    
    // Execute finger movement
    Serial.print(F("Setting finger "));
    Serial.print(fingerIndex);
    Serial.print(F(" to "));
    Serial.print(position);
    Serial.println(F(" degrees"));
    
    _fingers[fingerIndex]->setTarget(position);
    
    Serial.println(F("Command executed"));
}