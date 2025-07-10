/**
 * @file CommandInterface.cpp
 * @brief Enhanced serial command interface implementation
 */

#include "CommandInterface.h"
#include "EMGProcessor.h"
#include "Finger.h"
#include "GraspManager.h"
#include "ReflexEngine.h"
#include "SensorFusion.h"
#include "PowerMonitor.h"
#include "Calibration.h"

// Existing implementation (keeping intact)
// ...

// Add the following methods:

void CommandInterface::begin(
    EMGProcessor* emgProcessor,
    Finger** fingers,
    uint8_t numFingers,
    GraspManager* graspManager,
    ReflexEngine* reflexEngine,
    SensorFusion* sensorFusion,
    PowerMonitor* powerMonitor,
    Calibration* calibration
) {
    _emgProcessor = emgProcessor;
    _fingers = fingers;
    _numFingers = numFingers;
    _graspManager = graspManager;
    _reflexEngine = reflexEngine;
    _sensorFusion = sensorFusion;
    _powerMonitor = powerMonitor;
    _calibration = calibration;
    
    _cmdIndex = 0;
    memset(_cmdBuffer, 0, sizeof(_cmdBuffer));
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
                Serial.print("\b \b"); // Erase character on terminal
            }
            continue;
        }
        
        // Echo character
        Serial.write(c);
        
        // Process on newline
        if (c == '\n' || c == '\r') {
            Serial.println();
            
            // Process command if buffer is not empty
            if (_cmdIndex > 0) {
                _cmdBuffer[_cmdIndex] = '\0'; // Null-terminate
                processCommand(_cmdBuffer);
                
                // Clear buffer
                _cmdIndex = 0;
                memset(_cmdBuffer, 0, sizeof(_cmdBuffer));
            }
            
            // Show prompt
            Serial.print(F("> "));
        }
        // Add character to buffer if not full
        else if (_cmdIndex < CMD_BUFFER_SIZE - 1) {
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
    else if (strcmp(upperCmd, "PRINT_EMG") == 0) {
        printEMGFeatures();
    }
    else if (strncmp(upperCmd, "FORCE_SLIP", 10) == 0) {
        // Parse finger index parameter
        int fingerIndex = atoi(&cmd[10]);
        forceSlip(fingerIndex);
    }
    else if (strcmp(upperCmd, "POWER_STATUS") == 0) {
        printPowerStatus();
    }
    else if (strcmp(upperCmd, "CALIBRATE_EMG") == 0) {
        calibrateEMG();
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
    else if (strcmp(upperCmd, "PRINT_CALIBRATION") == 0) {
        printCalibration();
    }
    else if (strncmp(upperCmd, "SET_SLIP_THRESHOLD", 18) == 0) {
        float threshold = atof(&cmd[18]);
        setSlipThreshold(threshold);
    }
    else if (strcmp(upperCmd, "SYSTEM_STATUS") == 0) {
        printSystemStatus();
    }
    else {
        Serial.print(F("Unknown command: "));
        Serial.println(cmd);
    }
}

void CommandInterface::printHelp() {
    Serial.println(F("Available Commands:"));
    Serial.println(F("  HELP - Show this help message"));
    Serial.println(F("  PRINT_EMG - Display EMG feature vector"));
    Serial.println(F("  FORCE_SLIP <index> - Simulate slip on finger (0-3)"));
    Serial.println(F("  POWER_STATUS - Show battery voltage and current"));
    Serial.println(F("  SYSTEM_STATUS - Show complete system status"));
    Serial.println(F("  CALIBRATE_EMG - Calibrate EMG resting levels"));
    Serial.println(F("  SAVE_CALIBRATION - Save current calibration to EEPROM"));
    Serial.println(F("  LOAD_CALIBRATION - Load calibration from EEPROM"));
    Serial.println(F("  RESET_CALIBRATION - Reset to default calibration"));
    Serial.println(F("  PRINT_CALIBRATION - Show current calibration values"));
    Serial.println(F("  SET_SLIP_THRESHOLD <value> - Set slip detection threshold"));
    // Add any additional commands here
}

void CommandInterface::printEMGFeatures() {
    if (!_emgProcessor) {
        Serial.println(F("EMG processor not initialized"));
        return;
    }
    
    Serial.println(F("EMG Feature Vector:"));
    
    // Print normalized amplitudes
    Serial.println(F("Normalized Amplitudes:"));
    float features[NUM_EMG_CHANNELS];
    _emgProcessor->getFeatureVector(features);
    
    for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
        Serial.print(F("  Ch"));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.print(features[i], 3);
        
        // Print visual bar graph
        Serial.print(F(" |"));
        int barLength = features[i] * 30.0f;
        for (int j = 0; j < 30; j++) {
            Serial.print(j < barLength ? '#' : ' ');
        }
        Serial.println(F("|"));
    }
    
    // Print extended features for each channel
    Serial.println(F("Extended Features:"));
    
    for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
        Serial.print(F("  Ch"));
        Serial.print(i);
        Serial.print(F(": RMS="));
        Serial.print(_emgProcessor->getRMS(i), 3);
        Serial.print(F(" ZC="));
        Serial.print(_emgProcessor->getZeroCrossings(i));
        Serial.print(F(" SSC="));
        Serial.print(_emgProcessor->getSlopeSignChanges(i));
        Serial.print(F(" Active="));
        Serial.println(_emgProcessor->isActivityDetected(i) ? "Yes" : "No");
    }
}

void CommandInterface::forceSlip(int fingerIndex) {
    if (!_reflexEngine || !_fingers || fingerIndex < 0 || fingerIndex >= _numFingers) {
        Serial.println(F("Invalid finger index or ReflexEngine not initialized"));
        return;
    }
    
    Serial.print(F("Simulating slip on finger "));
    Serial.println(fingerIndex);
    
    // Force reflex on specified finger
    _fingers[fingerIndex]->reflexGrip();
}

void CommandInterface::printPowerStatus() {
    if (!_powerMonitor) {
        Serial.println(F("Power monitor not initialized"));
        return;
    }
    
    Serial.println(F("Power System Status:"));
    _powerMonitor->printStatus();
}

void CommandInterface::calibrateEMG() {
    if (!_emgProcessor) {
        Serial.println(F("EMG processor not initialized"));
        return;
    }
    
    Serial.println(F("Calibrating EMG rest levels..."));
    Serial.println(F("Ensure hand is at rest position."));
    Serial.println(F("Calibrating in 3..."));
    delay(1000);
    Serial.println(F("2..."));
    delay(1000);
    Serial.println(F("1..."));
    delay(1000);
    
    // Perform calibration
    _emgProcessor->calibrateRest();
    
    // If we have calibration module, update it too
    if (_calibration) {
        _calibration->calibrateEMGRest();
    }
    
    Serial.println(F("EMG calibration complete"));
}

void CommandInterface::saveCalibration() {
    if (!_calibration) {
        Serial.println(F("Calibration module not initialized"));
        return;
    }
    
    _calibration->saveToEEPROM();
}

void CommandInterface::loadCalibration() {
    if (!_calibration) {
        Serial.println(F("Calibration module not initialized"));
        return;
    }
    
    _calibration->loadFromEEPROM();
}

void CommandInterface::resetCalibration() {
    if (!_calibration) {
        Serial.println(F("Calibration module not initialized"));
        return;
    }
    
    _calibration->resetToDefaults();
}

void CommandInterface::printCalibration() {
    if (!_calibration) {
        Serial.println(F("Calibration module not initialized"));
        return;
    }
    
    _calibration->printCalibration();
}

void CommandInterface::setSlipThreshold(float threshold) {
    if (!_reflexEngine) {
        Serial.println(F("ReflexEngine not initialized"));
        return;
    }
    
    _reflexEngine->setSlipThreshold(threshold);
    
    // Update calibration data if available
    if (_calibration) {
        _calibration->setSlipThreshold(threshold);
    }
    
    Serial.print(F("Slip threshold set to: "));
    Serial.println(threshold, 3);
}

void CommandInterface::printSystemStatus() {
    Serial.println(F("===== System Status ====="));
    
    // Power status
    if (_powerMonitor) {
        _powerMonitor->printStatus();
    }
    
    // EMG status
    if (_emgProcessor) {
        Serial.println(F("EMG Status:"));
        for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
            Serial.print(F("  Ch"));
            Serial.print(i);
            Serial.print(F(": "));
            Serial.print(_emgProcessor->getFeature(i), 3);
            Serial.print(F(" ("));
            Serial.print(_emgProcessor->isActivityDetected(i) ? "Active" : "Inactive");
            Serial.println(F(")"));
        }
    }
    
    // Finger status
    if (_fingers) {
        Serial.println(F("Finger Status:"));
        for (uint8_t i = 0; i < _numFingers; i++) {
            Serial.print(F("  Finger"));
            Serial.print(i);
            Serial.print(F(": Pos="));
            Serial.print(_fingers[i]->getCurrentPosition());
            Serial.print(F(" Target="));
            Serial.print(_fingers[i]->getTarget());
            Serial.print(F(" FSR="));
            Serial.println(_fingers[i]->getFSR(), 3);
        }
    }
    
    // Sensor fusion status
    if (_sensorFusion) {
        Serial.print(F("Sensor Fusion: Grip Force="));
        Serial.print(_sensorFusion->getGripForceMultiplier(), 2);
        Serial.print(F(" Slip Risk="));
        Serial.println(_sensorFusion->getSlipRisk(), 2);
    }
    
    Serial.println(F("========================"));
}