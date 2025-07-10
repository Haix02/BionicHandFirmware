/**
 * @file CommandInterface.cpp
 * @brief Implementation of enhanced serial command interface
 * @version 3.0
 */

#include "CommandInterface.h"

CommandInterface::CommandInterface() 
    : _emgProcessor(nullptr), _fingers(nullptr), _graspManager(nullptr),
      _reflexEngine(nullptr), _sensorFusion(nullptr), _calibration(nullptr)
{
}

void CommandInterface::begin(EMGProcessor* emgProcessor, Finger** fingers, 
                           GraspManager* graspManager, ReflexEngine* reflexEngine, 
                           SensorFusion* sensorFusion, Calibration* calibration) 
{
    _emgProcessor = emgProcessor;
    _fingers = fingers;
    _graspManager = graspManager;
    _reflexEngine = reflexEngine;
    _sensorFusion = sensorFusion;
    _calibration = calibration;
    
    _commandBuffer = "";
}

void CommandInterface::processSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        
        // Process on newline
        if (c == '\n') {
            processCommand(_commandBuffer);
            _commandBuffer = "";
        } 
        else if (c != '\r') { // Ignore carriage returns
            _commandBuffer += c;
        }
    }
}

bool CommandInterface::processCommand(const String& command) {
    String cmd = command;
    cmd.trim();
    
    if (cmd.length() == 0) {
        return false;
    }
    
    // Check for command prefix
    int spaceIndex = cmd.indexOf(' ');
    String prefix = (spaceIndex >= 0) ? cmd.substring(0, spaceIndex) : cmd;
    String params = (spaceIndex >= 0) ? cmd.substring(spaceIndex + 1) : "";
    
    // Process command based on prefix
    if (prefix.equalsIgnoreCase("GRIP")) {
        handleGripCommand(params);
    }
    else if (prefix.equalsIgnoreCase("FINGER")) {
        handleFingerCommand(params);
    }
    else if (prefix.equalsIgnoreCase("EMG")) {
        handleEMGCommand(params);
    }
    else if (prefix.equalsIgnoreCase("PROFILE")) {
        handleProfileCommand(params);
    }
    else if (prefix.equalsIgnoreCase("CALIBRATE")) {
        handleCalibrationCommand(params);
    }
    else if (prefix.equalsIgnoreCase("REFLEX")) {
        handleReflexCommand(params);
    }
    else if (prefix.equalsIgnoreCase("SENSOR")) {
        handleSensorCommand(params);
    }
    else if (prefix.equalsIgnoreCase("SYSTEM")) {
        handleSystemCommand(params);
    }
    else if (prefix.equalsIgnoreCase("DEBUG")) {
        handleDebugCommand(params);
    }
    else if (prefix.equalsIgnoreCase("HELP")) {
        printHelp();
    }
    else {
        Serial.print(F("Unknown command: "));
        Serial.println(prefix);
        return false;
    }
    
    return true;
}

void CommandInterface::handleGripCommand(const String& params) {
    if (!_graspManager) return;
    
    if (params.equalsIgnoreCase("OPEN")) {
        _graspManager->setGraspType(GraspType::Open);
        Serial.println(F("Opening hand"));
    }
    else if (params.equalsIgnoreCase("CLOSE")) {
        _graspManager->setGraspType(GraspType::Close);
        Serial.println(F("Closing hand"));
    }
    else if (params.equalsIgnoreCase("POWER")) {
        _graspManager->setGraspType(GraspType::Power);
        Serial.println(F("Power grip"));
    }
    else if (params.equalsIgnoreCase("SPHERICAL")) {
        _graspManager->setGraspType(GraspType::Spherical);
        Serial.println(F("Spherical grip"));
    }
    else if (params.equalsIgnoreCase("PINCH")) {
        _graspManager->setGraspType(GraspType::Pinch);
        Serial.println(F("Pinch grip"));
    }
    else if (params.equalsIgnoreCase("HOOK")) {
        _graspManager->setGraspType(GraspType::Hook);
        Serial.println(F("Hook grip"));
    }
    else if (params.equalsIgnoreCase("LATERAL")) {
        _graspManager->setGraspType(GraspType::Lateral);
        Serial.println(F("Lateral/key grip"));
    }
    else if (params.startsWith("FORCE")) {
        int spaceIdx = params.indexOf(' ');
        if (spaceIdx > 0) {
            float force = params.substring(spaceIdx + 1).toFloat();
            _graspManager->setGripForce(force);
            Serial.print(F("Grip force set to: "));
            Serial.println(force);
        } else {
            Serial.print(F("Current force: "));
            Serial.println(_graspManager->getGripForce());
        }
    }
    else if (params.equalsIgnoreCase("STATUS")) {
        _graspManager->printDebug(Serial);
        Serial.println();
    }
    else {
        Serial.println(F("Available grip commands: OPEN, CLOSE, POWER, SPHERICAL, PINCH, HOOK, LATERAL, FORCE, STATUS"));
    }
}

void CommandInterface::handleFingerCommand(const String& params) {
    if (!_fingers) return;
    
    // Parse finger index and command
    int spaceIdx = params.indexOf(' ');
    if (spaceIdx <= 0) {
        Serial.println(F("Usage: FINGER <index> <command> [value]"));
        return;
    }
    
    int fingerIdx = params.substring(0, spaceIdx).toInt();
    String fingerCmd = params.substring(spaceIdx + 1);
    
    if (fingerIdx < 0 || fingerIdx >= NUM_FINGERS) {
        Serial.print(F("Invalid finger index: "));
        Serial.println(fingerIdx);
        return;
    }
    
    // Process finger-specific command
    if (fingerCmd.startsWith("POS")) {
        int cmdSpaceIdx = fingerCmd.indexOf(' ');
        if (cmdSpaceIdx > 0) {
            uint16_t pos = fingerCmd.substring(cmdSpaceIdx + 1).toInt();
            _fingers[fingerIdx]->setTarget(pos);
            Serial.print(F("Setting finger "));
            Serial.print(fingerIdx);
            Serial.print(F(" to position "));
            Serial.println(pos);
        } else {
            Serial.print(F("Finger "));
            Serial.print(fingerIdx);
            Serial.print(F(" position: "));
            Serial.println(_fingers[fingerIdx]->getCurrentPosition());
        }
    }
    else if (fingerCmd.equalsIgnoreCase("STATUS")) {
        Serial.print(F("Finger "));
        Serial.print(fingerIdx);
        Serial.print(F(": "));
        _fingers[fingerIdx]->printDebug(Serial);
        Serial.println();
    }
    else if (fingerCmd.equalsIgnoreCase("FSR")) {
        Serial.print(F("Finger "));
        Serial.print(fingerIdx);
        Serial.print(F(" FSR: "));
        Serial.println(_fingers[fingerIdx]->getFSR(), 3);
    }
    else if (fingerCmd.equalsIgnoreCase("REFLEX")) {
        Serial.print(F("Triggering reflex for finger "));
        Serial.println(fingerIdx);
        _fingers[fingerIdx]->reflexGrip();
    }
    else if (fingerCmd.startsWith("PID")) {
        int cmdSpaceIdx = fingerCmd.indexOf(' ');
        if (cmdSpaceIdx > 0) {
            String pidParams = fingerCmd.substring(cmdSpaceIdx + 1);
            float kp, ki, kd;
            
            // Parse PID params (kp,ki,kd)
            int comma1 = pidParams.indexOf(',');
            int comma2 = pidParams.indexOf(',', comma1 + 1);
            
            if (comma1 > 0 && comma2 > comma1) {
                kp = pidParams.substring(0, comma1).toFloat();
                ki = pidParams.substring(comma1 + 1, comma2).toFloat();
                kd = pidParams.substring(comma2 + 1).toFloat();
                
                _fingers[fingerIdx]->setPIDGains(kp, ki, kd);
                
                Serial.print(F("Finger "));
                Serial.print(fingerIdx);
                Serial.print(F(" PID gains set to: "));
                Serial.print(kp); Serial.print(F(","));
                Serial.print(ki); Serial.print(F(","));
                Serial.println(kd);
            } else {
                Serial.println(F("Invalid PID format. Use: FINGER <idx> PID kp,ki,kd"));
            }
        } else {
            Serial.println(F("Usage: FINGER <idx> PID kp,ki,kd"));
        }
    }
    else {
        Serial.println(F("Available finger commands: POS, STATUS, FSR, REFLEX, PID"));
    }
}

void CommandInterface::handleEMGCommand(const String& params) {
    if (!_emgProcessor) return;
    
    if (params.equalsIgnoreCase("STATUS")) {
        printEMGVisual();
    }
    else if (params.equalsIgnoreCase("STATS")) {
        printEMGStats();
    }
    else if (params.equalsIgnoreCase("CALIBRATE")) {
        Serial.println(F("Calibrating EMG resting levels..."));
        _emgProcessor->calibrateRest();
        Serial.println(F("Calibration complete"));
    }
    else if (params.startsWith("DMA")) {
        int spaceIdx = params.indexOf(' ');
        if (spaceIdx > 0) {
            String dmaParam = params.substring(spaceIdx + 1);
            bool enable = dmaParam.equalsIgnoreCase("ON") || dmaParam.equalsIgnoreCase("1");
            _emgProcessor->enableDMA(enable);
            Serial.print(F("EMG DMA mode: "));
            Serial.println(enable ? F("Enabled") : F("Disabled"));
        }
    }
    else if (params.equalsIgnoreCase("FEATURES")) {
        float features[NUM_EMG_CHANNELS];
        _emgProcessor->getFeatureVector(features);
        
        Serial.print(F("EMG Features: "));
        for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
            Serial.print(features[i], 3);
            Serial.print(i < NUM_EMG_CHANNELS - 1 ? F(", ") : F(""));
        }
        Serial.println();
    }
    else if (params.startsWith("THRESH")) {
        int spaceIdx = params.indexOf(' ');
        if (spaceIdx > 0) {
            float thresh = params.substring(spaceIdx + 1).toFloat();
            _emgProcessor->setAdaptiveThreshold(thresh);
            Serial.print(F("EMG adaptive threshold set to: "));
            Serial.println(thresh, 3);
        }
    }
    else {
        Serial.println(F("Available EMG commands: STATUS, STATS, CALIBRATE, DMA, FEATURES, THRESH"));
    }
}

void CommandInterface::handleProfileCommand(const String& params) {
    if (!_fingers) return;
    
    if (params.equalsIgnoreCase("NONE")) {
        for (uint8_t i = 0; i < NUM_FINGERS; i++) {
            _fingers[i]->setMotionProfile(MotionProfileType::None);
        }
        Serial.println(F("Motion profiles disabled"));
    }
    else if (params.equalsIgnoreCase("TRAP")) {
        for (uint8_t i = 0; i < NUM_FINGERS; i++) {
            _fingers[i]->setMotionProfile(MotionProfileType::Trapezoidal);
        }
        Serial.println(F("Trapezoidal motion profiles enabled"));
    }
    else if (params.equalsIgnoreCase("SCURVE")) {
        for (uint8_t i = 0; i < NUM_FINGERS; i++) {
            _fingers[i]->setMotionProfile(MotionProfileType::SCurve);
        }
        Serial.println(F("S-curve motion profiles enabled"));
    }
    else if (params.startsWith("SPEED")) {
        int spaceIdx = params.indexOf(' ');
        if (spaceIdx > 0) {
            float speed = params.substring(spaceIdx + 1).toFloat();
            if (_graspManager) {
                _graspManager->setGripTransitionSpeed(speed);
                Serial.print(F("Grip transition speed set to: "));
                Serial.println(speed);
            }
        }
    }
    else if (params.equalsIgnoreCase("SOFT")) {
        for (uint8_t i = 0; i < NUM_FINGERS; i++) {
            _fingers[i]->enableSoftContact(true);
        }
        Serial.println(F("Soft contact enabled"));
    }
    else if (params.equalsIgnoreCase("HARD")) {
        for (uint8_t i = 0; i < NUM_FINGERS; i++) {
            _fingers[i]->enableSoftContact(false);
        }
        Serial.println(F("Soft contact disabled (hard contact)"));
    }
    else if (params.equalsIgnoreCase("CASCADE")) {
        if (_graspManager) {
            _graspManager->enableNaturalTiming(true);
            Serial.println(F("Natural finger cascade timing enabled"));
        }
    }
    else if (params.equalsIgnoreCase("SYNC")) {
        if (_graspManager) {
            _graspManager->enableNaturalTiming(false);
            Serial.println(F("Synchronized finger movement enabled"));
        }
    }
    else {
        Serial.println(F("Available profile commands: NONE, TRAP, SCURVE, SPEED, SOFT, HARD, CASCADE, SYNC"));
    }
}

void CommandInterface::handleCalibrationCommand(const String& params) {
    if (!_calibration) return;
    
    if (params.equalsIgnoreCase("SAVE")) {
        _calibration->saveToEEPROM();
        Serial.println(F("Calibration saved to EEPROM"));
    }
    else if (params.equalsIgnoreCase("LOAD")) {
        bool success = _calibration->loadFromEEPROM();
        if (success) {
            Serial.println(F("Calibration loaded from EEPROM"));
        } else {
            Serial.println(F("Failed to load calibration from EEPROM"));
        }
    }
    else if (params.equalsIgnoreCase("EMG")) {
        Serial.println(F("Calibrating EMG..."));
        // Placeholder for calibration sequence
        _emgProcessor->calibrateRest();
        Serial.println(F("EMG calibration complete"));
    }
    else {
        Serial.println(F("Available calibration commands: SAVE, LOAD, EMG"));
    }
}

void CommandInterface::handleReflexCommand(const String& params) {
    if (!_reflexEngine) return;
    
    if (params.equalsIgnoreCase("ON")) {
        _reflexEngine->enable(true);
        Serial.println(F("Reflex system enabled"));
    }
    else if (params.equalsIgnoreCase("OFF")) {
        _reflexEngine->enable(false);
        Serial.println(F("Reflex system disabled"));
    }
    else if (params.startsWith("THRESH")) {
        int spaceIdx = params.indexOf(' ');
        if (spaceIdx > 0) {
            float thresh = params.substring(spaceIdx + 1).toFloat();
            _reflexEngine->setSlipThreshold(thresh);
            Serial.print(F("Slip threshold set to: "));
            Serial.println(thresh, 3);
        }
    }
    else if (params.equalsIgnoreCase("STATS")) {
        uint32_t count;
        float avgTime;
        _reflexEngine->getReflexStats(&count, &avgTime);
        
        Serial.print(F("Reflexes triggered: "));
        Serial.print(count);
        Serial.print(F(", Avg response time: "));
        Serial.print(avgTime, 3);
        Serial.println(F("ms"));
    }
    else {
        Serial.println(F("Available reflex commands: ON, OFF, THRESH, STATS"));
    }
}

void CommandInterface::handleSensorCommand(const String& params) {
    if (!_sensorFusion) return;
    
    if (params.equalsIgnoreCase("STATUS")) {
        _sensorFusion->printDebug(Serial);
        Serial.println();
    }
    else if (params.equalsIgnoreCase("ORIENT")) {
        float roll, pitch, yaw;
        _sensorFusion->getHandOrientation(&roll, &pitch, &yaw);
        
        Serial.print(F("Orientation - Roll: "));
        Serial.print(roll, 1);
        Serial.print(F("°, Pitch: "));
        Serial.print(pitch, 1);
        Serial.print(F("°, Yaw: "));
        Serial.print(yaw, 1);
        Serial.println(F("°"));
    }
    else if (params.equalsIgnoreCase("RISK")) {
        Serial.print(F("Slip risk: "));
        Serial.print(_sensorFusion->getSlipRisk(), 3);
        Serial.print(F(", Grip stability: "));
        Serial.print(_sensorFusion->getGripStability(), 3);
        Serial.print(F(", Recommended force: "));
        Serial.println(_sensorFusion->getRecommendedGripForce(), 3);
    }
    else {
        Serial.println(F("Available sensor commands: STATUS, ORIENT, RISK"));
    }
}

void CommandInterface::handleSystemCommand(const String& params) {
    if (params.equalsIgnoreCase("STATUS")) {
        // Print overall system status
        Serial.println(F("--- Bionic Hand System Status ---"));
        
        if (_graspManager) {
            Serial.print(F("Grasp: "));
            Serial.print((int)_graspManager->getGraspType());
            Serial.print(F(", Force: "));
            Serial.println(_graspManager->getGripForce(), 2);
        }
        
        if (_fingers) {
            for (uint8_t i = 0; i < NUM_FINGERS; i++) {
                Serial.print(F("Finger ")); Serial.print(i);
                Serial.print(F(": Pos=")); Serial.print(_fingers[i]->getCurrentPosition());
                Serial.print(F(", FSR=")); Serial.print(_fingers[i]->getFSR(), 2);
                Serial.print(F(", State=")); Serial.println((int)_fingers[i]->getState());
            }
        }
        
        if (_reflexEngine) {
            uint32_t count;
            float avgTime;
            _reflexEngine->getReflexStats(&count, &avgTime);
            
            Serial.print(F("Reflexes: "));
            Serial.print(count);
            Serial.print(F(", Response: "));
            Serial.print(avgTime, 1);
            Serial.println(F(" ms"));
        }
        
        if (_sensorFusion) {
            Serial.print(F("Slip risk: "));
            Serial.print(_sensorFusion->getSlipRisk(), 2);
            Serial.print(F(", Context: "));
            Serial.println(_sensorFusion->getActivityContext());
        }
        
        Serial.println(F("-------------------------------"));
    }
    else if (params.equalsIgnoreCase("MEMSTAT")) {
        Serial.print(F("Free RAM: "));
        Serial.print(freeMemory());
        Serial.println(F(" bytes"));
    }
    else if (params.equalsIgnoreCase("VERSION")) {
        Serial.println(F("Bionic Hand Firmware V3.0"));
        Serial.println(F("(c) 2025 Ottobock/Open Bionics"));
        Serial.print(F("Compiled: "));
        Serial.print(F(__DATE__));
        Serial.print(F(" "));
        Serial.println(F(__TIME__));
    }
    else if (params.equalsIgnoreCase("RESET")) {
        Serial.println(F("Resetting system in 3 seconds..."));
        delay(3000);
        NVIC_SystemReset(); // Reset Teensy
    }
    else {
        Serial.println(F("Available system commands: STATUS, MEMSTAT, VERSION, RESET"));
    }
}

// External function to get free memory
extern "C" char* sbrk(int incr);
int freeMemory() {
    char top;
    return &top - reinterpret_cast<char*>(sbrk(0));
}

void CommandInterface::handleDebugCommand(const String& params) {
    if (params.equalsIgnoreCase("EMG")) {
        printEMGVisual();
    }
    else if (params.equalsIgnoreCase("FINGERS")) {
        for (uint8_t i = 0; i < NUM_FINGERS; i++) {
            _fingers[i]->printDebug(Serial);
            Serial.println();
        }
    }
    else if (params.equalsIgnoreCase("ALL")) {
        Serial.println(F("--- DEBUG INFORMATION ---"));
        
        // EMG
        printEMGVisual();
        
        // Fingers
        for (uint8_t i = 0; i < NUM_FINGERS; i++) {
            _fingers[i]->printDebug(Serial);
            Serial.println();
        }
        
        // Grasp
        if (_graspManager) {
            _graspManager->printDebug(Serial);
            Serial.println();
        }
        
        // Sensor fusion
        if (_sensorFusion) {
            _sensorFusion->printDebug(Serial);
            Serial.println();
        }
        
        Serial.println(F("------------------------"));
    }
    else {
        Serial.println(F("Available debug commands: EMG, FINGERS, ALL"));
    }
}

void CommandInterface::printEMGVisual() {
    if (!_emgProcessor) return;
    
    float features[NUM_EMG_CHANNELS];
    _emgProcessor->getFeatureVector(features);
    
    Serial.println(F("EMG Levels:"));
    const int barWidth = 30;
    
    for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
        Serial.print(F("CH"));
        Serial.print(i);
        Serial.print(F(": "));
        
        // Generate bar graph
        int numBars = features[i] * barWidth;
        for (int j = 0; j < barWidth; j++) {
            Serial.print(j < numBars ? F("|") : F(" "));
        }
        Serial.print(F(" "));
        Serial.println(features[i], 3);
    }
}

void CommandInterface::printEMGStats() {
    if (!_emgProcessor) return;
    
    float features[NUM_EMG_CHANNELS];
    _emgProcessor->getFeatureVector(features);
    
    Serial.println(F("EMG Channel Statistics:"));
    
    for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
        Serial.print(F("CH"));
        Serial.print(i);
        Serial.print(F(": Feature="));
        Serial.print(features[i], 3);
        Serial.print(F(", RMS="));
        Serial.print(_emgProcessor->getRMS(i), 3);
        Serial.print(F(", ZC="));
        Serial.print(_emgProcessor->getZeroCrossings(i), 1);
        Serial.print(F(", WL="));
        Serial.print(_emgProcessor->getWaveformLength(i), 3);
        Serial.print(F(", Active="));
        Serial.println(_emgProcessor->isActivityDetected(i) ? F("Yes") : F("No"));
    }
}

void CommandInterface::printHelp() {
    Serial.println(F("--- Bionic Hand Firmware V3.0 Help ---"));
    Serial.println(F("Available commands:"));
    Serial.println(F("GRIP <type> - Set grip type (OPEN, CLOSE, POWER, SPHERICAL, PINCH, HOOK, LATERAL)"));
    Serial.println(F("GRIP FORCE <val> - Set grip force (0.0-1.0)"));
    Serial.println(F("FINGER <idx> <cmd> - Control individual finger (POS, STATUS, FSR, REFLEX, PID)"));
    Serial.println(F("EMG <cmd> - EMG commands (STATUS, STATS, CALIBRATE, DMA, FEATURES, THRESH)"));
    Serial.println(F("PROFILE <type> - Set motion profile (NONE, TRAP, SCURVE, SPEED, SOFT, HARD, CASCADE, SYNC)"));
    Serial.println(F("CALIBRATE <cmd> - Calibration commands (SAVE, LOAD, EMG)"));
    Serial.println(F("REFLEX <cmd> - Reflex control (ON, OFF, THRESH, STATS)"));
    Serial.println(F("SENSOR <cmd> - Sensor fusion (STATUS, ORIENT, RISK)"));
    Serial.println(F("SYSTEM <cmd> - System commands (STATUS, MEMSTAT, VERSION, RESET)"));
    Serial.println(F("DEBUG <target> - Debug information (EMG, FINGERS, ALL)"));
    Serial.println(F("HELP - Display this help message"));
    Serial.println(F("----------------------------------------"));
}