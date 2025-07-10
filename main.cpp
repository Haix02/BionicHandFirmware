/**
 * @file main.cpp
 * @brief Main system controller for Teensy 4.1 bionic hand with power monitoring
 * @version 2.0
 * @author Haix02
 * @date 2025-07-10 23:16:56 UTC
 */

#include <Arduino.h>
#include <IntervalTimer.h>
#include "config.h"
#include "Finger.h"
#include "EMGProcessor.h"
#include "GraspManager.h"
#include "ReflexEngine.h"
#include "CommandInterface.h"
#include "PowerMonitor.h"
#include "Calibration.h"

// System status LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

// ================================
// GLOBAL SYSTEM COMPONENTS
// ================================

// Create 4 finger instances
Finger finger0(FINGER_PWM_PINS[0], FINGER_FSR_PINS[0]);
Finger finger1(FINGER_PWM_PINS[1], FINGER_FSR_PINS[1]);
Finger finger2(FINGER_PWM_PINS[2], FINGER_FSR_PINS[2]);
Finger finger3(FINGER_PWM_PINS[3], FINGER_FSR_PINS[3]);

// Finger array for system components
Finger* fingers[NUM_FINGERS] = {&finger0, &finger1, &finger2, &finger3};

// Core system components
EMGProcessor emgProcessor;
GraspManager graspManager;
ReflexEngine reflexEngine;
CommandInterface commandInterface;
PowerMonitor powerMonitor;
Calibration calibration;

// High-frequency timer interrupts
IntervalTimer reflexTimer;        // 1000 Hz for reflex system
IntervalTimer emgSampleTimer;     // 2000 Hz for EMG sampling
IntervalTimer emgProcessTimer;    // 500 Hz for EMG feature processing

// ================================
// TIMING AND STATE VARIABLES
// ================================

// System timing
uint32_t lastMainLoopTime = 0;
uint32_t lastHeartbeatTime = 0;
uint32_t lastStatusTime = 0;
uint32_t lastPowerCheckTime = 0;
bool heartbeatLedState = false;

// System state
bool systemInitialized = false;
uint32_t bootTime = 0;
bool powerSavingMode = false;
bool emergencyMode = false;

// Performance monitoring
volatile uint32_t reflexUpdateCount = 0;
volatile uint32_t emgSampleCount = 0;
volatile uint32_t emgProcessCount = 0;

// Power management state
PowerState lastPowerState = PowerState::NORMAL;
uint32_t lowPowerStartTime = 0;
uint32_t criticalPowerStartTime = 0;

// ================================
// FUNCTION PROTOTYPES
// ================================

void setupHardware();
void initializeComponents();
void setupTimers();
float readFSR(uint8_t index);
void reflexUpdateISR();
void emgSampleISR();
void emgProcessISR();
void updateMainLoop();
void updateHeartbeat();
void updatePowerManagement();
void handlePowerStateChange(PowerState newState);
void printSystemStatus();
void printPowerWarning(PowerState state);

// ================================
// SETUP FUNCTION
// ================================

void setup() {
    // Record boot time
    bootTime = millis();
    
    // Initialize serial communication
    Serial.begin(115200);
    delay(100); // Brief delay for serial stability
    
    // Print startup banner
    Serial.println(F("====================================="));
    Serial.println(F("  Bionic Hand Firmware v2.0"));
    Serial.println(F("  Teensy 4.1 Prosthetic Controller"));
    Serial.println(F("  Author: Haix02"));
    Serial.println(F("  Build: 2025-07-10 23:16:56 UTC"));
    Serial.println(F("====================================="));
    
    // Setup hardware
    setupHardware();
    
    // Initialize all components
    initializeComponents();
    
    // Setup high-frequency timers
    setupTimers();
    
    // System ready
    systemInitialized = true;
    
    Serial.println(F("System initialization complete"));
    Serial.println(F("Type 'HELP' for available commands"));
    
    // Initial power status check
    powerMonitor.update();
    powerMonitor.printStatus();
    
    Serial.print(F("> "));
    
    // Turn off LED after successful initialization
    digitalWrite(LED_BUILTIN, LOW);
}

// ================================
// MAIN LOOP
// ================================

void loop() {
    // Check if system is properly initialized
    if (!systemInitialized) {
        delay(10);
        return;
    }
    
    // Calculate loop timing
    uint32_t currentTime = millis();
    float deltaTime = (currentTime - lastMainLoopTime) / 1000.0f;
    lastMainLoopTime = currentTime;
    
    // Update power management (highest priority)
    updatePowerManagement();
    
    // Update main system components
    updateMainLoop();
    
    // Update heartbeat LED (1000ms interval)
    updateHeartbeat();
    
    // Print system status every 30 seconds (reduced frequency to save power)
    if (currentTime - lastStatusTime >= 30000) {
        printSystemStatus();
        lastStatusTime = currentTime;
    }
    
    // Process serial commands
    commandInterface.update();
    
    // Power-aware delay
    if (emergencyMode) {
        delay(50); // Longer delay in emergency mode to save power
    } else if (powerSavingMode) {
        delayMicroseconds(500); // Reduced processing in power saving mode
    } else {
        delayMicroseconds(100); // Normal operation
    }
}

// ================================
// HARDWARE SETUP
// ================================

void setupHardware() {
    Serial.println(F("Setting up hardware..."));
    
    // Configure system LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // On during initialization
    
    // Set ADC resolution for Teensy 4.1
    analogReadResolution(12);     // 12-bit ADC (0-4095)
    analogReadAveraging(4);       // Average 4 samples for noise reduction
    
    // Configure PWM frequency for servo control
    analogWriteFrequency(FINGER_PWM_PINS[0], 50); // 50Hz for servos
    analogWriteFrequency(FINGER_PWM_PINS[1], 50);
    analogWriteFrequency(FINGER_PWM_PINS[2], 50);
    analogWriteFrequency(FINGER_PWM_PINS[3], 50);
    
    Serial.println(F("Hardware setup complete"));
}

// ================================
// COMPONENT INITIALIZATION
// ================================

void initializeComponents() {
    Serial.println(F("Initializing system components..."));
    
    // Initialize power monitor first (critical for safety)
    Serial.print(F("  Power Monitor... "));
    if (powerMonitor.begin()) {
        Serial.println(F("OK"));
    } else {
        Serial.println(F("FAILED - continuing without power monitoring"));
    }
    
    // Initialize EMG processor
    Serial.print(F("  EMG Processor... "));
    emgProcessor.begin();
    Serial.println(F("OK"));
    
    // Initialize all fingers
    Serial.print(F("  Fingers... "));
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
        fingers[i]->begin();
    }
    Serial.println(F("OK"));
    
    // Initialize grasp manager
    Serial.print(F("  Grasp Manager... "));
    graspManager.begin(fingers, NUM_FINGERS);
    Serial.println(F("OK"));
    
    // Initialize calibration module
    Serial.print(F("  Calibration... "));
    calibration.begin(&emgProcessor, &graspManager, fingers, NUM_FINGERS);
    if (calibration.loadCalibration()) {
        Serial.println(F("OK - Calibration loaded"));
    } else {
        Serial.println(F("OK - Using defaults"));
    }
    
    // Initialize reflex engine
    Serial.print(F("  Reflex Engine... "));
    reflexEngine.begin(fingers, NUM_FINGERS);
    reflexEngine.setFSRReadFunction(readFSR);
    reflexEngine.setSlipThreshold(-0.2f);
    reflexEngine.enable(true);
    Serial.println(F("OK"));
    
    // Initialize command interface
    Serial.print(F("  Command Interface... "));
    commandInterface.begin(&emgProcessor, &graspManager, &reflexEngine, 
                          &powerMonitor, &calibration, fingers);
    Serial.println(F("OK"));
    
    // Set initial grasp to open
    graspManager.executeGrasp("open");
    
    Serial.println(F("Component initialization complete"));
}

// ================================
// TIMER SETUP
// ================================

void setupTimers() {
    Serial.println(F("Setting up timer interrupts..."));
    
    // Reflex engine timer - 1000 Hz (1000 microseconds)
    Serial.print(F("  Reflex Timer (1000Hz)... "));
    if (reflexTimer.begin(reflexUpdateISR, 1000)) {
        Serial.println(F("OK"));
    } else {
        Serial.println(F("FAILED"));
    }
    
    // EMG sampling timer - 2000 Hz (500 microseconds)
    Serial.print(F("  EMG Sample Timer (2000Hz)... "));
    if (emgSampleTimer.begin(emgSampleISR, 500)) {
        Serial.println(F("OK"));
    } else {
        Serial.println(F("FAILED"));
    }
    
    // EMG processing timer - 500 Hz (2000 microseconds)
    Serial.print(F("  EMG Process Timer (500Hz)... "));
    if (emgProcessTimer.begin(emgProcessISR, 2000)) {
        Serial.println(F("OK"));
    } else {
        Serial.println(F("FAILED"));
    }
    
    Serial.println(F("Timer setup complete"));
}

// ================================
// POWER MANAGEMENT
// ================================

void updatePowerManagement() {
    uint32_t currentTime = millis();
    
    // Update power monitor every 1000ms
    if (currentTime - lastPowerCheckTime >= 1000) {
        powerMonitor.update();
        
        PowerState currentPowerState = powerMonitor.getPowerState();
        
        // Check for power state changes
        if (currentPowerState != lastPowerState) {
            handlePowerStateChange(currentPowerState);
            lastPowerState = currentPowerState;
        }
        
        // Implement power management based on current state
        switch (currentPowerState) {
            case PowerState::CRITICAL:
                if (!emergencyMode) {
                    emergencyMode = true;
                    powerSavingMode = true;
                    criticalPowerStartTime = currentTime;
                    
                    // Emergency power saving measures
                    graspManager.setGraspForce(0.4f);  // 40% force
                    reflexEngine.setLowPowerMode(true);
                    
                    // Reduce timer frequencies
                    emgSampleTimer.end();
                    emgSampleTimer.begin(emgSampleISR, 1000); // Reduce to 1000Hz
                    
                    printPowerWarning(PowerState::CRITICAL);
                }
                break;
                
            case PowerState::LOW:
                if (!powerSavingMode) {
                    powerSavingMode = true;
                    emergencyMode = false;
                    lowPowerStartTime = currentTime;
                    
                    // Power saving measures
                    graspManager.setGraspForce(0.7f);  // 70% force
                    reflexEngine.setLowPowerMode(true);
                    
                    printPowerWarning(PowerState::LOW);
                }
                break;
                
            case PowerState::NORMAL:
            case PowerState::EXTERNAL:
                if (powerSavingMode || emergencyMode) {
                    powerSavingMode = false;
                    emergencyMode = false;
                    
                    // Restore full performance
                    graspManager.setGraspForce(1.0f);  // 100% force
                    reflexEngine.setLowPowerMode(false);
                    
                    // Restore normal timer frequencies
                    emgSampleTimer.end();
                    emgSampleTimer.begin(emgSampleISR, 500); // Restore to 2000Hz
                    
                    if (currentPowerState == PowerState::EXTERNAL) {
                        Serial.println(F("External power detected - full performance restored"));
                    } else {
                        Serial.println(F("Battery voltage recovered - full performance restored"));
                    }
                }
                break;
        }
        
        lastPowerCheckTime = currentTime;
    }
}

void handlePowerStateChange(PowerState newState) {
    Serial.print(F("Power state changed to: "));
    
    switch (newState) {
        case PowerState::NORMAL:
            Serial.println(F("NORMAL"));
            break;
        case PowerState::LOW:
            Serial.println(F("LOW BATTERY"));
            break;
        case PowerState::CRITICAL:
            Serial.println(F("CRITICAL BATTERY"));
            break;
        case PowerState::EXTERNAL:
            Serial.println(F("EXTERNAL POWER"));
            break;
    }
}

void printPowerWarning(PowerState state) {
    Serial.println(F("\n⚠️  POWER WARNING ⚠️"));
    
    switch (state) {
        case PowerState::LOW:
            Serial.println(F("Battery voltage is low!"));
            Serial.println(F("- Grip force reduced to 70%"));
            Serial.println(F("- Power saving mode activated"));
            Serial.println(F("- Please charge battery soon"));
            break;
            
        case PowerState::CRITICAL:
            Serial.println(F("CRITICAL BATTERY VOLTAGE!"));
            Serial.println(F("- Emergency power saving active"));
            Serial.println(F("- Grip force reduced to 40%"));
            Serial.println(F("- System performance limited"));
            Serial.println(F("- CHARGE BATTERY IMMEDIATELY"));
            break;
            
        default:
            break;
    }
    
    powerMonitor.printStatus();
    Serial.println();
}

// ================================
// FSR READING FUNCTION
// ================================

float readFSR(uint8_t index) {
    // Validate index range
    if (index >= NUM_FINGERS) {
        return 0.0f;
    }
    
    // Read FSR value directly from finger object
    return fingers[index]->getFSR();
}

// ================================
// INTERRUPT SERVICE ROUTINES
// ================================

void reflexUpdateISR() {
    // High-priority reflex processing at 1000 Hz
    reflexEngine.update();
    reflexUpdateCount++;
}

void emgSampleISR() {
    // High-speed EMG sampling at 2000 Hz (or 1000 Hz in power saving)
    emgProcessor.sampleISR();
    emgSampleCount++;
}

void emgProcessISR() {
    // EMG feature processing at 500 Hz
    emgProcessor.update();
    emgProcessCount++;
}

// ================================
// MAIN LOOP FUNCTIONS
// ================================

void updateMainLoop() {
    uint32_t currentTime = millis();
    static uint32_t lastFingerUpdate = 0;
    static uint32_t lastGraspUpdate = 0;
    
    // Adjust update frequencies based on power state
    uint32_t fingerUpdateInterval = emergencyMode ? 20 : 10;  // 50Hz or 100Hz
    uint32_t graspUpdateInterval = emergencyMode ? 40 : 20;   // 25Hz or 50Hz
    
    // Update fingers
    if (currentTime - lastFingerUpdate >= fingerUpdateInterval) {
        float dt = (currentTime - lastFingerUpdate) / 1000.0f;
        
        for (uint8_t i = 0; i < NUM_FINGERS; i++) {
            fingers[i]->update(dt);
        }
        
        lastFingerUpdate = currentTime;
    }
    
    // Update grasp manager
    if (currentTime - lastGraspUpdate >= graspUpdateInterval) {
        float dt = (currentTime - lastGraspUpdate) / 1000.0f;
        graspManager.update(dt);
        lastGraspUpdate = currentTime;
    }
}

void updateHeartbeat() {
    uint32_t currentTime = millis();
    uint32_t heartbeatInterval;
    
    // Adjust heartbeat based on power state
    switch (powerMonitor.getPowerState()) {
        case PowerState::CRITICAL:
            heartbeatInterval = 200;  // Fast blink for critical
            break;
        case PowerState::LOW:
            heartbeatInterval = 500;  // Medium blink for low
            break;
        case PowerState::EXTERNAL:
            heartbeatInterval = 2000; // Slow blink for external power
            break;
        default:
            heartbeatInterval = 1000; // Normal blink
            break;
    }
    
    // Toggle LED
    if (currentTime - lastHeartbeatTime >= heartbeatInterval) {
        heartbeatLedState = !heartbeatLedState;
        digitalWrite(LED_BUILTIN, heartbeatLedState);
        lastHeartbeatTime = currentTime;
    }
}

void printSystemStatus() {
    uint32_t uptime = millis() - bootTime;
    uint32_t uptimeSeconds = uptime / 1000;
    uint32_t uptimeMinutes = uptimeSeconds / 60;
    uint32_t uptimeHours = uptimeMinutes / 60;
    
    Serial.println(F("\n===== SYSTEM STATUS ====="));
    
    // Uptime
    Serial.print(F("Uptime: "));
    if (uptimeHours > 0) {
        Serial.print(uptimeHours);
        Serial.print(F("h "));
    }
    if (uptimeMinutes % 60 > 0) {
        Serial.print(uptimeMinutes % 60);
        Serial.print(F("m "));
    }
    Serial.print(uptimeSeconds % 60);
    Serial.println(F("s"));
    
    // Power status
    Serial.print(F("Power: "));
    Serial.print(powerMonitor.getVoltage(), 2);
    Serial.print(F("V ("));
    Serial.print(powerMonitor.getBatteryPercentage());
    Serial.print(F("%) - "));
    switch (powerMonitor.getPowerState()) {
        case PowerState::NORMAL: Serial.println(F("NORMAL")); break;
        case PowerState::LOW: Serial.println(F("LOW")); break;
        case PowerState::CRITICAL: Serial.println(F("CRITICAL")); break;
        case PowerState::EXTERNAL: Serial.println(F("EXTERNAL")); break;
    }
    
    // System mode
    Serial.print(F("Mode: "));
    if (emergencyMode) {
        Serial.println(F("EMERGENCY"));
    } else if (powerSavingMode) {
        Serial.println(F("POWER SAVING"));
    } else {
        Serial.println(F("NORMAL"));
    }
    
    // Performance counters
    Serial.print(F("Counters - Reflex: "));
    Serial.print(reflexUpdateCount);
    Serial.print(F(", EMG: "));
    Serial.print(emgSampleCount);
    Serial.print(F(", Process: "));
    Serial.println(emgProcessCount);
    
    // Finger status
    Serial.println(F("Fingers:"));
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
        Serial.print(F("  F"));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.print(fingers[i]->getCurrentPosition());
        Serial.print(F("° FSR="));
        Serial.print(fingers[i]->getFSR(), 2);
        Serial.print(F(" Reflex="));
        Serial.println(fingers[i]->isReflexActive() ? "Active" : "Idle");
    }
    
    // Current grasp mode
    Serial.print(F("Grasp: "));
    Serial.print(graspManager.getCurrentGraspMode());
    Serial.print(F(" ("));
    Serial.print(graspManager.getGraspForce() * 100, 0);
    Serial.println(F("%)"));
    
    Serial.println(F("========================\n"));
}