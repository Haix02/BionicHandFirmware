/**
 * @file main.cpp
 * @brief Main program for Teensy 4.1 bionic hand firmware
 * @version 2.0
 */

#include <Arduino.h>
#include <IntervalTimer.h>
#include <EEPROM.h>
#include "config.h"
#include "Finger.h"
#include "EMGProcessor.h"
#include "GraspManager.h"
#include "ReflexEngine.h"
#include "SensorFusion.h"
#include "CommandInterface.h"
#include "Calibration.h"
#include "PowerMonitor.h"

// Pin definitions (if not in config.h)
#ifndef LED_PIN
#define LED_PIN 13
#endif

// Global system components
EMGProcessor emgProcessor;

// Create 4 finger instances
Finger finger0(FINGER_PWM_PINS[0], FINGER_FSR_PINS[0]);
Finger finger1(FINGER_PWM_PINS[1], FINGER_FSR_PINS[1]);
Finger finger2(FINGER_PWM_PINS[2], FINGER_FSR_PINS[2]);
Finger finger3(FINGER_PWM_PINS[3], FINGER_FSR_PINS[3]);

// Finger array for other components
Finger* fingers[NUM_FINGERS] = {&finger0, &finger1, &finger2, &finger3};

// System components
GraspManager graspManager;
ReflexEngine reflexEngine;
SensorFusion sensorFusion;
CommandInterface commandInterface;
Calibration calibration;
PowerMonitor powerMonitor(BATTERY_VOLTAGE_PIN);

// Timer interrupts
IntervalTimer emgTimer;
IntervalTimer reflexTimer;

// Timing variables
uint32_t lastUpdateTime = 0;
uint32_t lastHeartbeatTime = 0;
bool ledState = false;

// Function prototypes
void emgSampleISR();
void reflexUpdateISR();
float readFSR(uint8_t index);

void setup() {
    // Initialize serial
    Serial.begin(115200);
    delay(300); // Brief delay for stability
    
    // Initialize LED pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // Turn on during initialization
    
    Serial.println(F("Bionic Hand Firmware v2.0"));
    Serial.println(F("Initializing system..."));
    
    // Initialize fingers
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
        fingers[i]->begin();
    }
    
    // Initialize EMG processor
    emgProcessor.begin();
    
    // Initialize grasp manager
    graspManager.begin(fingers, NUM_FINGERS);
    
    // Initialize reflex engine
    reflexEngine.begin(fingers, NUM_FINGERS);
    reflexEngine.setFSRReadFunction(readFSR);
    reflexEngine.setSlipThreshold(REFLEX_SLIP_THRESHOLD);
    
    // Initialize sensor fusion
    sensorFusion.begin();
    
    // Initialize power monitor
    powerMonitor.begin();
    
    // Initialize calibration
    calibration.begin(&emgProcessor, fingers, NUM_FINGERS);
    if (!calibration.loadFromEEPROM()) {
        calibration.resetToDefaults();
        calibration.saveToEEPROM();
    }
    
    // Initialize command interface
    commandInterface.begin(&emgProcessor, &graspManager, &reflexEngine, 
                          &sensorFusion, &powerMonitor, &calibration);
    
    // Start timer interrupts
    emgTimer.begin(emgSampleISR, 1000000 / EMG_SAMPLE_HZ); // EMG sampling
    reflexTimer.begin(reflexUpdateISR, 1000); // 1kHz reflex update
    
    // Initial grasp - open
    graspManager.executeGrasp("open");
    
    Serial.println(F("System initialized"));
    digitalWrite(LED_PIN, LOW);
}

void loop() {
    // Current time
    uint32_t currentTime = millis();
    
    // Calculate time delta
    float dt = (currentTime - lastUpdateTime) / 1000.0f; // seconds
    lastUpdateTime = currentTime;
    
    // Update EMG processing
    emgProcessor.update();
    
    // Get EMG feature vector
    float emgVector[NUM_EMG_CHANNELS];
    emgProcessor.getFeatureVector(emgVector);
    
    // Get FSR values for sensor fusion
    float fsrValues[NUM_FINGERS];
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
        fsrValues[i] = fingers[i]->getFSR();
    }
    
    // Update sensor fusion
    sensorFusion.setEMGValues(emgVector, NUM_EMG_CHANNELS);
    sensorFusion.setFSRValues(fsrValues, NUM_FINGERS);
    sensorFusion.update();
    
    // Update grasp manager with grip force from sensor fusion
    float forceMultiplier = sensorFusion.getForceMultiplier();
    graspManager.update(dt);
    graspManager.setGraspForce(forceMultiplier);
    
    // Update fingers
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
        fingers[i]->update(dt);
    }
    
    // Update power monitor (1Hz)
    static uint32_t lastPowerCheck = 0;
    if (currentTime - lastPowerCheck >= 1000) {
        powerMonitor.update();
        
        // Check for low power condition
        if (powerMonitor.getPowerState() == PowerState::CRITICAL) {
            reflexEngine.setLowPowerMode(true);
            graspManager.setGraspForce(0.7f); // Reduce grip force in low power
            
            Serial.println(F("WARNING: Battery critically low!"));
        } else {
            reflexEngine.setLowPowerMode(false);
        }
        
        lastPowerCheck = currentTime;
    }
    
    // Process serial commands
    commandInterface.update();
    
    // Heartbeat LED - 1Hz blink
    if (currentTime - lastHeartbeatTime >= 1000) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        lastHeartbeatTime = currentTime;
    }
}

void emgSampleISR() {
    // Called at EMG_SAMPLE_HZ frequency
    emgProcessor.sampleISR();
}

void reflexUpdateISR() {
    // Called at 1kHz for reflex system
    reflexEngine.update();
}

float readFSR(uint8_t index) {
    // Validate index
    if (index >= NUM_FINGERS) return 0.0f;
    
    // Read FSR value directly from finger object
    return fingers[index]->getFSR();
}