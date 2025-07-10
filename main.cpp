/**
 * @file main.cpp
 * @brief Main program with ReflexEngine integration
 */

#include <Arduino.h>
#include <IntervalTimer.h>
#include "config.h"
#include "EMGProcessor.h"
#include "Finger.h"
#include "GraspManager.h"
#include "CommandInterface.h"
#include "SensorFusion.h"
#include "ReflexEngine.h"
#include "PowerMonitor.h"
#include "Calibration.h"

// Global component instances
EMGProcessor emgProcessor;

// Create finger instances
Finger finger0(FINGER_PWM_PINS[0], FSR_PINS[0]);
Finger finger1(FINGER_PWM_PINS[1], FSR_PINS[1]);
Finger finger2(FINGER_PWM_PINS[2], FSR_PINS[2]);
Finger finger3(FINGER_PWM_PINS[3], FSR_PINS[3]);

// Create finger pointer array for ReflexEngine
Finger* fingers[NUM_FINGERS] = {&finger0, &finger1, &finger2, &finger3};

GraspManager graspManager;
SensorFusion sensorFusion;
CommandInterface cmdInterface;
ReflexEngine reflexEngine;
PowerMonitor powerMonitor(BATTERY_VOLTAGE_PIN);
Calibration calibration;

// Timers
IntervalTimer emgTimer;
IntervalTimer reflexTimer;

// Forward declarations
void emgSampleISR();
void reflexUpdateISR();
float readFSR(uint8_t index);

void setup() {
    // Initialize serial
    Serial.begin(SERIAL_BAUD);
    delay(100); // Brief delay for stability
    Serial.println(F("Bionic Hand Firmware v2.0"));
    
    // Initialize components
    emgProcessor.begin();
    
    // Initialize fingers
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
        fingers[i]->begin();
    }
    
    // Initialize grasp manager
    graspManager.begin(fingers, NUM_FINGERS);
    
    // Initialize sensor fusion
    sensorFusion.begin();
    
    // Initialize power monitoring
    powerMonitor.begin();
    
    // Initialize calibration and load settings
    calibration.begin(&emgProcessor, fingers);
    calibration.loadFromEEPROM();
    
    // Initialize ReflexEngine
    reflexEngine.begin(fingers, NUM_FINGERS);
    reflexEngine.setFSRReadFunction(readFSR);
    reflexEngine.setSlipThreshold(FSR_SLIP_DFDT_THRESH);
    
    // Initialize command interface
    cmdInterface.begin(&emgProcessor, fingers, NUM_FINGERS, &graspManager, 
                     &reflexEngine, &sensorFusion, &powerMonitor, &calibration);
    
    // Start timers
    emgTimer.begin(emgSampleISR, 1000000 / EMG_SAMPLE_HZ); // EMG sampling timer
    reflexTimer.begin(reflexUpdateISR, 1000); // 1000 us = 1 kHz reflex update rate
    
    Serial.println(F("System initialization complete"));
}

void loop() {
    // Update EMG processing
    emgProcessor.update();
    
    // Get EMG feature vector
    float emgFeatures[NUM_EMG_CHANNELS];
    emgProcessor.getFeatureVector(emgFeatures);
    
    // Get FSR values for sensor fusion
    float fsrValues[NUM_FINGERS];
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
        fsrValues[i] = fingers[i]->getFSR();
    }
    
    // Update sensor fusion
    sensorFusion.setEMGFeatures(emgFeatures, NUM_EMG_CHANNELS);
    sensorFusion.setFSRValues(fsrValues, NUM_FINGERS);
    sensorFusion.update();
    
    // Update grasp manager with grip force from sensor fusion
    graspManager.update(emgFeatures, sensorFusion.getGripForceMultiplier());
    
    // Update finger control
    static uint32_t lastFingerUpdate = 0;
    uint32_t now = millis();
    if (now - lastFingerUpdate >= 10) { // 10ms = 100Hz update rate
        float dt = (now - lastFingerUpdate) / 1000.0f;
        
        // Update each finger
        for (uint8_t i = 0; i < NUM_FINGERS; i++) {
            fingers[i]->update(dt);
        }
        
        lastFingerUpdate = now;
    }
    
    // Update power monitoring
    static uint32_t lastPowerUpdate = 0;
    if (now - lastPowerUpdate >= 1000) { // 1 second update
        powerMonitor.update();
        
        // Check for low power condition
        if (powerMonitor.getPowerState() == PowerState::CRITICAL) {
            // Implement power-saving measures
            reflexEngine.setLowPowerMode(true);
        } else {
            reflexEngine.setLowPowerMode(false);
        }
        
        lastPowerUpdate = now;
    }
    
    // Process serial commands
    cmdInterface.update();
}

void emgSampleISR() {
    emgProcessor.sampleISR();
}

void reflexUpdateISR() {
    reflexEngine.update();
}

/**
 * @brief Read FSR value for the ReflexEngine
 * @param index Finger index (0-3)
 * @return Normalized FSR value (0.0-1.0)
 */
float readFSR(uint8_t index) {
    if (index >= NUM_FINGERS) {
        return 0.0f;
    }
    
    // Read from appropriate analog pin
    int rawValue = analogRead(FSR_PINS[index]);
    
    // Normalize to 0.0-1.0 range
    return rawValue / 1023.0f;
}