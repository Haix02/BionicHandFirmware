/**
 * @file main.cpp
 * @brief Teensy 4.1 main firmware for bionic hand, Version 3.0
 * @author Ottobock/Open Bionics Firmware Team
 * @version 3.0
 */

#include <Arduino.h>
#include <IntervalTimer.h>
#include "config.h"
#include "EMGProcessor.h"
#include "Finger.h"
#include "GraspManager.h"
#include "ReflexEngine.h" // V3 enhancement
#include "SensorFusion.h" // V3 enhancement
#include "CommandInterface.h"
#include "Calibration.h"

// ----------------------------
// ---- Global Objects --------
// ----------------------------

EMGProcessor emgProcessor;
Finger* fingers[NUM_FINGERS];
Finger finger_objs[NUM_FINGERS] = {
    Finger(FINGER_PWM_PINS[0], FSR_PINS[0]),
    Finger(FINGER_PWM_PINS[1], FSR_PINS[1]),
    Finger(FINGER_PWM_PINS[2], FSR_PINS[2]),
    Finger(FINGER_PWM_PINS[3], FSR_PINS[3]),
};
GraspManager graspManager(fingers);
ReflexEngine reflexEngine;     // V3 enhancement
SensorFusion sensorFusion;     // V3 enhancement
CommandInterface cmdInterface;
Calibration calibration(&emgProcessor, fingers);

// Timers
IntervalTimer emgTimer, fsrTimer;

// EMG feature vector buffer
float emgFeature[NUM_EMG_CHANNELS];

// FSR values buffer
float fsrValues[NUM_FINGERS];

// Heartbeat LED
volatile bool led_state = false;
IntervalTimer ledTimer;

// Forward declarations
void emgSampleISR();
void fsrSampleISR();
void ledHeartbeatISR();

// ----------------------------
// ---- Setup -----------------
// ----------------------------

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 2000);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Bind finger pointers for GraspManager
    for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
        fingers[i] = &finger_objs[i];
        fingers[i]->begin();
        
        // V3: Set motion profile for smooth movement
        fingers[i]->setMotionProfile(MotionProfileType::SCurve);
        fingers[i]->enableSoftContact(true);
    }

    // Initialize EMG processor
    emgProcessor.begin();
    emgProcessor.enableDMA(true); // V3: Enable DMA for high-speed sampling

    // Initialize sensor fusion
    sensorFusion.begin();
    
    // Initialize reflex engine
    reflexEngine.begin(fingers);
    reflexEngine.enable(true);

    // Load calibration data
    calibration.loadFromEEPROM();

    // Initialize command interface
    cmdInterface.begin(&emgProcessor, fingers, &graspManager, &reflexEngine, &sensorFusion, &calibration);

    // Start timers
    emgTimer.begin(emgSampleISR, 1000000UL / EMG_SAMPLE_HZ); // 1ms = 1000Hz
    fsrTimer.begin(fsrSampleISR, 1000000UL / FSR_SAMPLE_HZ); // 10ms = 100Hz

    // LED heartbeat at 1 Hz (optional)
    ledTimer.begin(ledHeartbeatISR, 500000); // Toggle every 0.5s

    Serial.println(F("Bionic Hand Firmware V3.0 Started."));
}

// ----------------------------
// ---- Main Loop -------------
// ----------------------------

void loop() {
    static uint32_t lastPrint = 0;
    static uint32_t lastControl = 0;
    static uint32_t lastSensorFusion = 0;

    // ---- Process EMG pipeline ----
    emgProcessor.process();
    emgProcessor.getFeatureVector(emgFeature);

    // ---- Update sensor fusion ----
    uint32_t now = millis();
    if (now - lastSensorFusion >= 20) { // 50Hz update rate
        // Get FSR values for all fingers
        for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
            fsrValues[i] = fingers[i]->getFSR();
        }
        
        // Update sensor fusion with latest data
        sensorFusion.setEMGFeatures(emgFeature, NUM_EMG_CHANNELS);
        sensorFusion.setFSRValues(fsrValues, NUM_FINGERS);
        sensorFusion.update();
        
        lastSensorFusion = now;
    }
    
    // ---- Update reflex engine ----
    reflexEngine.update();

    // ---- High-level grasp update (10ms) ----
    if (now - lastControl >= 10) {
        // Use sensor fusion to inform grasp behavior
        float slipRisk = sensorFusion.getSlipRisk();
        float recommendedForce = sensorFusion.getRecommendedGripForce();
        
        // Update grasp manager with context-aware parameters
        graspManager.update(emgFeature, recommendedForce);

        // Finger motion update
        for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
            fingers[i]->update(10.0f); // dt = 10ms
        }
        
        lastControl = now;
    }

    // ---- Serial command interface ----
    if (Serial.available()) {
        cmdInterface.processSerial();
    }

    // ---- Debug output (optional, every 100ms) ----
    if (now - lastPrint >= 100) {
        // Print limited status
        Serial.print(F("Grasp:")); Serial.print((int)graspManager.getGraspType());
        Serial.print(F(" SlipRisk:")); Serial.print(sensorFusion.getSlipRisk(), 2);
        Serial.print(F(" Fingers:"));
        for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
            Serial.print(F(" ")); Serial.print(fingers[i]->getCurrentPosition());
        }
        Serial.println();
        lastPrint = now;
    }
}

// ----------------------------
// ---- Timer ISRs ------------
// ----------------------------

void emgSampleISR() {
    emgProcessor.sampleISR();
}

void fsrSampleISR() {
    // FSR sampling at 100Hz: all fingers
    for (uint8_t i = 0; i < NUM_FINGERS; ++i) {
        fingers[i]->sampleFSR();
    }
}

void ledHeartbeatISR() {
    led_state = !led_state;
    digitalWrite(LED_PIN, led_state);
}