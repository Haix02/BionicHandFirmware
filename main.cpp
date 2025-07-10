/**
 * @file main.cpp
 * @brief Main firmware for Teensy 4.1 bionic hand with integrated ReflexEngine
 */

#include <Arduino.h>
#include <IntervalTimer.h>
#include "config.h"
#include "Finger.h"
#include "GraspManager.h"
#include "EMGProcessor.h"
#include "CommandInterface.h"
#include "SensorFusion.h"
#include "ReflexEngine.h" // Add include for ReflexEngine

// Existing global object declarations (unchanged)
EMGProcessor emgProcessor;
Finger finger1(SERVO_PIN_1, FSR_PIN_1);
Finger finger2(SERVO_PIN_2, FSR_PIN_2);
Finger finger3(SERVO_PIN_3, FSR_PIN_3);
Finger finger4(SERVO_PIN_4, FSR_PIN_4);
GraspManager graspManager;
CommandInterface cmdInterface;
SensorFusion sensorFusion;

// Create array of Finger pointers for ReflexEngine
Finger* fingers[NUM_FINGERS] = {&finger1, &finger2, &finger3, &finger4};

// ReflexEngine instance
ReflexEngine reflexEngine;

// IntervalTimer for reflex loop
IntervalTimer reflexTimer;

// Function prototype for FSR reading function
float readFSR(uint8_t index);

// Reflex update function for timer
void updateReflex() {
  reflexEngine.update();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Bionic Hand Firmware - ReflexEngine Integration");
  
  // Initialize fingers
  for (int i = 0; i < NUM_FINGERS; i++) {
    fingers[i]->begin();
  }
  
  // Initialize other components (unchanged)
  emgProcessor.begin();
  graspManager.begin(fingers, NUM_FINGERS);
  sensorFusion.begin();
  
  // Initialize ReflexEngine
  reflexEngine.begin(fingers, NUM_FINGERS);
  reflexEngine.setFSRReadFunction(readFSR);
  reflexEngine.setSlipThreshold(-0.2f); // Negative dF/dt threshold
  
  // Start reflex timer at 1kHz
  reflexTimer.begin(updateReflex, 1000); // 1000 microseconds = 1kHz
  
  // Initialize command interface with reflexEngine pointer
  cmdInterface.begin(&emgProcessor, &graspManager, &sensorFusion, &reflexEngine);
  
  Serial.println("ReflexEngine initialized and running at 1kHz");
}

void loop() {
  // Existing main loop code (unchanged)
  emgProcessor.update();
  graspManager.update();
  sensorFusion.update();
  
  // Update fingers
  for (int i = 0; i < NUM_FINGERS; i++) {
    fingers[i]->update();
  }
  
  // Process commands
  cmdInterface.update();
  
  // No need to call reflexEngine.update() here as it runs on the timer
}

/**
 * @brief Read FSR value for given finger index
 * @param index Finger index (0-3)
 * @return Normalized FSR value (0.0-1.0)
 */
float readFSR(uint8_t index) {
  // Map index to correct FSR pin
  static const uint8_t fsrPins[NUM_FINGERS] = {FSR_PIN_1, FSR_PIN_2, FSR_PIN_3, FSR_PIN_4};
  
  // Check valid range
  if (index >= NUM_FINGERS) {
    return 0.0f;
  }
  
  // Read analog value and normalize to 0.0-1.0
  float value = analogRead(fsrPins[index]) / 1023.0f;
  
  return value;
}