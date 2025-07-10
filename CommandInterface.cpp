/**
 * @file CommandInterface.cpp
 * @brief Command interface implementation with reflex debug commands
 */

#include "CommandInterface.h"
#include "EMGProcessor.h"
#include "GraspManager.h"
#include "SensorFusion.h"
#include "ReflexEngine.h"
#include <Arduino.h>

// Existing implementation (unchanged)

void CommandInterface::begin(EMGProcessor* emgProcessor, GraspManager* graspManager, 
                            SensorFusion* sensorFusion, ReflexEngine* reflexEngine) {
  _emgProcessor = emgProcessor;
  _graspManager = graspManager;
  _sensorFusion = sensorFusion;
  _reflexEngine = reflexEngine;
  
  _cmdIndex = 0;
  memset(_cmdBuffer, 0, sizeof(_cmdBuffer));
}

void CommandInterface::processCommand() {
  // Make command case-insensitive
  for (uint8_t i = 0; i < _cmdIndex; i++) {
    _cmdBuffer[i] = toupper(_cmdBuffer[i]);
  }
  
  // Original commands (unchanged)
  
  // Add new debug command for reflex testing
  if (strncmp(_cmdBuffer, "FORCE_SLIP ", 11) == 0 && _reflexEngine != nullptr) {
    // Extract finger index parameter
    int fingerIndex = atoi(_cmdBuffer + 11);
    
    // Validate finger index
    if (fingerIndex >= 0 && fingerIndex < NUM_FINGERS) {
      Serial.print("Simulating slip on finger ");
      Serial.println(fingerIndex);
      
      // Trigger manual reflex on the specified finger
      _reflexEngine->triggerManualReflex(fingerIndex);
    } else {
      Serial.println("Invalid finger index. Use 0-3.");
    }
  }
  
  // Add commands to adjust slip threshold
  else if (strncmp(_cmdBuffer, "SET_SLIP_THRESHOLD ", 19) == 0 && _reflexEngine != nullptr) {
    float threshold = atof(_cmdBuffer + 19);
    _reflexEngine->setSlipThreshold(threshold);
    Serial.print("Slip threshold set to ");
    Serial.println(threshold);
  }
  
  // Add command to toggle reflex system
  else if (strcmp(_cmdBuffer, "REFLEX_ON") == 0 && _reflexEngine != nullptr) {
    _reflexEngine->enable(true);
    Serial.println("Reflex system enabled");
  }
  else if (strcmp(_cmdBuffer, "REFLEX_OFF") == 0 && _reflexEngine != nullptr) {
    _reflexEngine->enable(false);
    Serial.println("Reflex system disabled");
  }
  
  // Help command update
  else if (strcmp(_cmdBuffer, "HELP") == 0) {
    printHelp();
  }
}

void CommandInterface::printHelp() {
  // Original help text (unchanged)
  
  // Add reflex system commands to help
  Serial.println("Reflex System Commands:");
  Serial.println("  FORCE_SLIP <index> - Simulate slip on finger (0-3)");
  Serial.println("  SET_SLIP_THRESHOLD <value> - Set slip detection threshold");
  Serial.println("  REFLEX_ON - Enable reflex system");
  Serial.println("  REFLEX_OFF - Disable reflex system");
}