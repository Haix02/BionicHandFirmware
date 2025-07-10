/**
 * @file CommandInterface.h
 * @brief Command interface for debugging and control
 */

#pragma once

#include <Arduino.h>

class EMGProcessor;
class GraspManager;
class SensorFusion;
class ReflexEngine; // Add ReflexEngine forward declaration

class CommandInterface {
public:
  // Existing declarations (unchanged)
  CommandInterface();
  void begin(EMGProcessor* emgProcessor, GraspManager* graspManager, 
             SensorFusion* sensorFusion, ReflexEngine* reflexEngine = nullptr);
  void update();
  
private:
  // Existing members (unchanged)
  EMGProcessor* _emgProcessor;
  GraspManager* _graspManager;
  SensorFusion* _sensorFusion;
  ReflexEngine* _reflexEngine; // Add ReflexEngine pointer
  
  // Command buffer and processing methods
  char _cmdBuffer[64];
  uint8_t _cmdIndex;
  
  void processCommand();
  void printHelp();
  // Other existing methods (unchanged)
};