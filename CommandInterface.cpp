/**
 * @file CommandInterface.cpp
 * @brief Enhanced serial command interface implementation
 * @version 2.0
 */

#include "CommandInterface.h"
#include "EMGProcessor.h"
#include "Finger.h"
#include "GraspManager.h"
#include "ReflexEngine.h"
#include "SensorFusion.h"
#include "PowerMonitor.h"

CommandInterface::CommandInterface()
    : _emgProcessor(nullptr),
      _fingers(nullptr),
      _numFingers(0),
      _graspManager(nullptr),
      _reflexEngine(nullptr),
      _sensorFusion(nullptr),
      _powerMonitor(nullptr),
      _cmdIndex(0)
{
    memset(_cmdBuffer, 0, CMD_BUF_LEN);
}

void CommandInterface::begin(
    EMGProcessor* emgProcessor,
    Finger** fingers,
    uint8_t numFingers,
    GraspManager* graspManager,
    ReflexEngine* reflexEngine,
    SensorFusion* sensorFusion,
    PowerMonitor* powerMonitor
) {
    _emgProcessor = emgProcessor;
    _fingers = fingers;
    _numFin