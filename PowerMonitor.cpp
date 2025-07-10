/**
 * @file PowerMonitor.cpp
 * @brief Implementation of battery voltage and current monitoring
 * @version 1.0
 */

#include "PowerMonitor.h"

PowerMonitor::PowerMonitor(uint8_t voltagePin)
    : _voltagePin(voltagePin),
      _batteryVoltage(0.0f),
      _currentDraw(0.0f),
      _powerConsumption(0.0f),
      _powerState(PowerState::NORMAL),
      _batteryPercentage(100),
      _criticalVoltage(6.4f),  // For 2S LiPo (3.2V per cell)
      _lowVoltage(6.8f),       // For 2S LiPo (3.4V per cell)
      _fullVoltage(8.4f),      // For 2S LiPo (4.2V per cell)
      _voltageDividerRatio(2.0f),  // Default ratio
      _lastUpdateTime(0)
{
    #ifdef USE_INA219
    _hasINA219 = false;
    #endif
}

bool PowerMonitor::begin() {
    // Configure analog pin for voltage monitoring
    if (_voltagePin > 0) {
        pinMode(_voltagePin, INPUT);
    }
    
    #ifdef USE_INA219
    // Initialize INA219 current sensor if available
    _hasINA219 = _ina219.begin();
    
    if (_hasINA219) {
        // Configure INA219
        _ina219.setCalibration_16V_400mA();
    }
    #endif
    
    // Do initial update
    update();
    
    return true;
}

void PowerMonitor::update() {
    // Limit update rate to once per 500ms
    uint32_t currentTime = millis();
    if (currentTime - _lastUpdateTime < 500) {
        return;
    }
    _lastUpdateTime = currentTime;
    
    // Read battery voltage
    if (_voltagePin > 0) {
        // Read from analog pin and apply voltage divider conversion
        float raw = analogRead(_voltagePin);
        _batteryVoltage = (raw / 1023.0f) * 3.3f * _voltageDividerRatio;
    }
    
    #ifdef USE_INA219
    // Read current and power if INA219 is available
    if (_hasINA219) {
        _currentDraw = _ina219.getCurrent_mA();
        _powerConsumption = _ina219.getPower_mW();
        
        // If we don't have a voltage pin, use INA219's voltage
        if (_voltagePin == 0) {
            _batteryVoltage = _ina219.getBusVoltage_V() + (_ina219.getShuntVoltage_mV() / 1000.0f);
        }
    }
    #endif
    
    // Determine power state based on voltage
    if (_batteryVoltage <= _criticalVoltage) {
        _powerState = PowerState::CRITICAL;
    } else if (_batteryVoltage <= _lowVoltage) {
        _powerState = PowerState::LOW;
    } else if (_batteryVoltage >= _fullVoltage) {
        _powerState = PowerState::FULL;
    } else {
        _powerState = PowerState::NORMAL;
    }
    
    // Calculate battery percentage
    _batteryPercentage = calculateBatteryPercentage();
}

float PowerMonitor::getBatteryVoltage() const {
    return _batteryVoltage;
}

float PowerMonitor::getCurrentDraw() const {
    return _currentDraw;
}

float PowerMonitor::getPowerConsumption() const {
    return _powerConsumption;
}

PowerState PowerMonitor::getPowerState() const {
    return _powerState;
}

uint8_t PowerMonitor::getBatteryPercentage() const {
    return _batteryPercentage;
}

void PowerMonitor::setThresholds(float critical, float low, float full) {
    _criticalVoltage = critical;
    _lowVoltage = low;
    _fullVoltage = full;
}

void PowerMonitor::printStatus(Stream& stream) const {
    stream.print(F("Battery: "));
    stream.print(_batteryVoltage, 2);
    stream.print(F("V ("));
    stream.print(_batteryPercentage);
    stream.print(F("%) | Current: "));
    stream.print(_currentDraw, 1);
    stream.print(F("mA | Power: "));
    stream.print(_powerConsumption, 1);
    stream.print(F("mW | State: "));
    
    switch (_powerState) {
        case PowerState::CRITICAL:
            stream.print(F("CRITICAL"));
            break;
        case PowerState::LOW:
            stream.print(F("LOW"));
            break;
        case PowerState::NORMAL:
            stream.print(F("NORMAL"));
            break;
        case PowerState::FULL:
            stream.print(F("FULL"));
            break;
    }
    stream.println();
}

uint8_t PowerMonitor::calculateBatteryPercentage() const {
    // Simple linear mapping from voltage to percentage
    if (_batteryVoltage <= _criticalVoltage) return 0;
    if (_batteryVoltage >= _fullVoltage) return 100;
    
    float range = _fullVoltage - _criticalVoltage;
    float normalizedVoltage = _batteryVoltage - _criticalVoltage;
    
    return (uint8_t)(normalizedVoltage * 100.0f / range);
}