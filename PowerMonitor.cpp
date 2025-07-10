/**
 * @file PowerMonitor.cpp
 * @brief Battery monitoring implementation
 */

#include "PowerMonitor.h"

PowerMonitor::PowerMonitor(uint8_t voltagePin) 
    : _voltagePin(voltagePin),
      _batteryVoltage(0.0f),
      _currentDraw(0.0f),
      _powerConsumption(0.0f),
      _batteryPercentage(100),
      _powerState(PowerState::NORMAL),
      _criticalVoltage(6.5f),  // For 2S LiPo (3.25V per cell)
      _lowVoltage(7.0f),       // For 2S LiPo (3.5V per cell)
      _voltageDividerRatio(2.0f), // Assuming voltage divider halves voltage
      _voltageHistoryIndex(0),
      _lastUpdateTime(0)
{
    // Initialize voltage history
    for (uint8_t i = 0; i < 10; i++) {
        _voltageHistory[i] = 0.0f;
    }
    
    #ifdef USE_INA219
    _hasINA219 = false;
    #endif
}

bool PowerMonitor::begin() {
    if (_voltagePin > 0) {
        pinMode(_voltagePin, INPUT);
    }
    
    #ifdef USE_INA219
    // Try to initialize INA219 current/voltage sensor
    _hasINA219 = _ina219.begin();
    if (_hasINA219) {
        // Configure for 16V, 400mA range for higher precision
        _ina219.setCalibration_16V_400mA();
    }
    #endif
    
    // Initial measurement
    update();
    
    return true;
}

void PowerMonitor::update() {
    uint32_t currentTime = millis();
    
    // Limit update rate to prevent excessive ADC usage
    if (currentTime - _lastUpdateTime < 500) {
        return;
    }
    _lastUpdateTime = currentTime;
    
    #ifdef USE_INA219
    if (_hasINA219) {
        // Use INA219 for accurate measurements if available
        _batteryVoltage = _ina219.getBusVoltage_V() + (_ina219.getShuntVoltage_mV() / 1000.0f);
        _currentDraw = _ina219.getCurrent_mA();
        _powerConsumption = _ina219.getPower_mW();
    } else 
    #endif
    if (_voltagePin > 0) {
        // Use analog pin for voltage measurement
        int rawValue = analogRead(_voltagePin);
        
        // Convert to voltage, considering voltage divider
        float measuredVoltage = (rawValue / 1023.0f) * 3.3f * _voltageDividerRatio;
        
        // Update rolling average
        _voltageHistory[_voltageHistoryIndex] = measuredVoltage;
        _voltageHistoryIndex = (_voltageHistoryIndex + 1) % 10;
        
        // Calculate average voltage to smooth readings
        float sumVoltage = 0.0f;
        for (uint8_t i = 0; i < 10; i++) {
            sumVoltage += _voltageHistory[i];
        }
        _batteryVoltage = sumVoltage / 10.0f;
        
        // Estimate current based on voltage drop (if no INA219)
        // This is a rough estimate and depends on your specific battery
        _currentDraw = 0.0f; // Not implemented for analog reading
        _powerConsumption = 0.0f;
    }
    
    // Calculate battery percentage
    _batteryPercentage = calculatePercentage(_batteryVoltage);
    
    // Determine power state
    if (_batteryVoltage <= _criticalVoltage) {
        _powerState = PowerState::CRITICAL;
    } else if (_batteryVoltage <= _lowVoltage) {
        _powerState = PowerState::LOW;
    } else {
        _powerState = PowerState::NORMAL;
    }
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

uint8_t PowerMonitor::getBatteryPercentage() const {
    return _batteryPercentage;
}

PowerState PowerMonitor::getPowerState() const {
    return _powerState;
}

void PowerMonitor::setThresholds(float critical, float low) {
    _criticalVoltage = critical;
    _lowVoltage = low;
}

void PowerMonitor::printStatus() const {
    Serial.print(F("Battery: "));
    Serial.print(_batteryVoltage, 2);
    Serial.print(F("V ("));
    Serial.print(_batteryPercentage);
    Serial.print(F("%) | Current: "));
    Serial.print(_currentDraw, 1);
    Serial.print(F("mA | Power: "));
    Serial.print(_powerConsumption, 1);
    Serial.print(F("mW | State: "));
    
    switch (_powerState) {
        case PowerState::NORMAL:
            Serial.print(F("NORMAL"));
            break;
        case PowerState::LOW:
            Serial.print(F("LOW"));
            break;
        case PowerState::CRITICAL:
            Serial.print(F("CRITICAL"));
            break;
        case PowerState::EXTERNAL:
            Serial.print(F("EXTERNAL"));
            break;
    }
    
    Serial.println();
}

uint8_t PowerMonitor::calculatePercentage(float voltage) const {
    // LiPo battery discharge curve is non-linear
    // This is a simplified calculation
    const float fullVoltage = 8.4f;  // 2S LiPo fully charged
    const float emptyVoltage = 6.0f; // 2S LiPo empty
    
    if (voltage >= fullVoltage) return 100;
    if (voltage <= emptyVoltage) return 0;
    
    // Map voltage to percentage
    return map(voltage * 100, emptyVoltage * 100, fullVoltage * 100, 0, 100);
}