/**
 * @file PowerMonitor.cpp
 * @brief System voltage and current monitoring implementation
 * @version 2.0
 * @author Haix02
 * @date 2025-07-10 23:14:49 UTC
 */

#include "PowerMonitor.h"

PowerMonitor::PowerMonitor()
    : _voltage(0.0f),
      _voltageFiltered(0.0f),
      _voltageHistoryIndex(0),
      _voltageHistoryFull(false),
      _current(0.0f),
      _power(0.0f),
      _powerState(PowerState::NORMAL),
      _lowPowerFlag(false),
      _criticalPowerFlag(false),
      _lowVoltageThreshold(POWER_LOW_VOLTAGE_THRESHOLD),
      _criticalVoltageThreshold(POWER_CRITICAL_VOLTAGE_THRESHOLD),
      _lastUpdateTime(0),
      _bootTime(0),
      _initialized(false),
      _hasINA219(false),
      _hasExternalPower(false)
{
    // Initialize voltage history array
    for (uint8_t i = 0; i < VOLTAGE_SAMPLE_COUNT; i++) {
        _voltageHistory[i] = 0.0f;
    }
}

bool PowerMonitor::begin() {
    Serial.println(F("Initializing power monitor..."));
    
    // Record boot time
    _bootTime = millis();
    
    // Configure analog pin for voltage measurement
    pinMode(POWER_VOLTAGE_PIN, INPUT);
    
    // Set ADC resolution for better precision
    analogReadResolution(12); // 12-bit ADC (0-4095)
    analogReadAveraging(8);   // Average 8 samples for noise reduction
    
    #ifdef USE_INA219
    // Initialize INA219 current sensor if available
    Wire.begin();
    _hasINA219 = _ina219.begin();
    
    if (_hasINA219) {
        // Configure INA219 for our expected voltage/current range
        // Assuming 16V, 400mA max for better resolution
        _ina219.setCalibration_16V_400mA();
        Serial.println(F("  INA219 current sensor detected"));
    } else {
        Serial.println(F("  INA219 current sensor not found"));
    }
    #endif
    
    // Initial voltage reading
    readVoltage();
    
    // Initialize filtered voltage
    _voltageFiltered = _voltage;
    
    // Fill voltage history with initial reading
    for (uint8_t i = 0; i < VOLTAGE_SAMPLE_COUNT; i++) {
        _voltageHistory[i] = _voltage;
    }
    
    // Initial power state assessment
    updatePowerState();
    
    _initialized = true;
    
    Serial.print(F("Power monitor initialized - Initial voltage: "));
    Serial.print(_voltage, 2);
    Serial.println(F("V"));
    
    return true;
}

void PowerMonitor::update() {
    if (!_initialized) return;
    
    uint32_t currentTime = millis();
    
    // Limit update rate to preserve system performance
    if (currentTime - _lastUpdateTime < POWER_UPDATE_INTERVAL) {
        return;
    }
    
    _lastUpdateTime = currentTime;
    
    // Read voltage from analog pin
    readVoltage();
    
    // Read current if INA219 is available
    readCurrent();
    
    // Update voltage history and filtering
    updateVoltageHistory(_voltage);
    _voltageFiltered = calculateFilteredVoltage();
    
    // Detect external power
    _hasExternalPower = detectExternalPower();
    
    // Update power state based on readings
    updatePowerState();
}

void PowerMonitor::readVoltage() {
    // Read analog value (12-bit ADC: 0-4095)
    uint16_t adcValue = analogRead(POWER_VOLTAGE_PIN);
    
    // Convert to voltage considering voltage divider
    // ADC voltage = (adcValue / 4095.0) * 3.3V
    // Actual voltage = ADC voltage * voltage divider ratio
    float adcVoltage = (adcValue / 4095.0f) * 3.3f;
    _voltage = adcVoltage * VOLTAGE_DIVIDER_RATIO;
    
    // Sanity check - clamp to reasonable range
    _voltage = constrain(_voltage, 0.0f, 20.0f);
}

void PowerMonitor::readCurrent() {
    #ifdef USE_INA219
    if (_hasINA219) {
        // Read current and power from INA219
        _current = _ina219.getCurrent_mA();
        _power = _ina219.getPower_mW();
        
        // Use INA219 voltage reading if available (more accurate)
        float ina219Voltage = _ina219.getBusVoltage_V() + (_ina219.getShuntVoltage_mV() / 1000.0f);
        
        // Blend INA219 voltage with analog reading for better accuracy
        if (ina219Voltage > 1.0f && ina219Voltage < 20.0f) {
            _voltage = (_voltage + ina219Voltage) / 2.0f;
        }
    } else {
        // Estimate current based on voltage drop (rough approximation)
        // This is not accurate but provides some indication
        _current = 0.0f;
        _power = 0.0f;
    }
    #else
    // No current measurement available
    _current = 0.0f;
    _power = 0.0f;
    #endif
}

void PowerMonitor::updateVoltageHistory(float voltage) {
    // Add new voltage reading to circular buffer
    _voltageHistory[_voltageHistoryIndex] = voltage;
    _voltageHistoryIndex = (_voltageHistoryIndex + 1) % VOLTAGE_SAMPLE_COUNT;
    
    // Mark history as full after first complete cycle
    if (_voltageHistoryIndex == 0) {
        _voltageHistoryFull = true;
    }
}

float PowerMonitor::calculateFilteredVoltage() {
    // Calculate average of voltage history for filtering
    float sum = 0.0f;
    uint8_t count = _voltageHistoryFull ? VOLTAGE_SAMPLE_COUNT : _voltageHistoryIndex;
    
    for (uint8_t i = 0; i < count; i++) {
        sum += _voltageHistory[i];
    }
    
    return (count > 0) ? (sum / count) : _voltage;
}

void PowerMonitor::updatePowerState() {
    // Use filtered voltage for state decisions to avoid false alarms
    float voltage = _voltageFiltered;
    
    // Detect external power (voltage significantly above battery max)
    if (voltage > POWER_MAX_VOLTAGE + 1.0f) {
        _powerState = PowerState::EXTERNAL;
        _lowPowerFlag = false;
        _criticalPowerFlag = false;
    }
    // Critical power state
    else if (voltage <= _criticalVoltageThreshold) {
        _powerState = PowerState::CRITICAL;
        _lowPowerFlag = true;
        _criticalPowerFlag = true;
    }
    // Low power state
    else if (voltage <= _lowVoltageThreshold) {
        _powerState = PowerState::LOW;
        _lowPowerFlag = true;
        _criticalPowerFlag = false;
    }
    // Normal operation
    else {
        _powerState = PowerState::NORMAL;
        _lowPowerFlag = false;
        _criticalPowerFlag = false;
    }
}

bool PowerMonitor::detectExternalPower() {
    // External power typically provides higher voltage than battery
    return (_voltageFiltered > POWER_MAX_VOLTAGE + 0.5f);
}

float PowerMonitor::getVoltage() const {
    return _voltageFiltered;
}

bool PowerMonitor::isLowPower() const {
    return _lowPowerFlag;
}

bool PowerMonitor::isCriticalPower() const {
    return _criticalPowerFlag;
}

PowerState PowerMonitor::getPowerState() const {
    return _powerState;
}

uint8_t PowerMonitor::getBatteryPercentage() const {
    return calculateBatteryPercentage(_voltageFiltered);
}

float PowerMonitor::getCurrent() const {
    return _current;
}

float PowerMonitor::getPower() const {
    return _power;
}

void PowerMonitor::setVoltageThresholds(float lowThreshold, float criticalThreshold) {
    _lowVoltageThreshold = constrain(lowThreshold, 5.0f, 15.0f);
    _criticalVoltageThreshold = constrain(criticalThreshold, 4.0f, _lowVoltageThreshold - 0.1f);
    
    Serial.print(F("Voltage thresholds updated - Low: "));
    Serial.print(_lowVoltageThreshold, 1);
    Serial.print(F("V, Critical: "));
    Serial.print(_criticalVoltageThreshold, 1);
    Serial.println(F("V"));
}

void PowerMonitor::printStatus() const {
    Serial.println(F("\n===== POWER STATUS ====="));
    
    // Voltage information
    Serial.print(F("Voltage: "));
    Serial.print(_voltageFiltered, 2);
    Serial.print(F("V (Raw: "));
    Serial.print(_voltage, 2);
    Serial.println(F("V)"));
    
    // Battery percentage
    Serial.print(F("Battery: "));
    Serial.print(getBatteryPercentage());
    Serial.println(F("%"));
    
    // Current and power (if available)
    if (_hasINA219 || _current > 0) {
        Serial.print(F("Current: "));
        Serial.print(_current, 1);
        Serial.println(F(" mA"));
        
        Serial.print(F("Power: "));
        Serial.print(_power, 1);
        Serial.println(F(" mW"));
    }
    
    // Power state
    Serial.print(F("State: "));
    switch (_powerState) {
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
    
    // Thresholds
    Serial.print(F("Thresholds - Low: "));
    Serial.print(_lowVoltageThreshold, 1);
    Serial.print(F("V, Critical: "));
    Serial.print(_criticalVoltageThreshold, 1);
    Serial.println(F("V"));
    
    // Uptime
    Serial.print(F("Uptime: "));
    Serial.print(getUptime() / 1000);
    Serial.println(F(" seconds"));
    
    Serial.println(F("========================\n"));
}

uint32_t PowerMonitor::getUptime() const {
    return millis() - _bootTime;
}

bool PowerMonitor::isExternalPowerConnected() const {
    return _hasExternalPower;
}

uint8_t PowerMonitor::calculateBatteryPercentage(float voltage) const {
    // LiPo battery discharge curve approximation (3S pack: 9.0V - 12.6V)
    if (voltage >= POWER_MAX_VOLTAGE) return 100;
    if (voltage <= POWER_MIN_VOLTAGE) return 0;
    
    // Non-linear battery discharge curve approximation
    // LiPo batteries have a relatively flat discharge curve until ~3.6V per cell
    
    float cellVoltage = voltage / 3.0f; // 3S pack
    uint8_t percentage;
    
    if (cellVoltage >= 4.1f) {
        percentage = 100;
    } else if (cellVoltage >= 3.9f) {
        percentage = mapFloat(cellVoltage, 3.9f, 4.1f, 80, 100);
    } else if (cellVoltage >= 3.7f) {
        percentage = mapFloat(cellVoltage, 3.7f, 3.9f, 40, 80);
    } else if (cellVoltage >= 3.5f) {
        percentage = mapFloat(cellVoltage, 3.5f, 3.7f, 15, 40);
    } else if (cellVoltage >= 3.3f) {
        percentage = mapFloat(cellVoltage, 3.3f, 3.5f, 5, 15);
    } else {
        percentage = mapFloat(cellVoltage, 3.0f, 3.3f, 0, 5);
    }
    
    return constrain(percentage, 0, 100);
}

float PowerMonitor::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) const {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}