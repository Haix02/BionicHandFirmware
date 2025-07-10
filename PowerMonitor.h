/**
 * @file PowerMonitor.h
 * @brief Battery voltage and current monitoring
 * @version 1.0
 */

#pragma once

#include <Arduino.h>
#include "config.h"

// Include INA219 library if available
#ifdef USE_INA219
#include <Adafruit_INA219.h>
#endif

/**
 * @enum PowerState
 * @brief Represents battery charge state
 */
enum class PowerState {
    CRITICAL,   // Critically low, power-saving needed
    LOW,        // Low but operational
    NORMAL,     // Normal operation
    FULL        // Fully charged
};

/**
 * @class PowerMonitor
 * @brief Monitors battery voltage and current consumption
 * 
 * Provides power system monitoring with safety features for
 * low battery conditions.
 */
class PowerMonitor {
public:
    PowerMonitor(uint8_t voltagePin = 0);
    
    /**
     * @brief Initialize the power monitor
     * @return true if initialized successfully
     */
    bool begin();
    
    /**
     * @brief Update power readings
     * Call this periodically from the main loop
     */
    void update();
    
    /**
     * @brief Get battery voltage
     * @return Battery voltage in volts
     */
    float getBatteryVoltage() const;
    
    /**
     * @brief Get current draw
     * @return Current in mA
     */
    float getCurrentDraw() const;
    
    /**
     * @brief Get power consumption
     * @return Power in mW
     */
    float getPowerConsumption() const;
    
    /**
     * @brief Get battery state
     * @return PowerState enum value
     */
    PowerState getPowerState() const;
    
    /**
     * @brief Get battery percentage
     * @return Estimated battery percentage (0-100)
     */
    uint8_t getBatteryPercentage() const;
    
    /**
     * @brief Set voltage thresholds
     * @param critical Voltage for critical state
     * @param low Voltage for low state
     * @param full Voltage for full state
     */
    void setThresholds(float critical, float low, float full);
    
    /**
     * @brief Print power system status
     * @param stream Stream to print to (e.g., Serial)
     */
    void printStatus(Stream& stream) const;
    
private:
    uint8_t _voltagePin;
    float _batteryVoltage;
    float _currentDraw;
    float _powerConsumption;
    PowerState _powerState;
    uint8_t _batteryPercentage;
    
    // Thresholds
    float _criticalVoltage;
    float _lowVoltage;
    float _fullVoltage;
    
    // INA219 current sensor (if available)
    #ifdef USE_INA219
    Adafruit_INA219 _ina219;
    bool _hasINA219;
    #endif
    
    // Voltage divider parameters
    float _voltageDividerRatio;
    
    // Last update time
    uint32_t _lastUpdateTime;
    
    // Calculate battery percentage based on voltage
    uint8_t calculateBatteryPercentage() const;
};