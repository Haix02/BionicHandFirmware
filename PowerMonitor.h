/**
 * @file PowerMonitor.h
 * @brief Battery voltage and current monitoring
 */

#pragma once

#include <Arduino.h>
#include "config.h"

// Optional INA219 support
#ifdef USE_INA219
#include <Adafruit_INA219.h>
#endif

/**
 * @enum PowerState
 * @brief Power system state categories
 */
enum class PowerState {
    NORMAL,     // Normal operating voltage
    LOW,        // Low voltage, but still operational
    CRITICAL,   // Critical voltage, implement power saving
    EXTERNAL    // Running on external power
};

class PowerMonitor {
public:
    /**
     * @brief Constructor
     * @param voltagePin Analog pin for voltage monitoring
     */
    PowerMonitor(uint8_t voltagePin = 0);
    
    /**
     * @brief Initialize power monitoring
     */
    bool begin();
    
    /**
     * @brief Update power measurements
     */
    void update();
    
    /**
     * @brief Get current battery voltage
     * @return Voltage in volts
     */
    float getBatteryVoltage() const;
    
    /**
     * @brief Get current current draw
     * @return Current in milliamps
     */
    float getCurrentDraw() const;
    
    /**
     * @brief Get power consumption
     * @return Power in milliwatts
     */
    float getPowerConsumption() const;
    
    /**
     * @brief Get remaining battery percentage
     * @return Percentage from 0-100
     */
    uint8_t getBatteryPercentage() const;
    
    /**
     * @brief Get power system state
     * @return PowerState enumeration value
     */
    PowerState getPowerState() const;
    
    /**
     * @brief Configure voltage thresholds
     * @param critical Critical voltage threshold
     * @param low Low voltage threshold
     */
    void setThresholds(float critical, float low);
    
    /**
     * @brief Print power status to serial
     */
    void printStatus() const;

private:
    uint8_t _voltagePin;
    float _batteryVoltage;
    float _currentDraw;
    float _powerConsumption;
    uint8_t _batteryPercentage;
    PowerState _powerState;
    
    // Voltage thresholds
    float _criticalVoltage;  // Enter power saving at this voltage
    float _lowVoltage;       // Warning threshold
    
    // Voltage divider ratio (depends on your hardware)
    float _voltageDividerRatio;
    
    // For averaging
    float _voltageHistory[10];
    uint8_t _voltageHistoryIndex;
    
    #ifdef USE_INA219
    Adafruit_INA219 _ina219;
    bool _hasINA219;
    #endif
    
    uint32_t _lastUpdateTime;
    
    // Calculate battery percentage based on voltage
    uint8_t calculatePercentage(float voltage) const;
};