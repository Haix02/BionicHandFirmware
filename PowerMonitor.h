/**
 * @file PowerMonitor.h
 * @brief System voltage and current monitoring for bionic hand
 * @version 2.0
 * @author Haix02
 * @date 2025-07-10 23:14:49 UTC
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// Optional INA219 support - uncomment if using INA219 current sensor
// #define USE_INA219

#ifdef USE_INA219
#include <Adafruit_INA219.h>
#endif

// Power monitoring configuration
#define POWER_VOLTAGE_PIN A10           // Analog pin for voltage measurement
#define POWER_LOW_VOLTAGE_THRESHOLD 6.5f // Low voltage threshold (volts)
#define POWER_CRITICAL_VOLTAGE_THRESHOLD 6.0f // Critical voltage threshold
#define POWER_MAX_VOLTAGE 12.6f         // Maximum expected voltage (3S LiPo)
#define POWER_MIN_VOLTAGE 9.0f          // Minimum safe voltage (3S LiPo)

// Voltage divider configuration (adjust based on your hardware)
#define VOLTAGE_DIVIDER_R1 10000.0f     // Upper resistor (10kΩ)
#define VOLTAGE_DIVIDER_R2 3300.0f      // Lower resistor (3.3kΩ)
#define VOLTAGE_DIVIDER_RATIO ((VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2) / VOLTAGE_DIVIDER_R2)

// Averaging configuration
#define VOLTAGE_SAMPLE_COUNT 10         // Number of samples for averaging
#define POWER_UPDATE_INTERVAL 100       // Update interval in milliseconds

/**
 * @enum PowerState
 * @brief Power system state enumeration
 */
enum class PowerState {
    NORMAL,     // Normal operation voltage
    LOW,        // Low voltage warning
    CRITICAL,   // Critical voltage - implement power saving
    EXTERNAL    // External power detected
};

class PowerMonitor {
public:
    PowerMonitor();
    
    /**
     * @brief Initialize power monitoring system
     * @return True if initialization successful
     */
    bool begin();
    
    /**
     * @brief Update voltage and current readings (call periodically)
     */
    void update();
    
    /**
     * @brief Get latest measured voltage
     * @return Battery voltage in volts
     */
    float getVoltage() const;
    
    /**
     * @brief Check if system is in low power condition
     * @return True if voltage below threshold
     */
    bool isLowPower() const;
    
    /**
     * @brief Check if system is in critical power condition
     * @return True if voltage critically low
     */
    bool isCriticalPower() const;
    
    /**
     * @brief Get current power state
     * @return PowerState enumeration
     */
    PowerState getPowerState() const;
    
    /**
     * @brief Get battery percentage estimate
     * @return Estimated battery percentage (0-100)
     */
    uint8_t getBatteryPercentage() const;
    
    /**
     * @brief Get current draw (if INA219 available)
     * @return Current in milliamps
     */
    float getCurrent() const;
    
    /**
     * @brief Get power consumption (if current measurement available)
     * @return Power in milliwatts
     */
    float getPower() const;
    
    /**
     * @brief Set custom voltage thresholds
     * @param lowThreshold Low voltage threshold
     * @param criticalThreshold Critical voltage threshold
     */
    void setVoltageThresholds(float lowThreshold, float criticalThreshold);
    
    /**
     * @brief Print power status to serial
     */
    void printStatus() const;
    
    /**
     * @brief Get uptime since last power cycle
     * @return Uptime in milliseconds
     */
    uint32_t getUptime() const;
    
    /**
     * @brief Check if external power is connected
     * @return True if external power detected
     */
    bool isExternalPowerConnected() const;

private:
    // Voltage measurement
    float _voltage;
    float _voltageFiltered;
    float _voltageHistory[VOLTAGE_SAMPLE_COUNT];
    uint8_t _voltageHistoryIndex;
    bool _voltageHistoryFull;
    
    // Current measurement (if available)
    float _current;
    float _power;
    
    // Power state
    PowerState _powerState;
    bool _lowPowerFlag;
    bool _criticalPowerFlag;
    
    // Thresholds
    float _lowVoltageThreshold;
    float _criticalVoltageThreshold;
    
    // Timing
    uint32_t _lastUpdateTime;
    uint32_t _bootTime;
    
    // Hardware status
    bool _initialized;
    bool _hasINA219;
    bool _hasExternalPower;
    
    #ifdef USE_INA219
    Adafruit_INA219 _ina219;
    #endif
    
    // Private methods
    void readVoltage();
    void readCurrent();
    void updatePowerState();
    void updateVoltageHistory(float voltage);
    float calculateFilteredVoltage();
    uint8_t calculateBatteryPercentage(float voltage) const;
    bool detectExternalPower();
    
    // Utility functions
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) const;
};