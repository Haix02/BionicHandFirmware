/**
 * @file ReflexEngine.h
 * @brief High-speed reflex detection and response system
 * @version 1.0
 */

#pragma once

#include <Arduino.h>
#include <IntervalTimer.h>
#include "config.h"
#include "Finger.h"

/**
 * @class ReflexEngine
 * @brief Implements rapid slip detection and reflex response
 * 
 * The ReflexEngine monitors FSR sensors at high frequency (1 kHz) to detect
 * object slippage via force derivative (dF/dt), triggering immediate grip
 * force adjustments when necessary.
 */
class ReflexEngine {
public:
    ReflexEngine();
    
    /**
     * @brief Initialize the reflex system
     * @param fingers Array of pointers to finger objects
     * @param numFingers Number of fingers in the array
     */
    void begin(Finger** fingers, uint8_t numFingers);
    
    /**
     * @brief Enable or disable the reflex system
     * @param enabled True to enable, false to disable
     */
    void enable(bool enabled);
    
    /**
     * @brief Configure slip detection sensitivity
     * @param threshold Negative threshold for slip detection (default -0.2)
     */
    void setSlipThreshold(float threshold);
    
    /**
     * @brief Configure reflexive force response
     * @param boost Force boost applied during reflex (0.0-1.0)
     */
    void setForceBoost(float boost);
    
    /**
     * @brief Get reflex statistics
     * @param count Pointer to store total reflex count
     * @param avgResponse Pointer to store average response time in ms
     */
    void getReflexStats(uint32_t* count, float* avgResponse);
    
    /**
     * @brief Update method - call from main loop for housekeeping
     */
    void update();
    
    /**
     * @brief Manually trigger reflexive grip on a specific finger
     * @param fingerIdx Index of finger to trigger reflex on
     */
    void triggerManualReflex(uint8_t fingerIdx);

private:
    Finger** _fingers;
    uint8_t _numFingers;
    IntervalTimer _reflexTimer;
    volatile bool _enabled;
    
    // Slip detection parameters
    float _slipThreshold;
    float _forceBoost;
    
    // FSR history for slip detection
    static constexpr uint8_t FSR_HISTORY_LEN = 5;
    volatile float _fsrHistory[MAX_FINGERS][FSR_HISTORY_LEN];
    volatile uint8_t _fsrHistoryIdx[MAX_FINGERS];
    
    // Reflex statistics
    volatile uint32_t _reflexCount;
    volatile uint32_t _totalResponseTime;
    volatile float _avgResponseTime;
    volatile uint32_t _lastSlipDetectTime;
    
    // Static functions for IntervalTimer
    static ReflexEngine* _instance;
    static void _reflexISRWrapper();
    
    // Internal methods
    void reflexISR();
    bool detectSlip(uint8_t fingerIdx, float* slipRate = nullptr);
    void triggerReflex(uint8_t fingerIdx);
};