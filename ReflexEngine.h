/**
 * @file ReflexEngine.h
 * @brief High-speed reflex detection and response system
 * @version 3.0
 */

#pragma once

#include <Arduino.h>
#include "config.h"
#include "Finger.h"
#include <IntervalTimer.h>

/**
 * @class ReflexEngine
 * @brief Provides neuromorphic-inspired reflex capabilities for grip control
 * 
 * The ReflexEngine runs a dedicated high-frequency loop for FSR monitoring
 * and rapid response to object slip events, simulating biological reflexes
 * without blocking the main control loop.
 */
class ReflexEngine {
public:
    ReflexEngine();
    
    /**
     * @brief Initialize the reflex engine
     * @param fingers Array of pointers to Finger objects
     */
    void begin(Finger** fingers);
    
    /**
     * @brief Enable/disable reflex functionality
     */
    void enable(bool enabled);
    
    /**
     * @brief Set the slip threshold for reflex triggering
     * @param threshold Negative rate of change threshold to trigger reflex
     */
    void setSlipThreshold(float threshold);
    
    /**
     * @brief Get reflex statistics for debugging
     */
    void getReflexStats(uint32_t* reflexCount, float* avgResponseTime);
    
    /**
     * @brief Main update function (called from main loop)
     */
    void update();

private:
    // Pointers to finger objects
    Finger** _fingers;
    
    // RefleX control
    IntervalTimer _reflexTimer;
    volatile bool _enabled;
    float _slipThreshold;
    
    // Reflex statistics
    volatile uint32_t _reflexCount;
    volatile uint32_t _totalResponseTime;
    volatile uint32_t _lastSlipDetectTime;
    volatile float _avgResponseTime;
    
    // FSR history for slip detection
    volatile float _fsrHistory[NUM_FINGERS][FSR_DERIV_WINDOW];
    volatile uint8_t _fsrHistoryIdx[NUM_FINGERS];
    
    // Method that gets called by IntervalTimer
    static void reflexISR();
    static ReflexEngine* _instance;  // Singleton for ISR access
    
    // Internal reflex processing methods
    void processFSRReadings();
    bool detectSlip(uint8_t fingerIdx, float* slipRate);
    void triggerReflex(uint8_t fingerIdx, float slipRate);
};