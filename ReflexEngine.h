/**
 * @file ReflexEngine.h
 * @brief High-speed slip detection and reflexive grip
 * @version 2.0
 */

#pragma once

#include <Arduino.h>
#include "Finger.h"
#include "config.h"

// FSR read function type definition
typedef float (*FSRReadFunction)(uint8_t index);

// Default parameters
#define MAX_FINGERS 8
#define FSR_HISTORY_SIZE 5
#define REFLEX_SLIP_THRESHOLD -0.2f  // Negative threshold for slip detection
#define REFLEX_MIN_FORCE 0.05f       // Minimum force for slip detection
#define REFLEX_COOLDOWN_MS 300       // Minimum time between reflexes

//#define DEBUG_REFLEX               // Uncomment for debug output

class ReflexEngine {
public:
    ReflexEngine();
    
    /**
     * @brief Initialize with finger array
     * @param fingers Array of finger pointers
     * @param numFingers Number of fingers in array
     */
    void begin(Finger** fingers, uint8_t numFingers);
    
    /**
     * @brief Set function to read FSR values
     * @param func Function pointer to read FSR
     */
    void setFSRReadFunction(FSRReadFunction func);
    
    /**
     * @brief Enable or disable the reflex system
     * @param enabled True to enable, false to disable
     */
    void enable(bool enabled);
    
    /**
     * @brief Set slip detection threshold
     * @param threshold Negative threshold for slip detection
     */
    void setSlipThreshold(float threshold);
    
    /**
     * @brief Set low power mode
     * @param enabled True to enable low power mode
     */
    void setLowPowerMode(bool enabled);
    
    /**
     * @brief Update method called from timer interrupt
     */
    void update();
    
    /**
     * @brief Manually trigger reflex on a finger
     * @param fingerIdx Finger index
     */
    void triggerManualReflex(uint8_t fingerIdx);
    
    /**
     * @brief Get reflex statistics
     * @param count Pointer to store reflex count
     * @param avgTime Pointer to store average response time (ms)
     */
    void getReflexStats(uint32_t* count, float* avgTime) const;
    
    // Static instance for ISR
    static ReflexEngine* _instance;
    
private:
    // Fingers
    Finger** _fingers;
    uint8_t _numFingers;
    
    // Function to read FSR values
    FSRReadFunction _fsrReadFunc;
    
    // Configuration
    bool _enabled;
    float _slipThreshold;
    bool _lowPowerMode;
    
    // FSR history for derivative calculation
    float _fsrHistory[MAX_FINGERS][FSR_HISTORY_SIZE];
    uint8_t _fsrHistoryIndex[MAX_FINGERS];
    
    // Reflex state
    uint32_t _lastReflexTime[MAX_FINGERS];
    bool _slipDetected[MAX_FINGERS];
    uint32_t _slipStartTime;
    
    // Statistics
    uint32_t _reflexCount;
    uint32_t _totalResponseTime;
    float _avgResponseTime;
    
    // Private methods
    bool detectSlip(uint8_t fingerIdx);
    void triggerReflex(uint8_t fingerIdx);
};