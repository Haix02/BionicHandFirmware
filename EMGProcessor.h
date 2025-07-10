/**
 * @file EMGProcessor.h
 * @brief Enhanced EMG signal processing with advanced feature extraction
 * @version 2.0
 */

#pragma once

#include <Arduino.h>
#include "config.h"
#include <arm_math.h> // CMSIS-DSP

// Number of features extracted per channel
#define FEATURES_PER_CHANNEL 4 // RMS, ZC, WL, SSC

class EMGProcessor {
public:
    // Extended features structure for EMG analysis
    struct ExtendedFeatures {
        float rms;         // Root Mean Square
        float zc;          // Zero Crossing count
        float wl;          // Waveform Length
        float ssc;         // Slope Sign Changes
        bool isActive;     // Activity detection flag
    };
    
    EMGProcessor();
    
    // Original API (preserved for compatibility)
    void begin();
    void sampleISR();      // Called from timer interrupt
    void update();         // Called from main loop
    
    float getFeature(uint8_t ch) const;
    void getFeatureVector(float* out) const;
    
    void setChannelOffset(uint8_t ch, float offset);
    void calibrateRest();
    
    // Enhanced API
    const float* getAllFeatures() const;
    void getExtendedFeatures(uint8_t ch, ExtendedFeatures* features) const;
    bool isChannelActive(uint8_t ch) const;
    float getRMS(uint8_t ch) const;

private:
    // Channel state structure
    struct ChannelState {
        float raw;             // Raw ADC value
        float filtered;        // Bandpass filtered signal
        float rectified;       // Full-wave rectified signal
        float rms;             // Root Mean Square value
        float offset;          // DC offset/baseline
        float norm;            // Normalized output (0.0-1.0)
        
        // Circular buffer for sliding window
        float buffer[EMG_BUFFER_SIZE];
        float windowSum;       // Sum of values in buffer
        uint16_t bufferIndex;  // Current position in buffer
        
        // Raw signal history for feature extraction
        float history[EMG_BUFFER_SIZE];
        
        // Advanced features
        float zc;              // Zero Crossing count
        float wl;              // Waveform Length
        float ssc;             // Slope Sign Changes
        bool active;           // Activity detection flag
    };
    
    // Channel states
    ChannelState _channels[NUM_EMG_CHANNELS];
    
    // Feature vector for external access
    // Format: [ch0_rms, ch0_zc, ch0_wl, ch0_ssc, ch1_rms, ch1_zc, ...]
    float _featureVector[NUM_EMG_CHANNELS * FEATURES_PER_CHANNEL];
    
    // CMSIS-DSP filter instances
    arm_biquad_casd_df1_inst_f32 _filters[NUM_EMG_CHANNELS];
    float _filterCoeffs[5 * NUM_EMG_CHANNELS]; // 5 coefficients per filter
    float _filterStates[4 * NUM_EMG_CHANNELS]; // 4 state variables per filter
    
    // Private methods
    void configureFilters();
    void processSample(uint8_t ch, float sample);
    void updateFeatures(uint8_t ch);
    void normalizeSignal(uint8_t ch);
};