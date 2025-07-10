/**
 * @file EMGProcessor.h
 * @brief Enhanced EMG signal processing with real-time DSP features
 * @version 2.0
 * @author Haix02
 * @date 2025-07-10
 */

#pragma once

#include <Arduino.h>
#include "config.h"
#include <arm_math.h>

// EMG processing parameters
#define EMG_SAMPLE_HZ 2000              // Sampling frequency (2 kHz)
#define EMG_BUFFER_SIZE 200             // 100ms window at 2kHz
#define EMG_FEATURES_PER_CHANNEL 4      // RMS, ZC, SSC, WL
#define EMG_FEATURE_WINDOW 200          // Samples for feature calculation
#define FEATURE_UPDATE_INTERVAL 40      // Update features every 40 samples (50Hz)

// Signal processing thresholds
#define EMG_ZC_THRESHOLD 15.0f          // Zero crossing detection threshold
#define EMG_SSC_THRESHOLD 10.0f         // Slope sign change threshold
#define EMG_ACTIVITY_THRESHOLD 0.05f    // Activity detection threshold
#define EMG_NORM_SCALE 2048.0f          // Normalization scale factor

// Feature normalization factors
#define EMG_ZC_NORM_FACTOR 50.0f        // Normalize ZC to 0-1 range
#define EMG_SSC_NORM_FACTOR 30.0f       // Normalize SSC to 0-1 range
#define EMG_WL_NORM_FACTOR 5000.0f      // Normalize WL to 0-1 range

// Extended feature structure
struct EMGFeatures {
    float rms;          // Root Mean Square
    float zc;           // Zero Crossings
    float ssc;          // Slope Sign Changes
    float wl;           // Waveform Length
    bool isActive;      // Activity detection flag
};

class EMGProcessor {
public:
    EMGProcessor();
    
    /**
     * @brief Initialize hardware and configure filters
     */
    void begin();
    
    /**
     * @brief High-speed sampling function (called from timer ISR at 2kHz)
     */
    void sampleISR();
    
    /**
     * @brief Update features and perform non-time-critical processing
     */
    void update();
    
    /**
     * @brief Get normalized amplitude for a channel (legacy API)
     * @param ch Channel index (0-7)
     * @return Normalized amplitude (0.0-1.0)
     */
    float getFeature(uint8_t ch) const;
    
    /**
     * @brief Fill array with normalized amplitudes for all channels
     * @param out Output array (must be NUM_EMG_CHANNELS in size)
     */
    void getFeatureVector(float* out) const;
    
    /**
     * @brief Set DC offset for a channel
     * @param ch Channel index
     * @param offset Offset value
     */
    void setChannelOffset(uint8_t ch, float offset);
    
    /**
     * @brief Get DC offset for a channel
     * @param ch Channel index
     * @return Current offset value
     */
    float getChannelOffset(uint8_t ch) const;
    
    /**
     * @brief Calibrate resting levels for all channels
     */
    void calibrateRest();
    
    /**
     * @brief Check if channel shows muscle activity
     * @param ch Channel index
     * @return True if activity detected
     */
    bool isChannelActive(uint8_t ch) const;
    
    /**
     * @brief Get individual feature values
     */
    float getRMS(uint8_t ch) const;
    float getZC(uint8_t ch) const;
    float getSSC(uint8_t ch) const;
    float getWL(uint8_t ch) const;
    
    /**
     * @brief Get extended features for a channel
     * @param ch Channel index
     * @param features Pointer to EMGFeatures structure
     */
    void getExtendedFeatures(uint8_t ch, EMGFeatures* features) const;
    
    /**
     * @brief Debug functions
     */
    void printChannelStatus(uint8_t ch) const;
    void printAllChannels() const;
    
    /**
     * @brief Public feature array for other modules
     * Format: [ch0_rms, ch0_zc, ch0_ssc, ch0_wl, ch1_rms, ch1_zc, ...]
     */
    float emgFeatures[NUM_EMG_CHANNELS * EMG_FEATURES_PER_CHANNEL];

private:
    // Channel data structure
    struct ChannelData {
        // Signal processing chain
        float raw;              // Raw ADC value
        float filtered;         // After bandpass filtering
        float rectified;        // After full-wave rectification
        float rms;              // Root Mean Square over window
        float offset;           // DC offset (baseline)
        float norm;             // Normalized output (0.0-1.0)
        
        // Advanced features
        float zc;               // Zero Crossings count
        float ssc;              // Slope Sign Changes count
        float wl;               // Waveform Length
        
        // Circular buffers
        float rectifiedBuffer[EMG_BUFFER_SIZE];  // For RMS calculation
        float rawHistory[EMG_BUFFER_SIZE];       // For feature extraction
        uint16_t bufferIndex;                    // Current buffer position
        float windowSum;                         // Sum of rectified window
        float lastSample;                        // Previous sample for derivatives
    };
    
    // Channel data array
    ChannelData _channels[NUM_EMG_CHANNELS];
    
    // CMSIS-DSP filter structures (2-stage biquad cascade per channel)
    arm_biquad_casd_df1_inst_f32 _filters[NUM_EMG_CHANNELS];
    float _filterCoeffs[10 * NUM_EMG_CHANNELS];  // 10 coefficients per channel
    float _filterStates[8 * NUM_EMG_CHANNELS];   // 8 state variables per channel
    
    // Processing state
    volatile uint32_t _sampleIndex;
    volatile uint16_t _featureUpdateCounter;
    volatile bool _featuresNeedUpdate = false;
    bool _initialized;
    
    // Private methods
    void configureFilters();
    void processSampleRealtime(uint8_t ch, float sample);
    void extractAdvancedFeatures(uint8_t ch);
    void updatePublicFeatureArray();
};