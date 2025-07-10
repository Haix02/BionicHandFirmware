/**
 * @file EMGProcessor.h
 * @brief Enhanced EMG signal processing with advanced feature extraction
 * @version 2.0
 */

#pragma once

#include <Arduino.h>
#include "config.h"
#include <arm_math.h> // CMSIS-DSP library

// Number of features per channel
#define EMG_FEATURES_PER_CHANNEL 4 // RMS, ZC, SSC, WL

class EMGProcessor {
public:
    EMGProcessor();
    
    /**
     * @brief Initialize hardware and filters
     */
    void begin();
    
    /**
     * @brief Process new samples (called from timer ISR)
     */
    void sampleISR();
    
    /**
     * @brief Update feature extraction (called from main loop)
     */
    void update();
    
    /**
     * @brief Get normalized amplitude for a channel
     * @param ch Channel index
     * @return Normalized amplitude (0.0-1.0)
     */
    float getFeature(uint8_t ch) const;
    
    /**
     * @brief Fill array with normalized amplitudes for all channels
     * @param out Output array (must be at least NUM_EMG_CHANNELS in size)
     */
    void getFeatureVector(float* out) const;
    
    /**
     * @brief Set DC offset for a channel
     * @param ch Channel index
     * @param offset Offset value
     */
    void setChannelOffset(uint8_t ch, float offset);
    
    /**
     * @brief Calibrate resting levels for all channels
     */
    void calibrateRest();
    
    /**
     * @brief Public array of all extracted features
     * Format: [ch0_rms, ch0_zc, ch0_ssc, ch0_wl, ch1_rms, ch1_zc, ...]
     */
    float emgFeatures[NUM_EMG_CHANNELS * EMG_FEATURES_PER_CHANNEL];

private:
    // Channel data structure
    struct ChannelData {
        float raw;             // Raw ADC value
        float filtered;        // After bandpass filtering
        float rectified;       // After rectification
        float rms;             // Root Mean Square
        float offset;          // DC offset/baseline
        float norm;            // Normalized output (0.0-1.0)
        float zc;              // Zero Crossing count
        float ssc;             // Slope Sign Changes
        float wl;              // Waveform Length
        
        // Circular buffer for sliding window
        float buffer[EMG_BUFFER_SIZE];
        float history[EMG_BUFFER_SIZE];
        uint16_t bufferIndex;
        float windowSum;
    };
    
    // Array of channel data
    ChannelData _channelData[NUM_EMG_CHANNELS];
    
    // CMSIS-DSP filter structures
    arm_biquad_casd_df1_inst_f32 _filters[NUM_EMG_CHANNELS];
    float _filterCoeffs[5 * NUM_EMG_CHANNELS]; // 5 coefficients per filter
    float _filterStates[4 * NUM_EMG_CHANNELS]; // 4 state variables per filter
    
    // Private methods
    void configureFilters();
    void processSample(uint8_t ch, float sample);
    void extractFeatures(uint8_t ch);
};