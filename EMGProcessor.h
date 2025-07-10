/**
 * @file EMGProcessor.h
 * @brief Enhanced EMG signal processing with DMA and advanced DSP
 * @version 3.0
 */

#pragma once

#include <Arduino.h>
#include "config.h"

// Add CMSIS DSP library for efficient signal processing
#include <arm_math.h>

/**
 * @class EMGProcessor
 * @brief Handles sampling, filtering, and feature extraction for EMG signals.
 * 
 * V3 Enhancements:
 * - DMA-based high-speed sampling
 * - Real-time DSP with CMSIS for optimized processing
 * - Adaptive windowing for activity segmentation
 * - Enhanced feature extraction (RMS, ZC, MAV, etc.)
 */
class EMGProcessor {
public:
    EMGProcessor();

    // Maintain existing public interface
    void begin();
    void sampleISR();   ///< Called from IntervalTimer (legacy support)
    void process();     ///< Call in main loop to process DSP pipeline

    float getFeature(uint8_t ch) const;
    void getFeatureVector(float *out) const;

    void setChannelOffset(uint8_t ch, float offset);
    void calibrateRest(); ///< Sets offsets to resting average
    
    // V3 Enhanced methods
    bool isActivityDetected(uint8_t ch) const;
    float getZeroCrossings(uint8_t ch) const;
    float getWaveformLength(uint8_t ch) const;
    float getRMS(uint8_t ch) const;
    void enableDMA(bool enable);
    void setAdaptiveThreshold(float ratio);

private:
    // Original data structures (maintain compatibility)
    struct EMGChannelState {
        float raw;             ///< Most recent raw sample
        float bandpassed;      ///< After IIR bandpass
        float rectified;       ///< After rectification
        float movavg_sum;      ///< For moving average
        float movavg_buf[EMG_MOVAVG_WINDOW];
        uint16_t movavg_idx;
        float rms_sum_sq;      ///< For RMS
        float rms;             ///< RMS over window
        float offset;          ///< Resting offset (for normalization)
        float norm;            ///< Normalized, thresholded output
        
        // V3 Enhanced features
        float zero_crossings;  ///< Zero crossing count in window
        float waveform_length; ///< Waveform length feature
        bool activity_detected;///< Active EMG segment detected
        float adaptive_threshold; ///< Dynamic threshold for activity
    };

    EMGChannelState channels[NUM_EMG_CHANNELS];

    // V3 Enhanced DMA structures
    static DMACh dmaChannel;
    static volatile uint16_t dmaBuffer[2][NUM_EMG_CHANNELS]; // Double-buffered DMA
    static volatile bool dmaBufferReady;
    static volatile uint8_t currentDmaBuffer;
    bool dmaModeEnabled;

    // Enhanced DSP pipeline
    // Using CMSIS-DSP for efficient filtering
    arm_biquad_casd_df1_inst_f32 bandpassFilter[NUM_EMG_CHANNELS];
    float bandpassCoeffs[5*NUM_EMG_CHANNELS]; // 5 coeffs per filter
    float bandpassState[4*NUM_EMG_CHANNELS];  // 4 state variables per filter
    
    // Adaptive windowing parameters
    float adaptiveThresholdRatio;
    uint16_t activityWindowStart[NUM_EMG_CHANNELS];
    uint16_t activityWindowSize[NUM_EMG_CHANNELS];
    
    // Enhanced processing methods
    void setupDMA();
    void processDMABuffer();
    static void dmaISR();
    
    void configureFilters();
    void applyDSPPipeline(uint8_t ch, float sample);
    void updateFeatures(uint8_t ch);
    void detectActivity(uint8_t ch);
    
    // Original methods (enhance internally)
    void updateMovingAverage(uint8_t ch, float val);
    void updateRMS(uint8_t ch, float val);
    void normalizeAndThreshold(uint8_t ch);
};