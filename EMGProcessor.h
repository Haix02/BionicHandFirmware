/**
 * @file EMGProcessor.h
 * @brief Enhanced EMG signal processing with advanced DSP features
 * @version 2.0
 */

#pragma once

#include <Arduino.h>
#include "config.h"

// Add CMSIS DSP library for optimized signal processing
#include <arm_math.h>

class EMGProcessor {
public:
    EMGProcessor();
    
    // Original public interface (maintained for compatibility)
    void begin();
    void sampleISR();   // Called from IntervalTimer
    void process();     // Call in main loop to process DSP pipeline
    
    float getFeature(uint8_t ch) const;
    void getFeatureVector(float *out) const;
    
    void setChannelOffset(uint8_t ch, float offset);
    void calibrateRest();
    
    // New enhanced methods
    bool isActivityDetected(uint8_t ch) const;
    float getZeroCrossings(uint8_t ch) const;
    float getSlopeSignChanges(uint8_t ch) const;
    float getRMS(uint8_t ch) const;
    void setActivityThreshold(float threshold);

private:
    struct EMGChannelState {
        float raw;             // Most recent raw sample
        float filtered;        // After bandpass filter
        float rectified;       // After rectification
        float movavg_sum;      // For moving average
        float movavg_buf[EMG_MOVAVG_WINDOW];
        uint16_t movavg_idx;
        float rms;             // RMS over window
        float offset;          // Resting offset (for normalization)
        float norm;            // Normalized, thresholded output
        
        // Enhanced features
        float zero_crossings;  // ZC feature
        float slope_sign_changes; // SSC feature
        bool activity_detected;// Activity state
        float raw_history[EMG_MOVAVG_WINDOW]; // Raw signal history
    };

    EMGChannelState channels[NUM_EMG_CHANNELS];
    
    // DSP components
    arm_biquad_casd_df1_inst_f32 bandpassFilters[NUM_EMG_CHANNELS];
    float filterCoeffs[5*NUM_EMG_CHANNELS]; // 5 coefficients per filter
    float filterStates[4*NUM_EMG_CHANNELS]; // 4 state variables per filter
    
    // Activity detection parameters
    float _activityThreshold;
    
    // Enhanced processing methods
    void configureFilters();
    void applyBandpass(uint8_t ch, float sample);
    void updateFeatures(uint8_t ch);
    void detectActivity(uint8_t ch);
    
    // Original processing methods (enhanced internally)
    void updateMovingAverage(uint8_t ch, float val);
    void normalizeAndThreshold(uint8_t ch);
};