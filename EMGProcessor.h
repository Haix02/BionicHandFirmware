/**
 * @file EMGProcessor.h
 * @brief Enhanced EMG signal processing with real-time DSP features
 */

#pragma once

#include <Arduino.h>
#include "config.h"
#include <arm_math.h>

#define FEATURE_VECTOR_SIZE 3 // RMS, ZC, SSC

class EMGProcessor {
public:
    EMGProcessor();
    
    // Original public interface
    void begin();
    void sampleISR();   // Called from IntervalTimer
    void update();      // Call in main loop
    
    float getFeature(uint8_t ch) const;
    void getFeatureVector(float *out) const;
    
    void setChannelOffset(uint8_t ch, float offset);
    void calibrateRest();
    
    // New enhanced methods
    void getExtendedFeatures(uint8_t ch, float* features, uint8_t numFeatures) const;
    bool isActivityDetected(uint8_t ch) const;
    float getRMS(uint8_t ch) const;
    float getZeroCrossings(uint8_t ch) const;
    float getSlopeSignChanges(uint8_t ch) const;

private:
    struct EMGChannelState {
        float raw;          // Most recent raw sample
        float filtered;     // Filtered sample
        float rectified;    // Rectified sample
        float movavg_sum;   // For moving average calculation
        float movavg_buf[EMG_MOVAVG_WINDOW];  // Moving average buffer
        uint16_t movavg_idx;     // Current index in movavg buffer
        float raw_history[EMG_MOVAVG_WINDOW]; // Raw signal history
        float offset;       // Baseline offset
        float norm;         // Normalized output (0.0-1.0)
        float rms;          // Root mean square
        uint16_t zero_crossings;  // Zero crossing count
        uint16_t slope_sign_changes; // Slope sign change count
        bool activity_detected;  // Activity detection flag
        float feature_vector[FEATURE_VECTOR_SIZE]; // Extended feature vector
    };
    
    EMGChannelState channels[NUM_EMG_CHANNELS];
    
    // DSP components
    arm_biquad_casd_df1_inst_f32 bandpassFilters[NUM_EMG_CHANNELS];
    float filterCoeffs[5*NUM_EMG_CHANNELS]; // 5 coefficients per filter
    float filterStates[4*NUM_EMG_CHANNELS]; // 4 state variables per filter
    
    // Private methods
    void configureFilters();
    void applyDSP(uint8_t ch, float raw);
    void updateFeatures(uint8_t ch);
    void updateMovingAverage(uint8_t ch, float val);
    void normalizeAndThreshold(uint8_t ch);
    void detectActivity(uint8_t ch);
};