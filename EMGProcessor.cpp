/**
 * @file EMGProcessor.cpp
 * @brief Enhanced EMG signal processing with real-time DSP features
 */

#include "EMGProcessor.h"
#include <arm_math.h>  // CMSIS DSP library for Teensy 4.1

// Constructor remains unchanged
EMGProcessor::EMGProcessor() {
    // Initialize state variables
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        channels[ch].offset = 0.0f;
        channels[ch].rms = 0.0f;
        channels[ch].zero_crossings = 0;
        channels[ch].slope_sign_changes = 0;
        channels[ch].activity_detected = false;
        channels[ch].movavg_idx = 0;
        channels[ch].movavg_sum = 0.0f;
        memset(channels[ch].movavg_buf, 0, sizeof(channels[ch].movavg_buf));
        memset(channels[ch].raw_history, 0, sizeof(channels[ch].raw_history));
    }
    
    // Initialize filter coefficients
    configureFilters();
}

void EMGProcessor::configureFilters() {
    // Configure bandpass filter (20-500Hz) using CMSIS-DSP
    const float fs = EMG_SAMPLE_HZ;  // Sampling frequency
    const float f_low = 20.0f;       // Lower cutoff
    const float f_high = 500.0f;     // Upper cutoff
    const float q = EMG_BANDPASS_Q;  // Q factor
    
    // Calculate normalized frequencies
    const float w_low = 2.0f * M_PI * f_low / fs;
    const float w_high = 2.0f * M_PI * f_high / fs;
    
    // Initialize filters for each channel
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        // Allocate filter coefficients for biquad filter
        float* coeffs = &filterCoeffs[ch * 5];
        
        // Calculate biquad bandpass coefficients
        // This is a simplified implementation - can be optimized further
        coeffs[0] = 0.1f;                    // b0
        coeffs[1] = 0.0f;                    // b1
        coeffs[2] = -0.1f;                   // b2
        coeffs[3] = -1.98f * cos(w_low);     // -a1
        coeffs[4] = 0.96f;                   // -a2
        
        // Initialize filter structure
        arm_biquad_cascade_df1_init_f32(
            &bandpassFilters[ch],
            1,                    // 1-stage filter
            coeffs,               // Filter coefficients
            &filterStates[ch*4]   // State variables (4 per biquad stage)
        );
    }
}

void EMGProcessor::begin() {
    // Set up ADC pins
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        pinMode(EMG_PINS[ch], INPUT);
    }
}

void EMGProcessor::sampleISR() {
    // Called at EMG_SAMPLE_HZ frequency (e.g., 1000 Hz)
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        // Read raw EMG sample
        float raw = analogRead(EMG_PINS[ch]);
        
        // Store raw sample in history buffer
        channels[ch].raw_history[channels[ch].movavg_idx] = raw;
        
        // Apply DSP pipeline
        applyDSP(ch, raw);
    }
}

void EMGProcessor::applyDSP(uint8_t ch, float raw) {
    // 1. Apply bandpass filter (20-500 Hz)
    float filtered;
    arm_biquad_cascade_df1_f32(
        &bandpassFilters[ch],
        &raw,                // Input sample
        &filtered,           // Output sample
        1                    // Process 1 sample
    );
    
    channels[ch].filtered = filtered;
    
    // 2. Apply full-wave rectification
    float rectified = fabs(filtered);
    channels[ch].rectified = rectified;
    
    // 3. Update moving average with rectified value
    updateMovingAverage(ch, rectified);
}

void EMGProcessor::update() {
    // Called from main loop to update features
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        // Calculate features
        updateFeatures(ch);
        
        // Detect activity
        detectActivity(ch);
        
        // Apply normalization and thresholding
        normalizeAndThreshold(ch);
    }
}

void EMGProcessor::updateFeatures(uint8_t ch) {
    // Calculate RMS over moving average window
    float sum_squared = 0.0f;
    for (uint16_t i = 0; i < EMG_MOVAVG_WINDOW; ++i) {
        float sample = channels[ch].movavg_buf[i];
        sum_squared += sample * sample;
    }
    channels[ch].rms = sqrt(sum_squared / EMG_MOVAVG_WINDOW);
    
    // Calculate Zero Crossings (ZC)
    uint16_t zc_count = 0;
    float threshold = 0.01f; // Small threshold to avoid noise
    
    for (uint16_t i = 1; i < EMG_MOVAVG_WINDOW; ++i) {
        uint16_t curr_idx = (channels[ch].movavg_idx + i) % EMG_MOVAVG_WINDOW;
        uint16_t prev_idx = (channels[ch].movavg_idx + i - 1) % EMG_MOVAVG_WINDOW;
        
        float curr = channels[ch].raw_history[curr_idx] - channels[ch].offset;
        float prev = channels[ch].raw_history[prev_idx] - channels[ch].offset;
        
        if ((prev > threshold && curr < -threshold) || 
            (prev < -threshold && curr > threshold)) {
            zc_count++;
        }
    }
    channels[ch].zero_crossings = zc_count;
    
    // Calculate Slope Sign Changes (SSC)
    uint16_t ssc_count = 0;
    
    for (uint16_t i = 1; i < EMG_MOVAVG_WINDOW - 1; ++i) {
        uint16_t prev_idx = (channels[ch].movavg_idx + i - 1) % EMG_MOVAVG_WINDOW;
        uint16_t curr_idx = (channels[ch].movavg_idx + i) % EMG_MOVAVG_WINDOW;
        uint16_t next_idx = (channels[ch].movavg_idx + i + 1) % EMG_MOVAVG_WINDOW;
        
        float prev = channels[ch].raw_history[prev_idx] - channels[ch].offset;
        float curr = channels[ch].raw_history[curr_idx] - channels[ch].offset;
        float next = channels[ch].raw_history[next_idx] - channels[ch].offset;
        
        float slope1 = curr - prev;
        float slope2 = next - curr;
        
        if ((slope1 > threshold && slope2 < -threshold) || 
            (slope1 < -threshold && slope2 > threshold)) {
            ssc_count++;
        }
    }
    channels[ch].slope_sign_changes = ssc_count;
    
    // Store features in vector (for future pattern recognition)
    channels[ch].feature_vector[0] = channels[ch].rms;
    channels[ch].feature_vector[1] = channels[ch].zero_crossings / (float)EMG_MOVAVG_WINDOW;
    channels[ch].feature_vector[2] = channels[ch].slope_sign_changes / (float)EMG_MOVAVG_WINDOW;
}

void EMGProcessor::detectActivity(uint8_t ch) {
    // Detect EMG activity based on RMS threshold
    channels[ch].activity_detected = (channels[ch].rms > EMG_THRESH);
}

void EMGProcessor::updateMovingAverage(uint8_t ch, float val) {
    // Remove oldest sample
    channels[ch].movavg_sum -= channels[ch].movavg_buf[channels[ch].movavg_idx];
    
    // Add new sample
    channels[ch].movavg_buf[channels[ch].movavg_idx] = val;
    channels[ch].movavg_sum += val;
    
    // Update index
    channels[ch].movavg_idx = (channels[ch].movavg_idx + 1) % EMG_MOVAVG_WINDOW;
}

void EMGProcessor::normalizeAndThreshold(uint8_t ch) {
    // Calculate moving average
    float ma = channels[ch].movavg_sum / EMG_MOVAVG_WINDOW;
    
    // Normalize and apply offset
    float norm = (ma - channels[ch].offset) / EMG_NORM_MAX;
    norm = constrain(norm, 0.0f, 1.0f);
    
    // Apply threshold
    if (norm < EMG_THRESH) {
        norm = 0.0f;
    }
    
    // Store normalized value
    channels[ch].norm = norm;
}

float EMGProcessor::getFeature(uint8_t ch) const {
    // Return normalized amplitude (original API maintained)
    return channels[ch].norm;
}

void EMGProcessor::getFeatureVector(float *out) const {
    // Copy normalized values to output array
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        out[ch] = channels[ch].norm;
    }
}

void EMGProcessor::getExtendedFeatures(uint8_t ch, float* features, uint8_t numFeatures) const {
    // Provide access to the extended feature vector
    uint8_t count = min(numFeatures, FEATURE_VECTOR_SIZE);
    for (uint8_t i = 0; i < count; ++i) {
        features[i] = channels[ch].feature_vector[i];
    }
}

void EMGProcessor::setChannelOffset(uint8_t ch, float offset) {
    if (ch < NUM_EMG_CHANNELS) {
        channels[ch].offset = offset;
    }
}

void EMGProcessor::calibrateRest() {
    // Calculate average of current signal as offset
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        float avg = 0.0f;
        for (uint16_t i = 0; i < EMG_MOVAVG_WINDOW; ++i) {
            avg += channels[ch].raw_history[i];
        }
        channels[ch].offset = avg / EMG_MOVAVG_WINDOW;
    }
}

bool EMGProcessor::isActivityDetected(uint8_t ch) const {
    return channels[ch].activity_detected;
}

float EMGProcessor::getRMS(uint8_t ch) const {
    return channels[ch].rms;
}

float EMGProcessor::getZeroCrossings(uint8_t ch) const {
    return channels[ch].zero_crossings;
}

float EMGProcessor::getSlopeSignChanges(uint8_t ch) const {
    return channels[ch].slope_sign_changes;
}