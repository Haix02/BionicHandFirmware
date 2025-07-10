/**
 * @file EMGProcessor.cpp
 * @brief Enhanced EMG signal processing implementation
 * @version 2.0
 */

#include "EMGProcessor.h"

EMGProcessor::EMGProcessor() : _activityThreshold(0.15f) {
    // Initialize channel states
    memset(channels, 0, sizeof(channels));
}

void EMGProcessor::begin() {
    // Configure all EMG pins as inputs
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        pinMode(EMG_PINS[ch], INPUT);
    }
    
    // Set up DSP filters
    configureFilters();
}

void EMGProcessor::configureFilters() {
    // Configure bandpass filter (20-500Hz) using CMSIS-DSP
    // Assuming 1kHz sampling rate
    const float fs = 1000.0f;  // Sampling frequency
    const float f_low = 20.0f;  // Lower cutoff
    const float f_high = 500.0f; // Upper cutoff
    const float q = 0.707f;     // Q factor
    
    // Calculate normalized frequencies
    const float w_low = 2.0f * M_PI * f_low / fs;
    const float w_high = 2.0f * M_PI * f_high / fs;
    
    // Initialize a filter for each channel
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        // Calculate biquad filter coefficients
        float* coeffs = &filterCoeffs[ch * 5];
        
        // Bandpass filter coefficients (simplified biquad)
        // This creates a basic bandpass filter - optimize these coefficients for your specific needs
        coeffs[0] = 0.1f;                 // b0
        coeffs[1] = 0.0f;                 // b1
        coeffs[2] = -0.1f;                // b2
        coeffs[3] = -1.8f * cos(w_low);   // a1 (negated)
        coeffs[4] = 0.8f;                 // a2 (negated)
        
        // Initialize filter structure
        arm_biquad_cascade_df1_init_f32(
            &bandpassFilters[ch], 
            1,                      // 1-stage filter
            coeffs,                 // Filter coefficients
            &filterStates[ch*4]     // State variables
        );
    }
    
    // Initialize channel states
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        channels[ch].offset = 0.0f;
        channels[ch].activity_detected = false;
        
        // Clear history buffers
        for (int i = 0; i < EMG_MOVAVG_WINDOW; i++) {
            channels[ch].movavg_buf[i] = 0.0f;
            channels[ch].raw_history[i] = 0.0f;
        }
    }
}

void EMGProcessor::sampleISR() {
    // Called from IntervalTimer at EMG_SAMPLE_HZ frequency
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        // Read ADC
        float raw = analogRead(EMG_PINS[ch]);
        channels[ch].raw = raw;
        
        // Store raw in history buffer
        channels[ch].raw_history[channels[ch].movavg_idx] = raw;
        
        // Apply bandpass filter immediately in the ISR
        applyBandpass(ch, raw);
    }
}

void EMGProcessor::applyBandpass(uint8_t ch, float sample) {
    float filtered;
    
    // Apply CMSIS-DSP biquad filter
    arm_biquad_cascade_df1_f32(
        &bandpassFilters[ch],
        &sample,      // Input sample
        &filtered,    // Output sample
        1             // Process one sample
    );
    
    // Store filtered result
    channels[ch].filtered = filtered;
    
    // Apply full-wave rectification
    channels[ch].rectified = fabs(filtered);
    
    // Update moving average with rectified value
    updateMovingAverage(ch, channels[ch].rectified);
}

void EMGProcessor::process() {
    // Main processing function called from loop()
    // Performs additional processing and feature extraction
    
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        // Calculate advanced features
        updateFeatures(ch);
        
        // Detect activity segments
        detectActivity(ch);
        
        // Normalize and apply threshold
        normalizeAndThreshold(ch);
    }
}

void EMGProcessor::updateFeatures(uint8_t ch) {
    EMGChannelState& state = channels[ch];
    
    // Calculate RMS
    float sum_squared = 0;
    for (int i = 0; i < EMG_MOVAVG_WINDOW; i++) {
        float val = state.movavg_buf[i];
        sum_squared += val * val;
    }
    state.rms = sqrt(sum_squared / EMG_MOVAVG_WINDOW);
    
    // Calculate Zero Crossings
    int zc_count = 0;
    float zc_threshold = 0.01f; // Small threshold to avoid noise
    
    for (int i = 1; i < EMG_MOVAVG_WINDOW; i++) {
        int prev_idx = (state.movavg_idx + i - 1) % EMG_MOVAVG_WINDOW;
        int curr_idx = (state.movavg_idx + i) % EMG_MOVAVG_WINDOW;
        
        float prev = state.raw_history[prev_idx] - state.offset;
        float curr = state.raw_history[curr_idx] - state.offset;
        
        if ((prev > zc_threshold && curr < -zc_threshold) || 
            (prev < -zc_threshold && curr > zc_threshold)) {
            zc_count++;
        }
    }
    state.zero_crossings = (float)zc_count;
    
    // Calculate Slope Sign Changes
    int ssc_count = 0;
    float ssc_threshold = 0.01f;
    
    for (int i = 1; i < EMG_MOVAVG_WINDOW - 1; i++) {
        int prev_idx = (state.movavg_idx + i - 1) % EMG_MOVAVG_WINDOW;
        int curr_idx = (state.movavg_idx + i) % EMG_MOVAVG_WINDOW;
        int next_idx = (state.movavg_idx + i + 1) % EMG_MOVAVG_WINDOW;
        
        float prev = state.raw_history[prev_idx] - state.offset;
        float curr = state.raw_history[curr_idx] - state.offset;
        float next = state.raw_history[next_idx] - state.offset;
        
        float slope1 = curr - prev;
        float slope2 = next - curr;
        
        if ((slope1 > ssc_threshold && slope2 < -ssc_threshold) || 
            (slope1 < -ssc_threshold && slope2 > ssc_threshold)) {
            ssc_count++;
        }
    }
    state.slope_sign_changes = (float)ssc_count;
}

void EMGProcessor::detectActivity(uint8_t ch) {
    EMGChannelState& state = channels[ch];
    
    // Use RMS energy to detect activity segments
    state.activity_detected = (state.rms > _activityThreshold);
}

void EMGProcessor::updateMovingAverage(uint8_t ch, float val) {
    EMGChannelState& state = channels[ch];
    
    // Remove oldest sample
    state.movavg_sum -= state.movavg_buf[state.movavg_idx];
    
    // Add new sample
    state.movavg_buf[state.movavg_idx] = val;
    state.movavg_sum += val;
    
    // Update index (circular buffer)
    state.movavg_idx = (state.movavg_idx + 1) % EMG_MOVAVG_WINDOW;
}

void EMGProcessor::normalizeAndThreshold(uint8_t ch) {
    EMGChannelState& state = channels[ch];
    
    // Calculate moving average
    float ma = state.movavg_sum / EMG_MOVAVG_WINDOW;
    
    // Apply offset and normalize
    float norm = (ma - state.offset) / 1024.0f;
    norm = constrain(norm, 0.0f, 1.0f);
    
    // Apply threshold only during detected activity
    if (state.activity_detected) {
        norm = (norm > _activityThreshold) ? norm : 0.0f;
    } else {
        norm = 0.0f;
    }
    
    state.norm = norm;
}

float EMGProcessor::getFeature(uint8_t ch) const {
    // Return the normalized EMG amplitude (maintains original API)
    return channels[ch].norm;
}

void EMGProcessor::getFeatureVector(float *out) const {
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        out[ch] = channels[ch].norm;
    }
}

void EMGProcessor::setChannelOffset(uint8_t ch, float offset) {
    if (ch < NUM_EMG_CHANNELS) {
        channels[ch].offset = offset;
    }
}

void EMGProcessor::calibrateRest() {
    // Set offsets to current resting values
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        float rest_value = 0.0f;
        
        // Average raw samples to get resting level
        for (int i = 0; i < EMG_MOVAVG_WINDOW; i++) {
            rest_value += channels[ch].raw_history[i];
        }
        rest_value /= EMG_MOVAVG_WINDOW;
        
        channels[ch].offset = rest_value;
    }
}

// New enhanced methods

bool EMGProcessor::isActivityDetected(uint8_t ch) const {
    return (ch < NUM_EMG_CHANNELS) ? channels[ch].activity_detected : false;
}

float EMGProcessor::getZeroCrossings(uint8_t ch) const {
    return (ch < NUM_EMG_CHANNELS) ? channels[ch].zero_crossings : 0.0f;
}

float EMGProcessor::getSlopeSignChanges(uint8_t ch) const {
    return (ch < NUM_EMG_CHANNELS) ? channels[ch].slope_sign_changes : 0.0f;
}

float EMGProcessor::getRMS(uint8_t ch) const {
    return (ch < NUM_EMG_CHANNELS) ? channels[ch].rms : 0.0f;
}

void EMGProcessor::setActivityThreshold(float threshold) {
    _activityThreshold = constrain(threshold, 0.05f, 0.5f);
}