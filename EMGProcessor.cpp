/**
 * @file EMGProcessor.cpp
 * @brief Enhanced EMG signal processing with advanced feature extraction
 * @version 2.0
 */

#include "EMGProcessor.h"
#include <arm_math.h> // CMSIS-DSP library

// Constructor implementation - initialize member variables
EMGProcessor::EMGProcessor() {
    // Initialize all channels
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        _channels[ch].raw = 0.0f;
        _channels[ch].filtered = 0.0f;
        _channels[ch].rectified = 0.0f;
        _channels[ch].rms = 0.0f;
        _channels[ch].offset = 0.0f;
        _channels[ch].norm = 0.0f;
        _channels[ch].bufferIndex = 0;
        _channels[ch].windowSum = 0.0f;
        _channels[ch].zc = 0.0f;
        _channels[ch].wl = 0.0f;
        _channels[ch].ssc = 0.0f;
        _channels[ch].active = false;
        
        // Initialize buffer and history
        for (uint16_t i = 0; i < EMG_BUFFER_SIZE; i++) {
            _channels[ch].buffer[i] = 0.0f;
            _channels[ch].history[i] = 0.0f;
        }
    }
    
    // Configure DSP filters
    configureFilters();
}

void EMGProcessor::begin() {
    // Set up analog pins for EMG input
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        pinMode(EMG_PINS[ch], INPUT);
    }
    
    // Calibrate baselines if needed
    calibrateRest();
}

void EMGProcessor::configureFilters() {
    // Configure bandpass filter (20-500Hz)
    // Using CMSIS-DSP biquad filter
    const float fs = EMG_SAMPLE_HZ;      // Sampling frequency (e.g., 1000Hz)
    const float f_low = 20.0f;           // Lower cutoff frequency
    const float f_high = 500.0f;         // Upper cutoff frequency
    const float q = EMG_BANDPASS_Q;      // Q factor (typically 0.707)
    
    // Calculate normalized frequencies
    const float w_low = 2.0f * PI * f_low / fs;
    const float w_high = 2.0f * PI * f_high / fs;
    
    // Initialize filters for each channel
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        // Calculate biquad coefficients for bandpass
        float* coeffs = _filterCoeffs + (ch * 5); // 5 coefficients per filter
        
        // Basic biquad bandpass coefficients (can be optimized further)
        coeffs[0] = 0.1f;                 // b0
        coeffs[1] = 0.0f;                 // b1
        coeffs[2] = -0.1f;                // b2
        coeffs[3] = -1.8f * cos(w_low);   // -a1
        coeffs[4] = 0.8f;                 // -a2
        
        // Initialize filter structure
        arm_biquad_cascade_df1_init_f32(
            &_filters[ch],            // Filter instance
            1,                        // 1-stage biquad
            coeffs,                   // Filter coefficients
            _filterStates + (ch * 4)  // Filter state variables (4 per filter)
        );
    }
}

void EMGProcessor::sampleISR() {
    // Called from timer interrupt at EMG_SAMPLE_HZ rate
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        // Read raw ADC value
        float raw = analogRead(EMG_PINS[ch]);
        
        // Store raw value in channel state and history buffer
        _channels[ch].raw = raw;
        _channels[ch].history[_channels[ch].bufferIndex] = raw;
        
        // Apply DSP pipeline
        processSample(ch, raw);
    }
}

void EMGProcessor::processSample(uint8_t ch, float sample) {
    // 1. Apply bandpass filter (20-500 Hz)
    float filtered;
    arm_biquad_cascade_df1_f32(
        &_filters[ch],
        &sample,        // Input sample
        &filtered,      // Output sample
        1               // Number of samples
    );
    _channels[ch].filtered = filtered;
    
    // 2. Apply full-wave rectification
    float rectified = fabs(filtered);
    _channels[ch].rectified = rectified;
    
    // 3. Update sliding window buffer
    uint16_t idx = _channels[ch].bufferIndex;
    
    // Remove oldest sample from sum
    _channels[ch].windowSum -= _channels[ch].buffer[idx];
    
    // Add new sample to buffer and sum
    _channels[ch].buffer[idx] = rectified;
    _channels[ch].windowSum += rectified;
    
    // Update buffer index (circular buffer)
    _channels[ch].bufferIndex = (idx + 1) % EMG_BUFFER_SIZE;
    
    // 4. Calculate RMS from window sum
    _channels[ch].rms = sqrt(_channels[ch].windowSum / EMG_BUFFER_SIZE);
    
    // 5. Detect activity based on RMS threshold
    _channels[ch].active = (_channels[ch].rms > EMG_THRESH);
    
    // 6. Calculate normalized output (threshold already applied)
    normalizeSignal(ch);
}

void EMGProcessor::update() {
    // Called from main loop at lower rate to extract features
    // This separates the time-critical sampling from feature extraction
    
    static uint32_t lastFeatureUpdate = 0;
    uint32_t now = millis();
    
    // Update features every 20ms (50Hz)
    if (now - lastFeatureUpdate >= 20) {
        for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
            updateFeatures(ch);
        }
        lastFeatureUpdate = now;
    }
}

void EMGProcessor::updateFeatures(uint8_t ch) {
    // Calculate Zero Crossing (ZC)
    // Number of times the signal crosses zero
    uint16_t zc = 0;
    float threshold = 0.015f; // Small threshold to avoid noise
    
    for (uint16_t i = 1; i < EMG_BUFFER_SIZE; i++) {
        uint16_t curr_idx = (i + _channels[ch].bufferIndex) % EMG_BUFFER_SIZE;
        uint16_t prev_idx = (i - 1 + _channels[ch].bufferIndex) % EMG_BUFFER_SIZE;
        
        float prev = _channels[ch].history[prev_idx] - _channels[ch].offset;
        float curr = _channels[ch].history[curr_idx] - _channels[ch].offset;
        
        if ((prev > threshold && curr < -threshold) || 
            (prev < -threshold && curr > threshold)) {
            zc++;
        }
    }
    _channels[ch].zc = (float)zc;
    
    // Calculate Waveform Length (WL)
    // Sum of absolute differences between adjacent samples
    float wl = 0.0f;
    for (uint16_t i = 1; i < EMG_BUFFER_SIZE; i++) {
        uint16_t curr_idx = (i + _channels[ch].bufferIndex) % EMG_BUFFER_SIZE;
        uint16_t prev_idx = (i - 1 + _channels[ch].bufferIndex) % EMG_BUFFER_SIZE;
        
        wl += fabs(_channels[ch].history[curr_idx] - _channels[ch].history[prev_idx]);
    }
    _channels[ch].wl = wl;
    
    // Calculate Slope Sign Change (SSC)
    // Number of times slope changes sign
    uint16_t ssc = 0;
    for (uint16_t i = 1; i < EMG_BUFFER_SIZE - 1; i++) {
        uint16_t prev_idx = (i - 1 + _channels[ch].bufferIndex) % EMG_BUFFER_SIZE;
        uint16_t curr_idx = (i + _channels[ch].bufferIndex) % EMG_BUFFER_SIZE;
        uint16_t next_idx = (i + 1 + _channels[ch].bufferIndex) % EMG_BUFFER_SIZE;
        
        float prev = _channels[ch].history[prev_idx];
        float curr = _channels[ch].history[curr_idx];
        float next = _channels[ch].history[next_idx];
        
        float slope1 = curr - prev;
        float slope2 = next - curr;
        
        if ((slope1 > threshold && slope2 < -threshold) || 
            (slope1 < -threshold && slope2 > threshold)) {
            ssc++;
        }
    }
    _channels[ch].ssc = (float)ssc;
    
    // Update feature vector for external access
    _featureVector[ch * FEATURES_PER_CHANNEL + 0] = _channels[ch].rms;
    _featureVector[ch * FEATURES_PER_CHANNEL + 1] = _channels[ch].zc / 100.0f;  // Normalize
    _featureVector[ch * FEATURES_PER_CHANNEL + 2] = _channels[ch].wl / 1000.0f; // Normalize
    _featureVector[ch * FEATURES_PER_CHANNEL + 3] = _channels[ch].ssc / 100.0f; // Normalize
}

void EMGProcessor::normalizeSignal(uint8_t ch) {
    // Calculate normalized output with offset removal and thresholding
    float value = _channels[ch].rms;
    value = (value - _channels[ch].offset) / EMG_NORM_MAX;
    
    // Apply threshold
    if (value < EMG_THRESH) {
        value = 0.0f;
    }
    
    // Clamp to range [0, 1]
    value = constrain(value, 0.0f, 1.0f);
    
    _channels[ch].norm = value;
}

float EMGProcessor::getFeature(uint8_t ch) const {
    // Return normalized EMG amplitude (original API maintained)
    return (ch < NUM_EMG_CHANNELS) ? _channels[ch].norm : 0.0f;
}

void EMGProcessor::getFeatureVector(float* out) const {
    // Copy normalized values to output array (original API maintained)
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        out[ch] = _channels[ch].norm;
    }
}

const float* EMGProcessor::getAllFeatures() const {
    // Return pointer to the entire feature vector
    return _featureVector;
}

void EMGProcessor::getExtendedFeatures(uint8_t ch, ExtendedFeatures* features) const {
    if (ch >= NUM_EMG_CHANNELS || !features) return;
    
    features->rms = _channels[ch].rms;
    features->zc = _channels[ch].zc;
    features->wl = _channels[ch].wl;
    features->ssc = _channels[ch].ssc;
    features->isActive = _channels[ch].active;
}

void EMGProcessor::setChannelOffset(uint8_t ch, float offset) {
    if (ch < NUM_EMG_CHANNELS) {
        _channels[ch].offset = offset;
    }
}

void EMGProcessor::calibrateRest() {
    Serial.println(F("Calibrating EMG rest levels..."));
    
    // Take multiple samples to establish baseline
    const uint16_t SAMPLES = 100;
    
    // For each channel, average the raw signal to establish offset
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        float sum = 0.0f;
        
        // Collect samples
        for (uint16_t i = 0; i < SAMPLES; i++) {
            sum += analogRead(EMG_PINS[ch]);
            delay(1); // Small delay between samples
        }
        
        // Set offset to average
        _channels[ch].offset = sum / SAMPLES;
        
        Serial.print(F("Channel "));
        Serial.print(ch);
        Serial.print(F(" offset: "));
        Serial.println(_channels[ch].offset);
    }
    
    Serial.println(F("Calibration complete"));
}

bool EMGProcessor::isChannelActive(uint8_t ch) const {
    return (ch < NUM_EMG_CHANNELS) ? _channels[ch].active : false;
}

float EMGProcessor::getRMS(uint8_t ch) const {
    return (ch < NUM_EMG_CHANNELS) ? _channels[ch].rms : 0.0f;
}