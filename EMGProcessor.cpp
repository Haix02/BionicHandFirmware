/**
 * @file EMGProcessor.cpp
 * @brief Enhanced EMG signal processing with advanced feature extraction
 * @version 2.0
 */

#include "EMGProcessor.h"
#include <arm_math.h> // CMSIS-DSP library for efficient DSP operations

// Constructor
EMGProcessor::EMGProcessor() {
    // Initialize member variables
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        _channelData[ch].raw = 0.0f;
        _channelData[ch].filtered = 0.0f;
        _channelData[ch].rectified = 0.0f;
        _channelData[ch].rms = 0.0f;
        _channelData[ch].offset = 0.0f;
        _channelData[ch].norm = 0.0f;
        _channelData[ch].zc = 0.0f;
        _channelData[ch].ssc = 0.0f;
        _channelData[ch].wl = 0.0f;
        _channelData[ch].bufferIndex = 0;
        _channelData[ch].windowSum = 0.0f;
        
        // Initialize buffer with zeros
        for (uint16_t i = 0; i < EMG_BUFFER_SIZE; i++) {
            _channelData[ch].buffer[i] = 0.0f;
            _channelData[ch].history[i] = 0.0f;
        }
    }
    
    // Initialize public feature array
    for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
        for (uint8_t j = 0; j < EMG_FEATURES_PER_CHANNEL; j++) {
            emgFeatures[i * EMG_FEATURES_PER_CHANNEL + j] = 0.0f;
        }
    }
    
    // Configure DSP filters
    configureFilters();
}

void EMGProcessor::begin() {
    // Configure ADC pins for EMG inputs
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        pinMode(EMG_PINS[ch], INPUT);
    }
    
    // Set ADC resolution to 12 bits for Teensy 4.1
    analogReadResolution(12);
}

void EMGProcessor::configureFilters() {
    // Configure bandpass filter (20-500Hz) using CMSIS-DSP biquad
    const float fs = EMG_SAMPLE_HZ;     // Sampling frequency
    const float f_low = 20.0f;          // Lower cutoff
    const float f_high = 500.0f;        // Upper cutoff
    const float q = 0.707f;             // Q factor
    
    // Calculate normalized frequencies
    const float w_low = 2.0f * PI * f_low / fs;
    const float w_high = 2.0f * PI * f_high / fs;
    
    // Initialize filters for each channel
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        // Calculate biquad coefficients
        float* coeffs = _filterCoeffs + (ch * 5);  // 5 coefficients per biquad
        
        // Simple bandpass biquad coefficients
        coeffs[0] = 0.1f;                         // b0
        coeffs[1] = 0.0f;                         // b1
        coeffs[2] = -0.1f;                        // b2
        coeffs[3] = -1.8f * cosf(w_low);          // -a1
        coeffs[4] = 0.8f;                         // -a2
        
        // Initialize filter structure
        arm_biquad_cascade_df1_init_f32(
            &_filters[ch],
            1,                          // Single stage
            coeffs,                     // Filter coefficients
            _filterStates + (ch * 4)    // Filter state variables (4 per stage)
        );
    }
}

void EMGProcessor::sampleISR() {
    // This method is called from timer interrupt at EMG_SAMPLE_HZ rate
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        // Read ADC value
        uint16_t rawADC = analogRead(EMG_PINS[ch]);
        float rawSample = (float)rawADC;
        
        // Store raw value
        _channelData[ch].raw = rawSample;
        
        // Store in history buffer
        uint16_t idx = _channelData[ch].bufferIndex;
        _channelData[ch].history[idx] = rawSample;
        
        // Apply signal processing pipeline
        processSample(ch, rawSample);
        
        // Update buffer index (circular buffer)
        _channelData[ch].bufferIndex = (idx + 1) % EMG_BUFFER_SIZE;
    }
}

void EMGProcessor::processSample(uint8_t ch, float sample) {
    // 1. Apply bandpass filter (20-500 Hz)
    float filtered;
    arm_biquad_cascade_df1_f32(
        &_filters[ch],
        &sample,       // Input
        &filtered,     // Output
        1              // One sample
    );
    _channelData[ch].filtered = filtered;
    
    // 2. Apply full-wave rectification
    float rectified = fabsf(filtered);
    _channelData[ch].rectified = rectified;
    
    // 3. Update sliding window for RMS calculation
    uint16_t idx = _channelData[ch].bufferIndex;
    
    // Remove oldest sample from sum
    _channelData[ch].windowSum -= _channelData[ch].buffer[idx];
    
    // Add new rectified sample
    _channelData[ch].buffer[idx] = rectified;
    _channelData[ch].windowSum += rectified;
    
    // 4. Calculate RMS over window
    float meanSquare = _channelData[ch].windowSum / EMG_BUFFER_SIZE;
    _channelData[ch].rms = sqrtf(meanSquare);
    
    // 5. Apply normalization and offset
    float normalized = (_channelData[ch].rms - _channelData[ch].offset) / 1023.0f;
    _channelData[ch].norm = constrain(normalized, 0.0f, 1.0f);
}

void EMGProcessor::update() {
    // This is called in the main loop to update features
    static uint32_t lastFeatureUpdate = 0;
    uint32_t currentTime = millis();
    
    // Update features at a lower rate (50Hz is usually sufficient)
    if (currentTime - lastFeatureUpdate >= 20) {
        for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
            extractFeatures(ch);
        }
        lastFeatureUpdate = currentTime;
    }
}

void EMGProcessor::extractFeatures(uint8_t ch) {
    // Calculate Zero Crossing (ZC) count
    uint16_t zc_count = 0;
    float zc_threshold = 0.01f; // Small threshold to avoid noise
    
    // Iterate through history buffer
    for (uint16_t i = 1; i < EMG_BUFFER_SIZE; i++) {
        uint16_t curr_idx = (i + _channelData[ch].bufferIndex) % EMG_BUFFER_SIZE;
        uint16_t prev_idx = (i - 1 + _channelData[ch].bufferIndex) % EMG_BUFFER_SIZE;
        
        float prev = _channelData[ch].history[prev_idx] - _channelData[ch].offset;
        float curr = _channelData[ch].history[curr_idx] - _channelData[ch].offset;
        
        if ((prev > zc_threshold && curr < -zc_threshold) || 
            (prev < -zc_threshold && curr > zc_threshold)) {
            zc_count++;
        }
    }
    _channelData[ch].zc = (float)zc_count;
    
    // Calculate Slope Sign Changes (SSC)
    uint16_t ssc_count = 0;
    float ssc_threshold = 0.01f;
    
    for (uint16_t i = 1; i < EMG_BUFFER_SIZE - 1; i++) {
        uint16_t prev_idx = (i - 1 + _channelData[ch].bufferIndex) % EMG_BUFFER_SIZE;
        uint16_t curr_idx = (i + _channelData[ch].bufferIndex) % EMG_BUFFER_SIZE;
        uint16_t next_idx = (i + 1 + _channelData[ch].bufferIndex) % EMG_BUFFER_SIZE;
        
        float prev = _channelData[ch].history[prev_idx];
        float curr = _channelData[ch].history[curr_idx];
        float next = _channelData[ch].history[next_idx];
        
        float slope1 = curr - prev;
        float slope2 = next - curr;
        
        if ((slope1 > ssc_threshold && slope2 < -ssc_threshold) || 
            (slope1 < -ssc_threshold && slope2 > ssc_threshold)) {
            ssc_count++;
        }
    }
    _channelData[ch].ssc = (float)ssc_count;
    
    // Calculate Waveform Length (WL)
    float wl = 0.0f;
    for (uint16_t i = 1; i < EMG_BUFFER_SIZE; i++) {
        uint16_t curr_idx = (i + _channelData[ch].bufferIndex) % EMG_BUFFER_SIZE;
        uint16_t prev_idx = (i - 1 + _channelData[ch].bufferIndex) % EMG_BUFFER_SIZE;
        
        wl += fabsf(_channelData[ch].history[curr_idx] - _channelData[ch].history[prev_idx]);
    }
    _channelData[ch].wl = wl;
    
    // Update public feature array for external access
    // Format: [RMS, ZC, SSC, WL] per channel
    uint8_t baseIdx = ch * EMG_FEATURES_PER_CHANNEL;
    emgFeatures[baseIdx + 0] = _channelData[ch].norm; // Normalized RMS
    emgFeatures[baseIdx + 1] = _channelData[ch].zc / 100.0f; // Normalized ZC
    emgFeatures[baseIdx + 2] = _channelData[ch].ssc / 100.0f; // Normalized SSC
    emgFeatures[baseIdx + 3] = _channelData[ch].wl / 1000.0f; // Normalized WL
}

float EMGProcessor::getFeature(uint8_t ch) const {
    if (ch >= NUM_EMG_CHANNELS) return 0.0f;
    return _channelData[ch].norm;
}

void EMGProcessor::getFeatureVector(float* out) const {
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        out[ch] = _channelData[ch].norm;
    }
}

void EMGProcessor::setChannelOffset(uint8_t ch, float offset) {
    if (ch < NUM_EMG_CHANNELS) {
        _channelData[ch].offset = offset;
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
        _channelData[ch].offset = sum / SAMPLES;
        
        Serial.print(F("Channel "));
        Serial.print(ch);
        Serial.print(F(" offset: "));
        Serial.println(_channelData[ch].offset);
    }
    
    Serial.println(F("Calibration complete"));
}