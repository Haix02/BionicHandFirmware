/**
 * @file EMGProcessor.cpp
 * @brief Enhanced EMG signal processing with real-time DSP features
 * @version 2.0
 * @author Haix02
 * @date 2025-07-10
 */

#include "EMGProcessor.h"
#include <arm_math.h> // CMSIS-DSP library

// Constructor
EMGProcessor::EMGProcessor() 
    : _sampleIndex(0),
      _featureUpdateCounter(0),
      _initialized(false)
{
    // Initialize all channel data
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        _channels[ch].raw = 0.0f;
        _channels[ch].filtered = 0.0f;
        _channels[ch].rectified = 0.0f;
        _channels[ch].rms = 0.0f;
        _channels[ch].offset = 0.0f;
        _channels[ch].norm = 0.0f;
        _channels[ch].zc = 0.0f;
        _channels[ch].ssc = 0.0f;
        _channels[ch].wl = 0.0f;
        _channels[ch].bufferIndex = 0;
        _channels[ch].windowSum = 0.0f;
        _channels[ch].lastSample = 0.0f;
        
        // Initialize circular buffers
        for (uint16_t i = 0; i < EMG_BUFFER_SIZE; i++) {
            _channels[ch].rectifiedBuffer[i] = 0.0f;
            _channels[ch].rawHistory[i] = 0.0f;
        }
    }
    
    // Initialize public feature array
    memset(emgFeatures, 0, sizeof(emgFeatures));
    
    // Configure DSP filters
    configureFilters();
}

void EMGProcessor::begin() {
    // Configure ADC pins for EMG inputs
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        pinMode(EMG_PINS[ch], INPUT);
    }
    
    // Set ADC resolution and averaging for Teensy 4.1
    analogReadResolution(12);           // 12-bit resolution (0-4095)
    analogReadAveraging(4);             // Average 4 samples for noise reduction
    
    // Initial calibration
    delay(100); // Brief settling time
    calibrateRest();
    
    _initialized = true;
}

void EMGProcessor::configureFilters() {
    // Configure bandpass filter (20-500Hz) using CMSIS-DSP biquad cascade
    const float fs = EMG_SAMPLE_HZ;     // Sampling frequency (2000 Hz)
    const float f_low = 20.0f;          // Lower cutoff frequency
    const float f_high = 500.0f;        // Upper cutoff frequency
    
    // Calculate normalized frequencies
    const float w_low = 2.0f * PI * f_low / fs;
    const float w_high = 2.0f * PI * f_high / fs;
    
    // Design butterworth bandpass filter coefficients
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        float* coeffs = &_filterCoeffs[ch * 10]; // 10 coefficients per channel (2 biquad stages)
        float* states = &_filterStates[ch * 8];  // 8 state variables per channel (4 per stage)
        
        // First stage - High pass at 20 Hz
        float wc_hp = tanf(w_low / 2.0f);
        float k1_hp = sqrtf(2.0f) * wc_hp;
        float k2_hp = wc_hp * wc_hp;
        float a0_hp = k2_hp + k1_hp + 1.0f;
        
        coeffs[0] = 1.0f / a0_hp;                    // b0
        coeffs[1] = -2.0f / a0_hp;                   // b1
        coeffs[2] = 1.0f / a0_hp;                    // b2
        coeffs[3] = (2.0f * (k2_hp - 1.0f)) / a0_hp; // -a1
        coeffs[4] = (k2_hp - k1_hp + 1.0f) / a0_hp;  // -a2
        
        // Second stage - Low pass at 500 Hz
        float wc_lp = tanf(w_high / 2.0f);
        float k1_lp = sqrtf(2.0f) * wc_lp;
        float k2_lp = wc_lp * wc_lp;
        float a0_lp = k2_lp + k1_lp + 1.0f;
        
        coeffs[5] = k2_lp / a0_lp;                   // b0
        coeffs[6] = 2.0f * k2_lp / a0_lp;            // b1
        coeffs[7] = k2_lp / a0_lp;                   // b2
        coeffs[8] = (2.0f * (k2_lp - 1.0f)) / a0_lp; // -a1
        coeffs[9] = (k2_lp - k1_lp + 1.0f) / a0_lp;  // -a2
        
        // Initialize filter structure (2-stage cascade)
        arm_biquad_cascade_df1_init_f32(
            &_filters[ch],
            2,                    // 2 biquad stages
            coeffs,               // Filter coefficients
            states                // State variables
        );
        
        // Clear state variables
        memset(states, 0, 8 * sizeof(float));
    }
}

void EMGProcessor::sampleISR() {
    // High-speed sampling function called from timer interrupt at 2kHz
    if (!_initialized) return;
    
    // Read all EMG channels rapidly
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        // Fast ADC read
        uint16_t rawADC = analogRead(EMG_PINS[ch]);
        float rawSample = (float)rawADC;
        
        // Store raw sample
        _channels[ch].raw = rawSample;
        
        // Store in history buffer
        uint16_t histIdx = (_sampleIndex + ch) % EMG_BUFFER_SIZE;
        _channels[ch].rawHistory[histIdx] = rawSample;
        
        // Apply real-time DSP pipeline
        processSampleRealtime(ch, rawSample);
    }
    
    // Increment global sample index
    _sampleIndex++;
    
    // Update feature calculation counter
    _featureUpdateCounter++;
    
    // Calculate features every 40 samples (50Hz rate for 2kHz sampling)
    if (_featureUpdateCounter >= FEATURE_UPDATE_INTERVAL) {
        _featureUpdateCounter = 0;
        // Set flag for main loop to calculate features
        _featuresNeedUpdate = true;
    }
}

void EMGProcessor::processSampleRealtime(uint8_t ch, float sample) {
    // 1. Apply bandpass filter (20-500 Hz) using CMSIS-DSP
    float filtered;
    arm_biquad_cascade_df1_f32(
        &_filters[ch],
        &sample,        // Input sample
        &filtered,      // Output sample
        1               // Process 1 sample
    );
    _channels[ch].filtered = filtered;
    
    // 2. Apply full-wave rectification
    float rectified = fabsf(filtered);
    _channels[ch].rectified = rectified;
    
    // 3. Update sliding window for RMS calculation (100ms window)
    uint16_t bufIdx = _channels[ch].bufferIndex;
    
    // Remove oldest sample from window sum
    _channels[ch].windowSum -= _channels[ch].rectifiedBuffer[bufIdx];
    
    // Add new rectified sample to buffer and sum
    _channels[ch].rectifiedBuffer[bufIdx] = rectified;
    _channels[ch].windowSum += rectified;
    
    // Advance buffer index (circular)
    _channels[ch].bufferIndex = (bufIdx + 1) % EMG_BUFFER_SIZE;
    
    // 4. Calculate RMS from window (mean square root)
    float meanSquare = _channels[ch].windowSum / EMG_BUFFER_SIZE;
    _channels[ch].rms = sqrtf(meanSquare);
    
    // 5. Apply normalization with offset compensation
    float normalized = (_channels[ch].rms - _channels[ch].offset);
    normalized = normalized / EMG_NORM_SCALE; // Scale factor for normalization
    _channels[ch].norm = constrain(normalized, 0.0f, 1.0f);
    
    // Store last sample for derivative calculations
    _channels[ch].lastSample = sample;
}

void EMGProcessor::update() {
    // Called from main loop to perform non-time-critical processing
    if (!_initialized) return;
    
    // Update features if needed (set by ISR)
    if (_featuresNeedUpdate) {
        _featuresNeedUpdate = false;
        
        for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
            extractAdvancedFeatures(ch);
        }
        
        // Update public feature array
        updatePublicFeatureArray();
    }
}

void EMGProcessor::extractAdvancedFeatures(uint8_t ch) {
    // Extract advanced time-domain features from the signal history
    
    // 1. Zero Crossing (ZC) - number of times signal crosses zero
    uint16_t zc_count = 0;
    const float zc_threshold = EMG_ZC_THRESHOLD;
    
    // Use a subset of history buffer for feature calculation
    uint16_t featureWindowSize = min(EMG_FEATURE_WINDOW, EMG_BUFFER_SIZE);
    uint16_t startIdx = (_sampleIndex >= featureWindowSize) ? 
                        (_sampleIndex - featureWindowSize) % EMG_BUFFER_SIZE : 0;
    
    for (uint16_t i = 1; i < featureWindowSize; i++) {
        uint16_t curr_idx = (startIdx + i) % EMG_BUFFER_SIZE;
        uint16_t prev_idx = (startIdx + i - 1) % EMG_BUFFER_SIZE;
        
        float prev = _channels[ch].rawHistory[prev_idx] - _channels[ch].offset;
        float curr = _channels[ch].rawHistory[curr_idx] - _channels[ch].offset;
        
        // Check for zero crossing with threshold
        if ((prev > zc_threshold && curr < -zc_threshold) || 
            (prev < -zc_threshold && curr > zc_threshold)) {
            zc_count++;
        }
    }
    _channels[ch].zc = (float)zc_count;
    
    // 2. Slope Sign Changes (SSC) - number of times slope changes direction
    uint16_t ssc_count = 0;
    const float ssc_threshold = EMG_SSC_THRESHOLD;
    
    for (uint16_t i = 1; i < featureWindowSize - 1; i++) {
        uint16_t prev_idx = (startIdx + i - 1) % EMG_BUFFER_SIZE;
        uint16_t curr_idx = (startIdx + i) % EMG_BUFFER_SIZE;
        uint16_t next_idx = (startIdx + i + 1) % EMG_BUFFER_SIZE;
        
        float prev = _channels[ch].rawHistory[prev_idx];
        float curr = _channels[ch].rawHistory[curr_idx];
        float next = _channels[ch].rawHistory[next_idx];
        
        float slope1 = curr - prev;
        float slope2 = next - curr;
        
        // Check for slope sign change with threshold
        if ((slope1 > ssc_threshold && slope2 < -ssc_threshold) || 
            (slope1 < -ssc_threshold && slope2 > ssc_threshold)) {
            ssc_count++;
        }
    }
    _channels[ch].ssc = (float)ssc_count;
    
    // 3. Waveform Length (WL) - cumulative length of waveform
    float wl = 0.0f;
    
    for (uint16_t i = 1; i < featureWindowSize; i++) {
        uint16_t curr_idx = (startIdx + i) % EMG_BUFFER_SIZE;
        uint16_t prev_idx = (startIdx + i - 1) % EMG_BUFFER_SIZE;
        
        float diff = _channels[ch].rawHistory[curr_idx] - _channels[ch].rawHistory[prev_idx];
        wl += fabsf(diff);
    }
    _channels[ch].wl = wl;
}

void EMGProcessor::updatePublicFeatureArray() {
    // Update the public emgFeatures array for other modules
    // Format: [ch0_rms, ch0_zc, ch0_ssc, ch0_wl, ch1_rms, ...]
    
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        uint8_t baseIdx = ch * EMG_FEATURES_PER_CHANNEL;
        
        // Normalize features for consistent scaling
        emgFeatures[baseIdx + 0] = _channels[ch].norm;                    // Normalized RMS
        emgFeatures[baseIdx + 1] = _channels[ch].zc / EMG_ZC_NORM_FACTOR; // Normalized ZC
        emgFeatures[baseIdx + 2] = _channels[ch].ssc / EMG_SSC_NORM_FACTOR; // Normalized SSC
        emgFeatures[baseIdx + 3] = _channels[ch].wl / EMG_WL_NORM_FACTOR; // Normalized WL
        
        // Clamp all features to [0, 1] range
        for (uint8_t f = 0; f < EMG_FEATURES_PER_CHANNEL; f++) {
            emgFeatures[baseIdx + f] = constrain(emgFeatures[baseIdx + f], 0.0f, 1.0f);
        }
    }
}

float EMGProcessor::getFeature(uint8_t ch) const {
    // Return normalized RMS amplitude for compatibility
    if (ch >= NUM_EMG_CHANNELS) return 0.0f;
    return _channels[ch].norm;
}

void EMGProcessor::getFeatureVector(float* out) const {
    // Fill output array with normalized RMS values for all channels
    if (!out) return;
    
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        out[ch] = _channels[ch].norm;
    }
}

void EMGProcessor::setChannelOffset(uint8_t ch, float offset) {
    if (ch < NUM_EMG_CHANNELS) {
        _channels[ch].offset = offset;
    }
}

float EMGProcessor::getChannelOffset(uint8_t ch) const {
    if (ch >= NUM_EMG_CHANNELS) return 0.0f;
    return _channels[ch].offset;
}

void EMGProcessor::calibrateRest() {
    if (!_initialized) return;
    
    Serial.println(F("Calibrating EMG rest levels..."));
    Serial.println(F("Keep muscles relaxed for 3 seconds"));
    
    // Collect samples for baseline calculation
    const uint16_t CALIB_SAMPLES = 1000; // 0.5 seconds at 2kHz
    float sums[NUM_EMG_CHANNELS] = {0};
    
    // Collect samples
    for (uint16_t sample = 0; sample < CALIB_SAMPLES; sample++) {
        for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
            sums[ch] += analogRead(EMG_PINS[ch]);
        }
        delayMicroseconds(500); // 2kHz sampling rate
    }
    
    // Calculate and store offsets
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        _channels[ch].offset = sums[ch] / CALIB_SAMPLES;
        
        Serial.print(F("Channel "));
        Serial.print(ch);
        Serial.print(F(" offset: "));
        Serial.println(_channels[ch].offset, 2);
    }
    
    Serial.println(F("EMG calibration complete"));
}

bool EMGProcessor::isChannelActive(uint8_t ch) const {
    if (ch >= NUM_EMG_CHANNELS) return false;
    return (_channels[ch].norm > EMG_ACTIVITY_THRESHOLD);
}

float EMGProcessor::getRMS(uint8_t ch) const {
    if (ch >= NUM_EMG_CHANNELS) return 0.0f;
    return _channels[ch].rms;
}

float EMGProcessor::getZC(uint8_t ch) const {
    if (ch >= NUM_EMG_CHANNELS) return 0.0f;
    return _channels[ch].zc;
}

float EMGProcessor::getSSC(uint8_t ch) const {
    if (ch >= NUM_EMG_CHANNELS) return 0.0f;
    return _channels[ch].ssc;
}

float EMGProcessor::getWL(uint8_t ch) const {
    if (ch >= NUM_EMG_CHANNELS) return 0.0f;
    return _channels[ch].wl;
}

void EMGProcessor::getExtendedFeatures(uint8_t ch, EMGFeatures* features) const {
    if (ch >= NUM_EMG_CHANNELS || !features) return;
    
    features->rms = _channels[ch].norm;
    features->zc = _channels[ch].zc;
    features->ssc = _channels[ch].ssc;
    features->wl = _channels[ch].wl;
    features->isActive = isChannelActive(ch);
}

void EMGProcessor::printChannelStatus(uint8_t ch) const {
    if (ch >= NUM_EMG_CHANNELS) return;
    
    Serial.print(F("Ch"));
    Serial.print(ch);
    Serial.print(F(": RMS="));
    Serial.print(_channels[ch].norm, 3);
    Serial.print(F(" ZC="));
    Serial.print(_channels[ch].zc, 1);
    Serial.print(F(" SSC="));
    Serial.print(_channels[ch].ssc, 1);
    Serial.print(F(" WL="));
    Serial.print(_channels[ch].wl, 1);
    Serial.print(F(" Active="));
    Serial.println(isChannelActive(ch) ? "Yes" : "No");
}

void EMGProcessor::printAllChannels() const {
    Serial.println(F("EMG Channel Status:"));
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        printChannelStatus(ch);
    }
}