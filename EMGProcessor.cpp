/**
 * @file EMGProcessor.cpp
 * @brief Real-time EMG signal processing with advanced DSP features
 * @version 2.0
 * @author Haix02
 * @date 2025-07-10 23:29:22 UTC
 */

#include "EMGProcessor.h"
#include <arm_math.h>

// Constructor
EMGProcessor::EMGProcessor()
    : _sampleCount(0),
      _featuresReady(false),
      _initialized(false),
      _lastFeatureUpdate(0),
      _featureUpdateInterval(20)  // Update features every 20ms (50Hz)
{
    // Initialize all channel data structures
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        _channels[ch].raw = 0.0f;
        _channels[ch].filtered = 0.0f;
        _channels[ch].rectified = 0.0f;
        _channels[ch].offset = 512.0f;  // Default 12-bit ADC midpoint
        
        // RMS initialization
        _channels[ch].rmsIndex = 0;
        _channels[ch].rmsSum = 0.0f;
        _channels[ch].rms = 0.0f;
        _channels[ch].rmsNormalized = 0.0f;
        
        // Feature extraction initialization
        _channels[ch].historyIndex = 0;
        _channels[ch].historyFull = false;
        _channels[ch].zc = 0.0f;
        _channels[ch].ssc = 0.0f;
        _channels[ch].wl = 0.0f;
        _channels[ch].active = false;
        
        // Initialize RMS buffer
        for (uint16_t i = 0; i < EMG_BUFFER_SIZE; i++) {
            _channels[ch].rmsBuffer[i] = 0.0f;
        }
        
        // Initialize raw history buffer
        for (uint16_t i = 0; i < EMG_FEATURE_WINDOW; i++) {
            _channels[ch].rawHistory[i] = 0.0f;
        }
        
        // Initialize filter state
        for (uint8_t i = 0; i < 4; i++) {
            _channels[ch].filterState[i] = 0.0f;
        }
    }
    
    // Initialize public feature array
    for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
        emgFeatures[i] = 0.0f;
    }
    
    // Configure DSP filters
    configureFilters();
}

void EMGProcessor::begin() {
    Serial.println(F("Initializing EMG processor..."));
    
    // Configure ADC pins for EMG input
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        pinMode(EMG_PINS[ch], INPUT);
    }
    
    // Set ADC parameters for optimal EMG sampling
    analogReadResolution(12);    // 12-bit resolution (0-4095)
    analogReadAveraging(1);      // No averaging for real-time processing
    
    // Brief settling time for hardware
    delay(100);
    
    // Perform initial calibration
    calibrateRest();
    
    _initialized = true;
    Serial.println(F("EMG processor ready"));
}

void EMGProcessor::configureFilters() {
    // Design second-order Butterworth bandpass filter (20-500 Hz at 2kHz sampling)
    const float fs = EMG_SAMPLE_RATE;           // 2000 Hz
    const float f_low = EMG_LOW_CUTOFF;         // 20 Hz
    const float f_high = EMG_HIGH_CUTOFF;       // 500 Hz
    
    // Calculate normalized frequencies
    const float w_low = 2.0f * PI * f_low / fs;   // 0.0628 rad/sample
    const float w_high = 2.0f * PI * f_high / fs; // 1.5708 rad/sample
    
    // Calculate center frequency and bandwidth
    const float w0 = sqrtf(w_low * w_high);       // Center frequency
    const float bw = w_high - w_low;              // Bandwidth
    
    // Calculate Q factor
    const float Q = w0 / bw;
    
    // Pre-warp frequencies for bilinear transform
    const float w0_warped = tanf(w0 / 2.0f);
    const float bw_warped = tanf(bw / 2.0f);
    
    // Calculate analog prototype coefficients
    const float k = w0_warped / Q;
    const float norm = 1.0f + k + w0_warped * w0_warped;
    
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        float* coeffs = _channels[ch].filterCoeffs;
        
        // Calculate digital filter coefficients using bilinear transform
        // Bandpass filter: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
        
        // Numerator coefficients (zeros)
        coeffs[0] = k / norm;           // b0
        coeffs[1] = 0.0f;               // b1
        coeffs[2] = -k / norm;          // b2
        
        // Denominator coefficients (poles) - stored as negative for CMSIS-DSP
        coeffs[3] = (2.0f * (w0_warped * w0_warped - 1.0f)) / norm;  // -a1
        coeffs[4] = (1.0f - k + w0_warped * w0_warped) / norm;       // -a2
        
        // Initialize CMSIS-DSP biquad filter structure
        arm_biquad_cascade_df1_init_f32(
            &_channels[ch].filter,      // Filter instance
            1,                          // Number of biquad stages (1 for second-order)
            coeffs,                     // Filter coefficients [b0, b1, b2, -a1, -a2]
            _channels[ch].filterState   // State variables [x1, x2, y1, y2]
        );
    }
}

void EMGProcessor::sampleISR() {
    // High-speed sampling function called at 2kHz from timer interrupt
    if (!_initialized) return;
    
    // Sample all EMG channels rapidly
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        // Read raw ADC value (12-bit: 0-4095)
        uint16_t adcValue = analogRead(EMG_PINS[ch]);
        float rawSample = (float)adcValue;
        
        // Store raw sample
        _channels[ch].raw = rawSample;
        
        // Process sample through DSP pipeline
        processSample(ch, rawSample);
        
        // Store in history buffer for feature extraction
        _channels[ch].rawHistory[_channels[ch].historyIndex] = rawSample;
        _channels[ch].historyIndex = (_channels[ch].historyIndex + 1) % EMG_FEATURE_WINDOW;
        
        // Mark history buffer as full after first complete cycle
        if (_channels[ch].historyIndex == 0) {
            _channels[ch].historyFull = true;
        }
    }
    
    // Increment global sample counter
    _sampleCount++;
    
    // Set flag for feature processing every 40 samples (50Hz at 2kHz)
    if (_sampleCount % 40 == 0) {
        _featuresReady = true;
    }
}

void EMGProcessor::processSample(uint8_t ch, float sample) {
    // 1. Remove DC offset
    float offsetRemoved = sample - _channels[ch].offset;
    
    // 2. Apply bandpass filter (20-500 Hz) using CMSIS-DSP
    float filtered;
    arm_biquad_cascade_df1_f32(
        &_channels[ch].filter,      // Filter instance
        &offsetRemoved,             // Input sample
        &filtered,                  // Output sample
        1                           // Number of samples to process
    );
    _channels[ch].filtered = filtered;
    
    // 3. Apply full-wave rectification
    float rectified = fabsf(filtered);
    _channels[ch].rectified = rectified;
    
    // 4. Update RMS calculation using circular buffer
    updateRMS(ch, rectified);
    
    // 5. Normalize RMS value to 0-1 range
    _channels[ch].rmsNormalized = _channels[ch].rms / EMG_RMS_SCALE;
    _channels[ch].rmsNormalized = constrain(_channels[ch].rmsNormalized, 0.0f, 1.0f);
    
    // 6. Update activity detection
    _channels[ch].active = (_channels[ch].rmsNormalized > EMG_ACTIVITY_THRESHOLD);
}

void EMGProcessor::updateRMS(uint8_t ch, float rectified) {
    // Get current buffer index
    uint16_t idx = _channels[ch].rmsIndex;
    
    // Remove oldest value from running sum
    _channels[ch].rmsSum -= _channels[ch].rmsBuffer[idx];
    
    // Add new rectified value to buffer and sum
    _channels[ch].rmsBuffer[idx] = rectified;
    _channels[ch].rmsSum += rectified;
    
    // Advance buffer index (circular buffer)
    _channels[ch].rmsIndex = (idx + 1) % EMG_BUFFER_SIZE;
    
    // Calculate RMS: sqrt(mean(x^2))
    // Note: We're using rectified values, so this is actually mean absolute value
    // For true RMS, we would square before storing, but rectified mean works well for EMG
    float meanSquare = _channels[ch].rmsSum / EMG_BUFFER_SIZE;
    _channels[ch].rms = sqrtf(meanSquare);
}

void EMGProcessor::update() {
    // Non-time-critical feature processing called from main loop
    if (!_initialized) return;
    
    uint32_t currentTime = millis();
    
    // Process features at controlled rate (50Hz) or when ready flag is set
    if (_featuresReady || (currentTime - _lastFeatureUpdate >= _featureUpdateInterval)) {
        _featuresReady = false;
        _lastFeatureUpdate = currentTime;
        
        // Extract advanced features for all channels
        for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
            if (_channels[ch].historyFull) {
                extractFeatures(ch);
                normalizeFeatures(ch);
            }
        }
        
        // Update public feature array
        updatePublicArray();
    }
}

void EMGProcessor::extractFeatures(uint8_t ch) {
    // Calculate Zero Crossings (ZC)
    _channels[ch].zc = calculateZeroCrossings(ch);
    
    // Calculate Slope Sign Changes (SSC)
    _channels[ch].ssc = calculateSlopeSignChanges(ch);
    
    // Calculate Waveform Length (WL)
    _channels[ch].wl = calculateWaveformLength(ch);
}

float EMGProcessor::calculateZeroCrossings(uint8_t ch) {
    uint16_t crossings = 0;
    uint16_t startIdx = _channels[ch].historyIndex;
    
    // Iterate through history buffer to count zero crossings
    for (uint16_t i = 1; i < EMG_FEATURE_WINDOW; i++) {
        uint16_t prevIdx = (startIdx + i - 1) % EMG_FEATURE_WINDOW;
        uint16_t currIdx = (startIdx + i) % EMG_FEATURE_WINDOW;
        
        // Get offset-corrected values
        float prev = _channels[ch].rawHistory[prevIdx] - _channels[ch].offset;
        float curr = _channels[ch].rawHistory[currIdx] - _channels[ch].offset;
        
        // Check for zero crossing with threshold to avoid noise
        if ((prev > EMG_ZC_THRESHOLD && curr < -EMG_ZC_THRESHOLD) ||
            (prev < -EMG_ZC_THRESHOLD && curr > EMG_ZC_THRESHOLD)) {
            crossings++;
        }
    }
    
    return (float)crossings;
}

float EMGProcessor::calculateSlopeSignChanges(uint8_t ch) {
    uint16_t changes = 0;
    uint16_t startIdx = _channels[ch].historyIndex;
    
    // Iterate through history buffer to count slope sign changes
    for (uint16_t i = 1; i < EMG_FEATURE_WINDOW - 1; i++) {
        uint16_t prevIdx = (startIdx + i - 1) % EMG_FEATURE_WINDOW;
        uint16_t currIdx = (startIdx + i) % EMG_FEATURE_WINDOW;
        uint16_t nextIdx = (startIdx + i + 1) % EMG_FEATURE_WINDOW;
        
        float prev = _channels[ch].rawHistory[prevIdx];
        float curr = _channels[ch].rawHistory[currIdx];
        float next = _channels[ch].rawHistory[nextIdx];
        
        // Calculate slopes
        float slope1 = curr - prev;
        float slope2 = next - curr;
        
        // Check for slope sign change with threshold
        if ((slope1 > EMG_SSC_THRESHOLD && slope2 < -EMG_SSC_THRESHOLD) ||
            (slope1 < -EMG_SSC_THRESHOLD && slope2 > EMG_SSC_THRESHOLD)) {
            changes++;
        }
    }
    
    return (float)changes;
}

float EMGProcessor::calculateWaveformLength(uint8_t ch) {
    float length = 0.0f;
    uint16_t startIdx = _channels[ch].historyIndex;
    
    // Sum absolute differences between consecutive samples
    for (uint16_t i = 1; i < EMG_FEATURE_WINDOW; i++) {
        uint16_t prevIdx = (startIdx + i - 1) % EMG_FEATURE_WINDOW;
        uint16_t currIdx = (startIdx + i) % EMG_FEATURE_WINDOW;
        
        float diff = _channels[ch].rawHistory[currIdx] - _channels[ch].rawHistory[prevIdx];
        length += fabsf(diff);
    }
    
    return length;
}

void EMGProcessor::normalizeFeatures(uint8_t ch) {
    // Normalize features to 0-1 range using predefined scale factors
    _channels[ch].zc = constrain(_channels[ch].zc / EMG_ZC_SCALE, 0.0f, 1.0f);
    _channels[ch].ssc = constrain(_channels[ch].ssc / EMG_SSC_SCALE, 0.0f, 1.0f);
    _channels[ch].wl = constrain(_channels[ch].wl / EMG_WL_SCALE, 0.0f, 1.0f);
}

void EMGProcessor::updatePublicArray() {
    // Update public emgFeatures array with primary feature (normalized RMS)
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        emgFeatures[ch] = _channels[ch].rmsNormalized;
    }
}

// Accessor functions implementation

float EMGProcessor::getFeature(uint8_t ch) const {
    if (ch >= NUM_EMG_CHANNELS) return 0.0f;
    return _channels[ch].rmsNormalized;
}

void EMGProcessor::getFeatureVector(float* out) const {
    if (!out) return;
    
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        out[ch] = _channels[ch].rmsNormalized;
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

bool EMGProcessor::isChannelActive(uint8_t ch) const {
    if (ch >= NUM_EMG_CHANNELS) return false;
    return _channels[ch].active;
}

float EMGProcessor::getRMS(uint8_t ch) const {
    if (ch >= NUM_EMG_CHANNELS) return 0.0f;
    return _channels[ch].rmsNormalized;
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

void EMGProcessor::calibrateRest() {
    Serial.println(F("Calibrating EMG baseline levels..."));
    Serial.println(F("Keep all muscles relaxed for 3 seconds"));
    
    // Calibration parameters
    const uint16_t CALIB_SAMPLES = 2000;  // 1 second at 2kHz
    float sums[NUM_EMG_CHANNELS] = {0.0f};
    
    // Wait for user preparation
    delay(1000);
    Serial.println(F("Calibrating... stay relaxed!"));
    
    // Collect baseline samples
    for (uint16_t sample = 0; sample < CALIB_SAMPLES; sample++) {
        for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
            sums[ch] += (float)analogRead(EMG_PINS[ch]);
        }
        delayMicroseconds(500);  // 2kHz sampling rate
    }
    
    // Calculate and store DC offsets
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        _channels[ch].offset = sums[ch] / CALIB_SAMPLES;
        
        Serial.print(F("Ch"));
        Serial.print(ch);
        Serial.print(F(": "));
        Serial.print(_channels[ch].offset, 1);
        Serial.println(F(" ADC units"));
    }
    
    Serial.println(F("EMG calibration complete"));
}

void EMGProcessor::printChannelStatus(uint8_t ch) const {
    if (ch >= NUM_EMG_CHANNELS) return;
    
    Serial.print(F("Ch"));
    Serial.print(ch);
    Serial.print(F(": RMS="));
    Serial.print(_channels[ch].rmsNormalized, 4);
    Serial.print(F(" ZC="));
    Serial.print(_channels[ch].zc, 3);
    Serial.print(F(" SSC="));
    Serial.print(_channels[ch].ssc, 3);
    Serial.print(F(" WL="));
    Serial.print(_channels[ch].wl, 3);
    Serial.print(F(" Raw="));
    Serial.print(_channels[ch].raw, 0);
    Serial.print(F(" Active="));
    Serial.println(_channels[ch].active ? "YES" : "no");
}