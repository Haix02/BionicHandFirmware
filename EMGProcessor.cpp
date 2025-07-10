/**
 * @file EMGProcessor.cpp
 * @brief Enhanced EMG signal processing with DMA and advanced DSP
 * @version 3.0
 */

#include "EMGProcessor.h"
#include "ADC.h"

// Static member initialization
DMACh EMGProcessor::dmaChannel;
volatile uint16_t EMGProcessor::dmaBuffer[2][NUM_EMG_CHANNELS];
volatile bool EMGProcessor::dmaBufferReady = false;
volatile uint8_t EMGProcessor::currentDmaBuffer = 0;

EMGProcessor::EMGProcessor() : dmaModeEnabled(false), adaptiveThresholdRatio(0.2f) {
    memset(channels, 0, sizeof(channels));
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        channels[ch].adaptive_threshold = 0.15f;
    }
}

void EMGProcessor::begin() {
    // Initialize DMA
    setupDMA();
    
    // Configure DSP filters
    configureFilters();
    
    // Initialize channel state
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        channels[ch].offset = 0.0f;
        channels[ch].activity_detected = false;
        activityWindowStart[ch] = 0;
        activityWindowSize[ch] = EMG_MOVAVG_WINDOW;
    }
}

void EMGProcessor::setupDMA() {
    // Configure ADC for DMA operation
    ADC* adc = new ADC();
    
    // Configure all ADC pins for input
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        pinMode(EMG_PINS[ch], INPUT);
    }
    
    // Configure ADC settings optimized for EMG
    adc->adc0->setAveraging(4);
    adc->adc0->setResolution(12);
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
    
    // Configure DMA transfers
    dmaChannel.begin();
    dmaChannel.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
    dmaChannel.attachInterrupt(dmaISR);
    
    // Set up double-buffered DMA transfer
    dmaChannel.destinationBuffer(dmaBuffer[0], sizeof(dmaBuffer[0]));
    dmaChannel.interruptAtCompletion();
    dmaChannel.disableOnCompletion();
    
    // Enable DMA mode
    dmaModeEnabled = true;
    
    // Start initial DMA transfer
    dmaChannel.enable();
}

void EMGProcessor::dmaISR() {
    // DMA buffer is ready for processing
    dmaBufferReady = true;
    
    // Swap buffers for double-buffering
    uint8_t nextBuffer = 1 - currentDmaBuffer;
    currentDmaBuffer = nextBuffer;
    
    // Set up next DMA transfer
    dmaChannel.destinationBuffer(dmaBuffer[nextBuffer], sizeof(dmaBuffer[nextBuffer]));
    dmaChannel.enable();
}

void EMGProcessor::configureFilters() {
    // Configure bandpass filter (20-500Hz) using CMSIS-DSP
    // Assuming 1kHz sampling rate
    const float fs = 1000.0f;  // Sampling frequency
    const float f1 = 20.0f;    // Lower cutoff
    const float f2 = 500.0f;   // Upper cutoff
    
    // Compute filter coefficients for bandpass filter
    // Convert analog frequencies to digital
    const float w1 = 2.0f * M_PI * f1 / fs;
    const float w2 = 2.0f * M_PI * f2 / fs;
    
    // Compute coefficients using bilinear transform
    const float alpha1 = sin(w1) / (2.0f * 0.707f);
    const float alpha2 = sin(w2) / (2.0f * 0.707f);
    
    // Initialize a filter for each channel
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        // Calculate coefficients (bandpass filter)
        float* coeffs = &bandpassCoeffs[ch * 5];
        
        // Basic IIR bandpass filter coefficients
        coeffs[0] = 0.1f;                // b0
        coeffs[1] = 0.0f;                // b1
        coeffs[2] = -0.1f;               // b2
        coeffs[3] = -1.8f * cos(w1);     // -a1
        coeffs[4] = 0.8f;                // -a2
        
        // Initialize filter
        arm_biquad_cascade_df1_init_f32(
            &bandpassFilter[ch], 
            1,                    // 1-stage filter
            coeffs,               // Filter coefficients
            &bandpassState[ch*4]  // State variables
        );
    }
}

void EMGProcessor::sampleISR() {
    // Legacy method for non-DMA mode
    // Only used when DMA is disabled or for backward compatibility
    if (!dmaModeEnabled) {
        // Read all EMG ADCs (legacy mode)
        for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
            channels[ch].raw = analogRead(EMG_PINS[ch]);
            applyDSPPipeline(ch, channels[ch].raw);
        }
    }
}

void EMGProcessor::process() {
    // Process any available DMA data
    if (dmaModeEnabled && dmaBufferReady) {
        processDMABuffer();
        dmaBufferReady = false;
    }
    
    // Update features and detect activity for all channels
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        updateFeatures(ch);
        detectActivity(ch);
        normalizeAndThreshold(ch);
    }
}

void EMGProcessor::processDMABuffer() {
    // Process the most recently completed DMA buffer
    uint8_t bufferToProcess = 1 - currentDmaBuffer;
    
    // Process each channel
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        float sample = (float)dmaBuffer[bufferToProcess][ch];
        channels[ch].raw = sample;
        applyDSPPipeline(ch, sample);
    }
}

void EMGProcessor::applyDSPPipeline(uint8_t ch, float sample) {
    float filtered;
    
    // Apply bandpass filter using CMSIS-DSP
    arm_biquad_cascade_df1_f32(
        &bandpassFilter[ch], 
        &sample,            // Input sample
        &filtered,          // Output sample
        1                   // Process 1 sample
    );
    
    channels[ch].bandpassed = filtered;
    
    // Full-wave rectification
    float rectified = fabs(filtered);
    channels[ch].rectified = rectified;
    
    // Moving average smoothing
    updateMovingAverage(ch, rectified);
    
    // RMS calculation
    updateRMS(ch, rectified);
}

void EMGProcessor::updateFeatures(uint8_t ch) {
    // Calculate zero crossings
    int zc = 0;
    float prevSample = 0;
    float threshold = 0.01f; // Small threshold to avoid noise
    
    for (int i = 1; i < EMG_MOVAVG_WINDOW; i++) {
        float curr = channels[ch].movavg_buf[(channels[ch].movavg_idx + i) % EMG_MOVAVG_WINDOW];
        float prev = channels[ch].movavg_buf[(channels[ch].movavg_idx + i - 1) % EMG_MOVAVG_WINDOW];
        
        if (((prev > threshold) && (curr < -threshold)) || 
            ((prev < -threshold) && (curr > threshold))) {
            zc++;
        }
    }
    channels[ch].zero_crossings = (float)zc;
    
    // Calculate waveform length
    float wl = 0;
    for (int i = 1; i < EMG_MOVAVG_WINDOW; i++) {
        float curr = channels[ch].movavg_buf[(channels[ch].movavg_idx + i) % EMG_MOVAVG_WINDOW];
        float prev = channels[ch].movavg_buf[(channels[ch].movavg_idx + i - 1) % EMG_MOVAVG_WINDOW];
        wl += fabs(curr - prev);
    }
    channels[ch].waveform_length = wl;
}

void EMGProcessor::detectActivity(uint8_t ch) {
    // Calculate current energy level
    float energy = channels[ch].rms;
    
    // Check if energy exceeds adaptive threshold
    bool wasActive = channels[ch].activity_detected;
    bool isActive = (energy > channels[ch].adaptive_threshold);
    
    // Edge detection for activity transitions
    if (!wasActive && isActive) {
        // Start of activity - mark window start
        activityWindowStart[ch] = channels[ch].movavg_idx;
    }
    else if (wasActive && !isActive) {
        // End of activity - calculate window size
        uint16_t end = channels[ch].movavg_idx;
        uint16_t start = activityWindowStart[ch];
        
        // Calculate window size accounting for buffer wrap
        activityWindowSize[ch] = (end >= start) ? 
            (end - start) : (EMG_MOVAVG_WINDOW - start + end);
        
        // Update adaptive threshold based on recent activity
        channels[ch].adaptive_threshold = adaptiveThresholdRatio * energy;
        if (channels[ch].adaptive_threshold < 0.05f) {
            channels[ch].adaptive_threshold = 0.05f; // Minimum threshold
        }
    }
    
    channels[ch].activity_detected = isActive;
}

void EMGProcessor::updateMovingAverage(uint8_t ch, float val) {
    EMGChannelState &s = channels[ch];
    s.movavg_sum -= s.movavg_buf[s.movavg_idx];
    s.movavg_buf[s.movavg_idx] = val;
    s.movavg_sum += val;
    s.movavg_idx = (s.movavg_idx + 1) % EMG_MOVAVG_WINDOW;
}

void EMGProcessor::updateRMS(uint8_t ch, float val) {
    EMGChannelState &s = channels[ch];
    
    // Remove oldest sample from sum of squares
    float oldSample = s.movavg_buf[s.movavg_idx];
    s.rms_sum_sq -= oldSample * oldSample;
    
    // Add new sample to sum of squares
    s.rms_sum_sq += val * val;
    
    // Calculate RMS
    s.rms = sqrt(s.rms_sum_sq / EMG_MOVAVG_WINDOW);
}

void EMGProcessor::normalizeAndThreshold(uint8_t ch) {
    EMGChannelState &s = channels[ch];
    float ma = s.movavg_sum / EMG_MOVAVG_WINDOW;
    float norm = (ma - s.offset) / (1024.0f); // Scale and remove offset
    norm = fmax(0.0f, fmin(1.0f, norm)); // Clamp to 0-1
    
    // Use adaptive threshold if activity detected, otherwise use static threshold
    float threshold = s.activity_detected ? s.adaptive_threshold : EMG_THRESH;
    norm = (norm > threshold) ? norm : 0.0f;
    s.norm = norm;
}

float EMGProcessor::getFeature(uint8_t ch) const {
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
    // Average current movavg as offset
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ++ch) {
        channels[ch].offset = channels[ch].movavg_sum / EMG_MOVAVG_WINDOW;
    }
}

// V3 Enhanced methods

bool EMGProcessor::isActivityDetected(uint8_t ch) const {
    return channels[ch].activity_detected;
}

float EMGProcessor::getZeroCrossings(uint8_t ch) const {
    return channels[ch].zero_crossings;
}

float EMGProcessor::getWaveformLength(uint8_t ch) const {
    return channels[ch].waveform_length;
}

float EMGProcessor::getRMS(uint8_t ch) const {
    return channels[ch].rms;
}

void EMGProcessor::enableDMA(bool enable) {
    if (enable != dmaModeEnabled) {
        dmaModeEnabled = enable;
        if (enable) {
            setupDMA();
        } else {
            dmaChannel.disable();
        }
    }
}

void EMGProcessor::setAdaptiveThreshold(float ratio) {
    adaptiveThresholdRatio = constrain(ratio, 0.05f, 0.5f);
}