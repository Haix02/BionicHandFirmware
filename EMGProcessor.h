/**
 * @file EMGProcessor.h
 * @brief Real-time EMG signal processing with advanced DSP features
 * @version 2.0
 * @author Haix02
 * @date 2025-07-10 23:30:51 UTC
 */

#pragma once

#include <Arduino.h>
#include "config.h"
#include <arm_math.h> // CMSIS-DSP library for Teensy 4.1

// DSP Configuration Parameters
#define EMG_SAMPLE_RATE 2000        // Sampling frequency (Hz)
#define EMG_BUFFER_SIZE 200         // 100ms window at 2kHz for RMS
#define EMG_FEATURES_PER_CHANNEL 4  // RMS, ZC, SSC, WL
#define EMG_FEATURE_WINDOW 200      // Feature calculation window size

// Filter parameters
#define EMG_FILTER_ORDER 2          // Second-order biquad filter
#define EMG_LOW_CUTOFF 20.0f        // Low cutoff frequency (Hz)
#define EMG_HIGH_CUTOFF 500.0f      // High cutoff frequency (Hz)

// Feature extraction thresholds
#define EMG_ZC_THRESHOLD 15.0f      // Zero crossing detection threshold (ADC units)
#define EMG_SSC_THRESHOLD 10.0f     // Slope sign change threshold (ADC units)
#define EMG_ACTIVITY_THRESHOLD 0.05f // Channel activity detection threshold (normalized)

// Normalization scale factors
#define EMG_RMS_SCALE 2048.0f       // RMS normalization scale (12-bit ADC range)
#define EMG_ZC_SCALE 50.0f          // Zero crossing normalization factor
#define EMG_SSC_SCALE 30.0f         // Slope sign change normalization factor
#define EMG_WL_SCALE 5000.0f        // Waveform length normalization factor

/**
 * @brief EMG signal processor with real-time DSP capabilities
 * 
 * This class provides comprehensive EMG signal processing including:
 * - Real-time bandpass filtering (20-500 Hz)
 * - Full-wave rectification and RMS calculation
 * - Advanced feature extraction (ZC, SSC, WL)
 * - Multi-channel processing for up to 8 EMG inputs
 */
class EMGProcessor {
public:
    /**
     * @brief Constructor - initializes all data structures
     */
    EMGProcessor();
    
    /**
     * @brief Initialize EMG processor and configure hardware
     */
    void begin();
    
    /**
     * @brief High-speed sampling ISR (called at 2kHz from timer interrupt)
     * 
     * This function performs time-critical signal acquisition and basic processing:
     * - Reads all 8 EMG channels
     * - Applies bandpass filtering
     * - Performs rectification and RMS updates
     * - Maintains signal history buffers
     */
    void sampleISR();
    
    /**
     * @brief Feature processing update (called from main loop at ~50Hz)
     * 
     * Performs computationally intensive feature extraction:
     * - Zero crossing detection
     * - Slope sign change analysis
     * - Waveform length calculation
     * - Feature normalization
     */
    void update();
    
    /**
     * @brief Get primary normalized feature for a channel
     * @param ch Channel index (0-7)
     * @return Normalized RMS amplitude (0.0-1.0)
     */
    float getFeature(uint8_t ch) const;
    
    /**
     * @brief Fill array with primary features for all channels
     * @param out Output array (must be NUM_EMG_CHANNELS in size)
     */
    void getFeatureVector(float* out) const;
    
    /**
     * @brief Set DC offset for a channel (for calibration)
     * @param ch Channel index (0-7)
     * @param offset DC offset value in ADC units
     */
    void setChannelOffset(uint8_t ch, float offset);
    
    /**
     * @brief Get current DC offset for a channel
     * @param ch Channel index (0-7)
     * @return Current offset value in ADC units
     */
    float getChannelOffset(uint8_t ch) const;
    
    /**
     * @brief Perform automatic baseline calibration
     * 
     * Records resting EMG levels and sets appropriate DC offsets.
     * User should remain relaxed during calibration.
     */
    void calibrateRest();
    
    /**
     * @brief Check if channel shows muscle activity
     * @param ch Channel index (0-7)
     * @return True if RMS amplitude exceeds activity threshold
     */
    bool isChannelActive(uint8_t ch) const;
    
    /**
     * @brief Get individual feature values
     */
    float getRMS(uint8_t ch) const;        // Normalized RMS amplitude
    float getZC(uint8_t ch) const;         // Normalized zero crossings
    float getSSC(uint8_t ch) const;        // Normalized slope sign changes
    float getWL(uint8_t ch) const;         // Normalized waveform length
    
    /**
     * @brief Print comprehensive channel status for debugging
     * @param ch Channel index (0-7)
     */
    void printChannelStatus(uint8_t ch) const;
    
    /**
     * @brief Public feature array for external module access
     * 
     * Contains primary feature (normalized RMS) for each channel.
     * Updated automatically by the processor at ~50Hz.
     * Format: emgFeatures[channel] = normalized_rms_amplitude
     */
    float emgFeatures[NUM_EMG_CHANNELS];

private:
    /**
     * @brief Per-channel data structure containing all signal processing state
     */
    struct EMGChannel {
        // Real-time signal processing chain
        float raw;                          // Current raw ADC sample
        float filtered;                     // Bandpass filtered signal
        float rectified;                    // Full-wave rectified signal
        float offset;                       // DC offset (baseline) in ADC units
        
        // RMS calculation using circular buffer
        float rmsBuffer[EMG_BUFFER_SIZE];   // Circular buffer for 100ms window
        uint16_t rmsIndex;                  // Current buffer write position
        float rmsSum;                       // Running sum for efficient RMS calc
        float rms;                          // Current RMS value
        float rmsNormalized;                // Normalized RMS (0.0-1.0)
        
        // Feature extraction buffers
        float rawHistory[EMG_FEATURE_WINDOW];   // Raw signal history for features
        uint16_t historyIndex;                  // History buffer write position
        bool historyFull;                       // Flag indicating buffer is full
        
        // Extracted time-domain features
        float zc;                           // Zero crossings count (normalized)
        float ssc;                          // Slope sign changes count (normalized)
        float wl;                           // Waveform length (normalized)
        
        // CMSIS-DSP filter structures
        arm_biquad_casd_df1_inst_f32 filter;   // Biquad filter instance
        float filterCoeffs[5];                  // Filter coefficients [b0,b1,b2,-a1,-a2]
        float filterState[4];                   // Filter state variables [x1,x2,y1,y2]
        
        // Channel state
        bool active;                        // Activity detection flag
    };
    
    // Channel data array (8 channels)
    EMGChannel _channels[NUM_EMG_CHANNELS];
    
    // Processing control variables
    volatile uint32_t _sampleCount;         // Total samples processed
    volatile bool _featuresReady;           // Flag for feature processing
    bool _initialized;                      // Initialization complete flag
    
    // Timing control
    uint32_t _lastFeatureUpdate;            // Last feature update timestamp
    uint32_t _featureUpdateInterval;        // Feature update interval (ms)
    
    // Private processing methods
    
    /**
     * @brief Configure CMSIS-DSP biquad filters for all channels
     * 
     * Calculates second-order Butterworth bandpass filter coefficients
     * for 20-500 Hz passband at 2kHz sampling rate.
     */
    void configureFilters();
    
    /**
     * @brief Process single sample through DSP pipeline
     * @param ch Channel index
     * @param sample Raw ADC sample value
     */
    void processSample(uint8_t ch, float sample);
    
    /**
     * @brief Update RMS calculation with new rectified sample
     * @param ch Channel index
     * @param rectified Rectified sample value
     */
    void updateRMS(uint8_t ch, float rectified);
    
    /**
     * @brief Extract advanced features from signal history
     * @param ch Channel index
     */
    void extractFeatures(uint8_t ch);
    
    /**
     * @brief Normalize extracted features to 0-1 range
     * @param ch Channel index
     */
    void normalizeFeatures(uint8_t ch);
    
    /**
     * @brief Update public emgFeatures array with latest data
     */
    void updatePublicArray();
    
    // Feature calculation utilities
    
    /**
     * @brief Calculate zero crossings in signal history
     * @param ch Channel index
     * @return Number of zero crossings detected
     */
    float calculateZeroCrossings(uint8_t ch);
    
    /**
     * @brief Calculate slope sign changes in signal history
     * @param ch Channel index
     * @return Number of slope direction changes detected
     */
    float calculateSlopeSignChanges(uint8_t ch);
    
    /**
     * @brief Calculate waveform length (sum of absolute differences)
     * @param ch Channel index
     * @return Total waveform length
     */
    float calculateWaveformLength(uint8_t ch);
};