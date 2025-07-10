/**
 * @file config.h
 * @brief System configuration and pin definitions
 * @version 2.0
 * @author Haix02
 * @date 2025-07-10
 */

#pragma once

#include <Arduino.h>

// ================================
// SYSTEM CONFIGURATION
// ================================

#define NUM_FINGERS 4
#define NUM_EMG_CHANNELS 8

// ================================
// PIN DEFINITIONS
// ================================

// Finger PWM control pins (servos/motors)
const uint8_t FINGER_PWM_PINS[NUM_FINGERS] = {
    2,  // Finger 0 (Thumb)
    3,  // Finger 1 (Index)
    4,  // Finger 2 (Middle)
    5   // Finger 3 (Ring/Pinky)
};

// Finger FSR sensor pins
const uint8_t FINGER_FSR_PINS[NUM_FINGERS] = {
    A0, // Finger 0 FSR
    A1, // Finger 1 FSR
    A2, // Finger 2 FSR
    A3  // Finger 3 FSR
};

// EMG input pins
const uint8_t EMG_PINS[NUM_EMG_CHANNELS] = {
    A8,  // EMG Channel 0
    A9,  // EMG Channel 1
    A10, // EMG Channel 2
    A11, // EMG Channel 3
    A12, // EMG Channel 4
    A13, // EMG Channel 5
    A14, // EMG Channel 6
    A15  // EMG Channel 7
};

// ================================
// FINGER POSITION LIMITS
// ================================

#define FINGER_POS_MIN 0    // Fully open (degrees)
#define FINGER_POS_MAX 180  // Fully closed (degrees)

// ================================
// EMG PROCESSING PARAMETERS
// ================================

#define EMG_SAMPLE_HZ 2000          // EMG sampling frequency
#define EMG_BUFFER_SIZE 200         // 100ms window at 2kHz
#define EMG_FEATURES_PER_CHANNEL 4  // RMS, ZC, SSC, WL
#define EMG_NORM_SCALE 2048.0f      // 12-bit ADC normalization
#define EMG_ACTIVITY_THRESHOLD 0.05f // Activity detection threshold

// Feature calculation thresholds
#define EMG_ZC_THRESHOLD 15.0f      // Zero crossing threshold
#define EMG_SSC_THRESHOLD 10.0f     // Slope sign change threshold

// ================================
// REFLEX ENGINE PARAMETERS
// ================================

#define REFLEX_SLIP_THRESHOLD -0.2f  // Slip detection threshold (negative)
#define REFLEX_MIN_FORCE 0.05f       // Minimum force for slip detection

// ================================
// COMMAND INTERFACE
// ================================

#define CMD_BUFFER_SIZE 64           // Serial command buffer size

// ================================
// TIMING PARAMETERS
// ================================

#define MAIN_LOOP_HZ 100            // Main loop frequency
#define FINGER_UPDATE_HZ 100        // Finger update frequency
#define GRASP_UPDATE_HZ 50          // Grasp manager update frequency