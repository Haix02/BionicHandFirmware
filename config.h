/**
 * @file config.h
 * @brief Hardware pin assignments and system configuration constants for Teensy 4.1 Bionic Hand.
 * @author Ottobock/Open Bionics Firmware Team
 * @version 3.0
 */

#pragma once

#include <Arduino.h>

// ----------------------------
// ---- Pin Assignments -------
// ----------------------------

constexpr uint8_t NUM_EMG_CHANNELS = 8;
constexpr uint8_t NUM_FINGERS = 4;
constexpr uint8_t NUM_FSR = NUM_FINGERS;

// Assign analog pins for EMG inputs
constexpr uint8_t EMG_PINS[NUM_EMG_CHANNELS] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Assign analog pins for Finger FSR sensors
constexpr uint8_t FSR_PINS[NUM_FSR] = {A8, A9, A10, A11};

// Optional IMU pins (I2C)
constexpr uint8_t IMU_SDA_PIN = 18; // Default I2C pins for Teensy 4.1
constexpr uint8_t IMU_SCL_PIN = 19;

// Assign actuator PWM pins (Servo/Dynamixel/ESC. Adapt as required)
constexpr uint8_t FINGER_PWM_PINS[NUM_FINGERS] = {2, 3, 4, 5};

// Optional: LED heartbeat
constexpr uint8_t LED_PIN = 13;

// ----------------------------
// ---- Signal Processing -----
// ----------------------------

// EMG Sampling
constexpr float EMG_SAMPLE_HZ = 1000.0f;
constexpr uint16_t EMG_MOVAVG_WINDOW_MS = 100;
constexpr uint16_t EMG_MOVAVG_WINDOW = (uint16_t)((EMG_SAMPLE_HZ * EMG_MOVAVG_WINDOW_MS) / 1000.0f);

// FSR Sampling
constexpr float FSR_SAMPLE_HZ = 100.0f;

// Filtering - IIR Biquad Bandpass (20-500Hz)
constexpr float EMG_BANDPASS_FC_LOW = 20.0f;
constexpr float EMG_BANDPASS_FC_HIGH = 500.0f;
constexpr float EMG_BANDPASS_Q = 0.707f;

// Normalization & Threshold
constexpr float EMG_NORM_MAX = 1.0f; // Max normalized value
constexpr float EMG_THRESH = 0.15f;  // Activation threshold (tune per subject)

// FSR slip detection
constexpr float FSR_SLIP_DFDT_THRESH = -0.2f; // Negative derivative threshold for slip (tune as needed)
constexpr uint16_t FSR_DERIV_WINDOW = 4;      // Sample window (ms) for dF/dt

// PID (example values, tune as needed)
constexpr float FINGER_PID_KP = 2.0f;
constexpr float FINGER_PID_KI = 0.01f;
constexpr float FINGER_PID_KD = 0.1f;

// Motion Profiles
constexpr float MAX_VELOCITY_DEG_PER_SEC = 400.0f;
constexpr float MAX_ACCELERATION_DEG_PER_SEC2 = 800.0f;

// Reflex parameters
constexpr float REFLEX_RESPONSE_TARGET_MS = 5.0f;  // Target response time in ms

// ----------------------------
// ---- Serial Interface ------
// ----------------------------

constexpr unsigned long SERIAL_BAUD = 115200;

// ----------------------------
// ---- Safety Limits ---------
// ----------------------------

constexpr uint16_t FINGER_POS_MIN = 0;   // PWM/Angle min
constexpr uint16_t FINGER_POS_MAX = 180; // PWM/Angle max