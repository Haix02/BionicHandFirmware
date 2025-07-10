/**
 * @file Calibration.cpp
 * @brief Implementation of user calibration and personalization
 * @version 3.0
 */

#include "Calibration.h"

Calibration::Calibration(EMGProcessor* emgProcessor, Finger** fingers)
    : _emgProcessor(emgProcessor), _fingers(fingers)
{
    // Initialize with defaults
    restoreDefaults();
}

bool Calibration::loadFromEEPROM() {
    // Read calibration data from EEPROM
    CalibrationData storedData;
    EEPROM.get(EEPROM_ADDR, storedData);
    
    // Check if data is valid (magic number check)
    if (storedData.magic != CALIBRATION_MAGIC) {
        Serial.println(F("No valid calibration found in EEPROM"));
        return false;
    }
    
    // Valid data found, copy to current settings
    _data = storedData;
    
    // Apply loaded calibration to components
    applyCalibration();
    
    Serial.println(F("Calibration loaded from EEPROM"));
    return true;
}

void Calibration::saveToEEPROM() {
    // Ensure magic number is set
    _data.magic = CALIBRATION_MAGIC;
    
    // Write data to EEPROM
    EEPROM.put(EEPROM_ADDR, _data);
    
    Serial.println(F("Calibration saved to EEPROM"));
}

void Calibration::restoreDefaults() {
    // Set magic number
    _data.magic = CALIBRATION_MAGIC;
    
    // EMG defaults
    for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
        _data.emgThresholds[i] = EMG_THRESH;
        _data.emgOffsets[i] = 0.0f;
    }
    
    // Finger defaults
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
        _data.fingerOffsets[i] = 0;
        _data.fsrScaling[i] = 1.0f;
    }
    
    // Slip detection default
    _data.slipThreshold = FSR_SLIP_DFDT_THRESH;
    
    // Apply defaults to components
    applyCalibration();
    
    Serial.println(F("Default calibration restored"));
}

void Calibration::setEMGThreshold(uint8_t channel, float threshold) {
    if (channel < NUM_EMG_CHANNELS) {
        _data.emgThresholds[channel] = threshold;
        
        // Apply to EMG processor if available
        if (_emgProcessor) {
            // In V3, we would use the adaptive threshold setting
            // This is a placeholder as the actual method might differ
        }
    }
}

void Calibration::setEMGOffset(uint8_t channel, float offset) {
    if (channel < NUM_EMG_CHANNELS && _emgProcessor) {
        _data.emgOffsets[channel] = offset;
        _emgProcessor->setChannelOffset(channel, offset);
    }
}

void Calibration::setFingerOffset(uint8_t finger, int16_t offset) {
    if (finger < NUM_FINGERS && _fingers) {
        _data.fingerOffsets[finger] = offset;
        _fingers[finger]->setOffset(offset);
    }
}

void Calibration::setFSRScaling(uint8_t finger, float scaling) {
    if (finger < NUM_FINGERS) {
        _data.fsrScaling[finger] = scaling;
        // Apply FSR scaling if the finger class supports it
    }
}

void Calibration::setSlipThreshold(float threshold) {
    _data.slipThreshold = threshold;
}

const CalibrationData& Calibration::getData() const {
    return _data;
}

void Calibration::applyCalibration() {
    // Apply EMG calibration
    if (_emgProcessor) {
        for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
            _emgProcessor->setChannelOffset(i, _data.emgOffsets[i]);
            // Set thresholds if applicable
        }
    }
    
    // Apply finger offsets
    if (_fingers) {
        for (uint8_t i = 0; i < NUM_FINGERS; i++) {
            _fingers[i]->setOffset(_data.fingerOffsets[i]);
            // Apply FSR scaling if supported
        }
    }
    
    // Apply slip threshold to ReflexEngine if accessible
    // This would require a ReflexEngine pointer, which we don't have in this class
}

uint16_t Calibration::calculateChecksum(const CalibrationData& data) {
    // Simple checksum implementation for data validation
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&data);
    uint16_t checksum = 0;
    
    // Skip the magic number in the checksum calculation
    for (size_t i = sizeof(data.magic); i < sizeof(CalibrationData); i++) {
        checksum += bytes[i];
    }
    
    return checksum;
}