/**
 * @file Calibration.cpp
 * @brief System calibration implementation with EEPROM storage
 */

#include "Calibration.h"

Calibration::Calibration()
    : _emgProcessor(nullptr), _fingers(nullptr)
{
    memset(&_calibData, 0, sizeof(_calibData));
    _calibData.magicNumber = MAGIC_NUMBER;
    _calibData.slipThreshold = FSR_SLIP_DFDT_THRESH;
}

void Calibration::begin(EMGProcessor* emgProcessor, Finger** fingers) {
    _emgProcessor = emgProcessor;
    _fingers = fingers;
}

bool Calibration::loadFromEEPROM() {
    // Read data from EEPROM
    CalibrationData storedData;
    EEPROM.get(0, storedData);
    
    // Verify magic number
    if (storedData.magicNumber != MAGIC_NUMBER) {
        Serial.println(F("No valid calibration found in EEPROM"));
        resetToDefaults();
        return false;
    }
    
    // Verify checksum
    uint8_t calculatedChecksum = calculateChecksum(storedData);
    if (calculatedChecksum != storedData.checksum) {
        Serial.println(F("Calibration checksum error"));
        resetToDefaults();
        return false;
    }
    
    // Data valid, copy to working set
    memcpy(&_calibData, &storedData, sizeof(CalibrationData));
    
    // Apply calibration to components
    applyCalibration();
    
    Serial.println(F("Calibration loaded from EEPROM"));
    return true;
}

void Calibration::saveToEEPROM() {
    // Calculate checksum before saving
    _calibData.checksum = calculateChecksum(_calibData);
    
    // Write to EEPROM
    EEPROM.put(0, _calibData);
    
    Serial.println(F("Calibration saved to EEPROM"));
}

void Calibration::resetToDefaults() {
    // Set magic number
    _calibData.magicNumber = MAGIC_NUMBER;
    
    // Reset EMG thresholds and offsets
    for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
        _calibData.emgThresholds[i] = EMG_THRESH;
        _calibData.emgOffsets[i] = 0.0f;
    }
    
    // Reset finger offsets
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
        _calibData.fingerOffsets[i] = 0;
    }
    
    // Reset slip threshold
    _calibData.slipThreshold = FSR_SLIP_DFDT_THRESH;
    
    // Apply defaults
    applyCalibration();
    
    Serial.println(F("Reset to default calibration"));
}

void Calibration::calibrateEMGRest() {
    if (!_emgProcessor) return;
    
    // Calibrate EMG resting levels
    _emgProcessor->calibrateRest();
    
    // Store calibrated offsets
    for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
        // Get offset from EMG processor (this would need a getter method)
        // For now, we'll update our records but not retrieve actual values
        // _calibData.emgOffsets[i] = _emgProcessor->getChannelOffset(i);
    }
    
    Serial.println(F("EMG rest levels calibrated"));
}

void Calibration::setEMGThreshold(uint8_t channel, float threshold) {
    if (channel >= NUM_EMG_CHANNELS) return;
    
    _calibData.emgThresholds[channel] = threshold;
    
    // Apply to EMG processor if available
    if (_emgProcessor) {
        // EMG processor would need a method to set threshold
        // For now, just storing it in our calibration data
    }
}

void Calibration::setEMGOffset(uint8_t channel, float offset) {
    if (channel >= NUM_EMG_CHANNELS) return;
    
    _calibData.emgOffsets[channel] = offset;
    
    // Apply to EMG processor
    if (_emgProcessor) {
        _emgProcessor->setChannelOffset(channel, offset);
    }
}

void Calibration::setFingerOffset(uint8_t finger, int16_t offset) {
    if (finger >= NUM_FINGERS || !_fingers) return;
    
    _calibData.fingerOffsets[finger] = offset;
    
    // Apply to finger
    _fingers[finger]->setOffset(offset);
}

void Calibration::setSlipThreshold(float threshold) {
    _calibData.slipThreshold = threshold;
}

void Calibration::applyCalibration() {
    // Apply EMG calibration
    if (_emgProcessor) {
        for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
            _emgProcessor->setChannelOffset(i, _calibData.emgOffsets[i]);
            // Would also set thresholds if method available
        }
    }
    
    // Apply finger offsets
    if (_fingers) {
        for (uint8_t i = 0; i < NUM_FINGERS; i++) {
            _fingers[i]->setOffset(_calibData.fingerOffsets[i]);
        }
    }
    
    // Note: slip threshold would be applied to ReflexEngine
    // but we don't have that reference in this class
}

void Calibration::printCalibration() {
    Serial.println(F("Current Calibration:"));
    
    Serial.println(F("EMG Offsets:"));
    for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
        Serial.print(F("  Ch"));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.println(_calibData.emgOffsets[i]);
    }
    
    Serial.println(F("EMG Thresholds:"));
    for (uint8_t i = 0; i < NUM_EMG_CHANNELS; i++) {
        Serial.print(F("  Ch"));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.println(_calibData.emgThresholds[i], 3);
    }
    
    Serial.println(F("Finger Offsets:"));
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
        Serial.print(F("  Finger"));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.println(_calibData.fingerOffsets[i]);
    }
    
    Serial.print(F("Slip Threshold: "));
    Serial.println(_calibData.slipThreshold, 3);
}

uint8_t Calibration::calculateChecksum(const CalibrationData& data) {
    // Simple checksum calculation
    const uint8_t* bytes = (const uint8_t*)&data;
    uint8_t sum = 0;
    
    // Skip the checksum field itself
    for (size_t i = 0; i < sizeof(CalibrationData) - sizeof(uint8_t); i++) {
        sum += bytes[i];
    }
    
    return sum;
}