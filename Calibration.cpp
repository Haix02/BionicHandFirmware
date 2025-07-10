/**
 * @file Calibration.cpp
 * @brief EEPROM-based calibration storage for bionic hand system
 * @version 2.0
 * @author Haix02
 * @date 2025-07-10 23:13:16 UTC
 */

#include "Calibration.h"
#include <EEPROM.h>

Calibration::Calibration()
    : _emgProcessor(nullptr),
      _graspManager(nullptr),
      _fingers(nullptr),
      _numFingers(0),
      _initialized(false)
{
    // Initialize calibration data with defaults
    resetCalibrationData();
}

void Calibration::begin(EMGProcessor* emgProcessor, 
                       GraspManager* graspManager,
                       Finger** fingers, 
                       uint8_t numFingers)
{
    _emgProcessor = emgProcessor;
    _graspManager = graspManager;
    _fingers = fingers;
    _numFingers = min(numFingers, (uint8_t)MAX_FINGERS);
    _initialized = true;
    
    Serial.println(F("Calibration module initialized"));
}

bool Calibration::loadCalibration() {
    if (!_initialized) {
        Serial.println(F("ERROR: Calibration not initialized"));
        return false;
    }
    
    Serial.println(F("Loading calibration from EEPROM..."));
    
    // Read calibration data from EEPROM
    CalibrationData storedData;
    EEPROM.get(EEPROM_START_ADDRESS, storedData);
    
    // Validate magic number
    if (storedData.magicNumber != CALIBRATION_MAGIC) {
        Serial.println(F("No valid calibration found - using defaults"));
        resetCalibration();
        return false;
    }
    
    // Validate version
    if (storedData.version != CALIBRATION_VERSION) {
        Serial.print(F("Calibration version mismatch (found v"));
        Serial.print(storedData.version);
        Serial.print(F(", expected v"));
        Serial.print(CALIBRATION_VERSION);
        Serial.println(F(") - resetting to defaults"));
        resetCalibration();
        return false;
    }
    
    // Verify checksum
    uint16_t calculatedChecksum = calculateChecksum(storedData);
    if (calculatedChecksum != storedData.checksum) {
        Serial.println(F("Calibration checksum error - data corrupted"));
        resetCalibration();
        return false;
    }
    
    // Data is valid - copy to working memory
    memcpy(&_calibData, &storedData, sizeof(CalibrationData));
    
    // Apply calibration to system components
    applyCalibration();
    
    Serial.println(F("Calibration loaded successfully"));
    printCalibration();
    
    return true;
}

bool Calibration::saveCalibration() {
    if (!_initialized) {
        Serial.println(F("ERROR: Calibration not initialized"));
        return false;
    }
    
    Serial.println(F("Saving calibration to EEPROM..."));
    
    // Update timestamp
    _calibData.timestamp = millis();
    
    // Calculate and store checksum
    _calibData.checksum = calculateChecksum(_calibData);
    
    // Read current EEPROM data to check if write is necessary
    CalibrationData currentData;
    EEPROM.get(EEPROM_START_ADDRESS, currentData);
    
    // Only write if data has changed (preserve EEPROM life)
    if (memcmp(&currentData, &_calibData, sizeof(CalibrationData)) != 0) {
        EEPROM.put(EEPROM_START_ADDRESS, _calibData);
        Serial.println(F("Calibration saved to EEPROM"));
    } else {
        Serial.println(F("Calibration unchanged - EEPROM write skipped"));
    }
    
    return true;
}

void Calibration::resetCalibration() {
    Serial.println(F("Resetting calibration to defaults..."));
    
    // Reset to default values
    resetCalibrationData();
    
    // Apply defaults to system
    applyCalibration();
    
    // Save defaults to EEPROM
    saveCalibration();
    
    Serial.println(F("Calibration reset complete"));
}

void Calibration::calibrateEMG() {
    if (!_emgProcessor) {
        Serial.println(F("ERROR: EMG processor not available"));
        return;
    }
    
    Serial.println(F("Starting EMG calibration..."));
    Serial.println(F("Please relax all muscles for 5 seconds"));
    
    // Wait for user preparation
    for (int i = 5; i > 0; i--) {
        Serial.print(i);
        Serial.println(F("..."));
        delay(1000);
    }
    
    Serial.println(F("Calibrating resting levels..."));
    
    // Perform EMG calibration
    _emgProcessor->calibrateRest();
    
    // Store new offsets in calibration data
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        _calibData.emgOffsets[ch] = _emgProcessor->getChannelOffset(ch);
    }
    
    // Set activity thresholds based on resting levels
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        _calibData.emgThresholds[ch] = EMG_DEFAULT_THRESHOLD;
    }
    
    Serial.println(F("EMG calibration complete"));
    
    // Automatically save after calibration
    saveCalibration();
}

void Calibration::setEMGThreshold(uint8_t channel, float threshold) {
    if (channel >= NUM_EMG_CHANNELS) {
        Serial.println(F("ERROR: Invalid EMG channel"));
        return;
    }
    
    _calibData.emgThresholds[channel] = constrain(threshold, 0.01f, 1.0f);
    
    Serial.print(F("EMG channel "));
    Serial.print(channel);
    Serial.print(F(" threshold set to "));
    Serial.println(threshold, 4);
}

void Calibration::setEMGOffset(uint8_t channel, float offset) {
    if (channel >= NUM_EMG_CHANNELS) {
        Serial.println(F("ERROR: Invalid EMG channel"));
        return;
    }
    
    _calibData.emgOffsets[channel] = offset;
    
    // Apply to EMG processor if available
    if (_emgProcessor) {
        _emgProcessor->setChannelOffset(channel, offset);
    }
    
    Serial.print(F("EMG channel "));
    Serial.print(channel);
    Serial.print(F(" offset set to "));
    Serial.println(offset, 2);
}

void Calibration::setFingerOffset(uint8_t finger, int16_t offset) {
    if (finger >= _numFingers) {
        Serial.println(F("ERROR: Invalid finger index"));
        return;
    }
    
    _calibData.fingerOffsets[finger] = constrain(offset, -50, 50);
    
    // Apply to finger if available
    if (_fingers && _fingers[finger]) {
        _fingers[finger]->setOffset(offset);
    }
    
    Serial.print(F("Finger "));
    Serial.print(finger);
    Serial.print(F(" offset set to "));
    Serial.println(offset);
}

void Calibration::setGraspForceMultiplier(float multiplier) {
    _calibData.graspForceMultiplier = constrain(multiplier, 0.1f, 2.0f);
    
    // Apply to grasp manager if available
    if (_graspManager) {
        _graspManager->setGraspForce(_calibData.graspForceMultiplier);
    }
    
    Serial.print(F("Grasp force multiplier set to "));
    Serial.println(multiplier, 2);
}

void Calibration::setReflexSensitivity(float sensitivity) {
    _calibData.reflexSensitivity = constrain(sensitivity, 0.1f, 2.0f);
    
    Serial.print(F("Reflex sensitivity set to "));
    Serial.println(sensitivity, 2);
}

float Calibration::getEMGThreshold(uint8_t channel) const {
    if (channel >= NUM_EMG_CHANNELS) return EMG_DEFAULT_THRESHOLD;
    return _calibData.emgThresholds[channel];
}

float Calibration::getEMGOffset(uint8_t channel) const {
    if (channel >= NUM_EMG_CHANNELS) return 0.0f;
    return _calibData.emgOffsets[channel];
}

int16_t Calibration::getFingerOffset(uint8_t finger) const {
    if (finger >= MAX_FINGERS) return 0;
    return _calibData.fingerOffsets[finger];
}

float Calibration::getGraspForceMultiplier() const {
    return _calibData.graspForceMultiplier;
}

float Calibration::getReflexSensitivity() const {
    return _calibData.reflexSensitivity;
}

void Calibration::printCalibration() const {
    Serial.println(F("\n===== CURRENT CALIBRATION ====="));
    
    // Header info
    Serial.print(F("Version: "));
    Serial.println(_calibData.version);
    Serial.print(F("Timestamp: "));
    Serial.print(_calibData.timestamp / 1000);
    Serial.println(F(" seconds"));
    
    // EMG calibration
    Serial.println(F("\nEMG Channels:"));
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        Serial.print(F("  Ch"));
        Serial.print(ch);
        Serial.print(F(": Offset="));
        Serial.print(_calibData.emgOffsets[ch], 2);
        Serial.print(F(", Threshold="));
        Serial.println(_calibData.emgThresholds[ch], 4);
    }
    
    // Finger calibration
    Serial.println(F("\nFingers:"));
    for (uint8_t f = 0; f < _numFingers; f++) {
        Serial.print(F("  Finger"));
        Serial.print(f);
        Serial.print(F(": Offset="));
        Serial.println(_calibData.fingerOffsets[f]);
    }
    
    // System parameters
    Serial.println(F("\nSystem Parameters:"));
    Serial.print(F("  Grasp Force Multiplier: "));
    Serial.println(_calibData.graspForceMultiplier, 2);
    Serial.print(F("  Reflex Sensitivity: "));
    Serial.println(_calibData.reflexSensitivity, 2);
    
    Serial.println(F("===============================\n"));
}

void Calibration::resetCalibrationData() {
    // Set magic number and version
    _calibData.magicNumber = CALIBRATION_MAGIC;
    _calibData.version = CALIBRATION_VERSION;
    _calibData.timestamp = millis();
    
    // Reset EMG calibration to defaults
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        _calibData.emgOffsets[ch] = EMG_DEFAULT_OFFSET;
        _calibData.emgThresholds[ch] = EMG_DEFAULT_THRESHOLD;
    }
    
    // Reset finger offsets to zero
    for (uint8_t f = 0; f < MAX_FINGERS; f++) {
        _calibData.fingerOffsets[f] = 0;
    }
    
    // Reset system parameters to defaults
    _calibData.graspForceMultiplier = 1.0f;
    _calibData.reflexSensitivity = 1.0f;
    
    // Clear reserved space
    memset(_calibData.reserved, 0, sizeof(_calibData.reserved));
    
    // Calculate checksum
    _calibData.checksum = calculateChecksum(_calibData);
}

void Calibration::applyCalibration() {
    Serial.println(F("Applying calibration to system..."));
    
    // Apply EMG calibration
    if (_emgProcessor) {
        for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
            _emgProcessor->setChannelOffset(ch, _calibData.emgOffsets[ch]);
        }
        Serial.println(F("  EMG calibration applied"));
    }
    
    // Apply finger offsets
    if (_fingers) {
        for (uint8_t f = 0; f < _numFingers; f++) {
            if (_fingers[f]) {
                _fingers[f]->setOffset(_calibData.fingerOffsets[f]);
            }
        }
        Serial.println(F("  Finger offsets applied"));
    }
    
    // Apply grasp parameters
    if (_graspManager) {
        _graspManager->setGraspForce(_calibData.graspForceMultiplier);
        Serial.println(F("  Grasp parameters applied"));
    }
    
    Serial.println(F("Calibration application complete"));
}

uint16_t Calibration::calculateChecksum(const CalibrationData& data) const {
    // Simple 16-bit checksum calculation
    const uint8_t* bytes = (const uint8_t*)&data;
    uint16_t checksum = 0;
    
    // Calculate checksum over all data except the checksum field itself
    size_t dataSize = sizeof(CalibrationData) - sizeof(data.checksum);
    
    for (size_t i = 0; i < dataSize; i++) {
        checksum += bytes[i];
        checksum = (checksum << 1) | (checksum >> 15); // Rotate left
    }
    
    return checksum;
}

bool Calibration::isCalibrationValid() const {
    return (_calibData.magicNumber == CALIBRATION_MAGIC &&
            _calibData.version == CALIBRATION_VERSION &&
            calculateChecksum(_calibData) == _calibData.checksum);
}

void Calibration::exportCalibration() const {
    Serial.println(F("\n===== CALIBRATION EXPORT ====="));
    Serial.println(F("Copy the following data for backup:"));
    Serial.println(F("Format: [Magic][Ver][EMG_Offsets][EMG_Thresholds][Finger_Offsets][Params]"));
    
    Serial.print(F("CAL_DATA:"));
    Serial.print(_calibData.magicNumber, HEX);
    Serial.print(F(","));
    Serial.print(_calibData.version);
    
    // EMG offsets
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        Serial.print(F(","));
        Serial.print(_calibData.emgOffsets[ch], 4);
    }
    
    // EMG thresholds
    for (uint8_t ch = 0; ch < NUM_EMG_CHANNELS; ch++) {
        Serial.print(F(","));
        Serial.print(_calibData.emgThresholds[ch], 4);
    }
    
    // Finger offsets
    for (uint8_t f = 0; f < MAX_FINGERS; f++) {
        Serial.print(F(","));
        Serial.print(_calibData.fingerOffsets[f]);
    }
    
    // System parameters
    Serial.print(F(","));
    Serial.print(_calibData.graspForceMultiplier, 4);
    Serial.print(F(","));
    Serial.print(_calibData.reflexSensitivity, 4);
    
    Serial.println();
    Serial.println(F("==============================\n"));
}

void Calibration::factoryReset() {
    Serial.println(F("FACTORY RESET: Erasing all calibration data..."));
    
    // Clear EEPROM area
    for (int addr = EEPROM_START_ADDRESS; addr < EEPROM_START_ADDRESS + sizeof(CalibrationData); addr++) {
        EEPROM.write(addr, 0xFF);
    }
    
    // Reset to defaults
    resetCalibration();
    
    Serial.println(F("Factory reset complete"));
}

bool Calibration::performSelfTest() const {
    Serial.println(F("Performing calibration self-test..."));
    
    bool testPassed = true;
    
    // Test EEPROM read/write
    uint8_t testValue = 0xAA;
    uint8_t testAddress = EEPROM_START_ADDRESS + sizeof(CalibrationData) + 10;
    
    EEPROM.write(testAddress, testValue);
    uint8_t readValue = EEPROM.read(testAddress);
    
    if (readValue != testValue) {
        Serial.println(F("  EEPROM test: FAILED"));
        testPassed = false;
    } else {
        Serial.println(F("  EEPROM test: PASSED"));
    }
    
    // Test checksum calculation
    CalibrationData testData = _calibData;
    uint16_t checksum1 = calculateChecksum(testData);
    uint16_t checksum2 = calculateChecksum(testData);
    
    if (checksum1 != checksum2) {
        Serial.println(F("  Checksum test: FAILED"));
        testPassed = false;
    } else {
        Serial.println(F("  Checksum test: PASSED"));
    }
    
    // Test data validation
    if (!isCalibrationValid()) {
        Serial.println(F("  Data validation: FAILED"));
        testPassed = false;
    } else {
        Serial.println(F("  Data validation: PASSED"));
    }
    
    Serial.print(F("Self-test result: "));
    Serial.println(testPassed ? "PASSED" : "FAILED");
    
    return testPassed;
}