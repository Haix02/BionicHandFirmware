/**
 * @file SensorFusion.cpp
 * @brief Enhanced sensor fusion with IMU integration and slip detection
 * @version 2.0
 */

#include "SensorFusion.h"

SensorFusion::SensorFusion()
    : _imuPresent(false),
      _fsrCount(0),
      _emgCount(0),
      _slipRisk(0.0f),
      _forceMultiplier(1.0f),
      _lastUpdateTime(0)
{
    // Initialize arrays
    memset(_fsrValues, 0, sizeof(_fsrValues));
    memset(_fsrLastValues, 0, sizeof(_fsrLastValues));
    memset(_fsrDerivatives, 0, sizeof(_fsrDerivatives));
    memset(_fsrFiltered, 0, sizeof(_fsrFiltered));
    
    memset(_emgValues, 0, sizeof(_emgValues));
    
    memset(_accel, 0, sizeof(_accel));
    memset(_gyro, 0, sizeof(_gyro));
    memset(_orientation, 0, sizeof(_orientation));
}

bool SensorFusion::begin() {
    // Initialize I2C communication
    Wire.begin();
    
    // Try to initialize MPU6050
    _imuPresent = initIMU();
    
    if (_imuPresent) {
        Serial.println(F("IMU initialized successfully"));
    } else {
        Serial.println(F("IMU not detected"));
    }
    
    return true;
}

bool SensorFusion::initIMU() {
    // Check if IMU is present on I2C bus
    Wire.beginTransmission(MPU6050_ADDR);
    bool devicePresent = (Wire.endTransmission() == 0);
    
    if (!devicePresent) {
        return false;
    }
    
    // Initialize MPU6050
    // Wake up the device
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // Set to zero to wake up
    Wire.endTransmission(true);
    
    // Configure gyro range to ±250 deg/s
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);  // GYRO_CONFIG register
    Wire.write(0);     // 0x00 = 250 deg/s
    Wire.endTransmission(true);
    
    // Configure accel range to ±2g
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C);  // ACCEL_CONFIG register
    Wire.write(0);     // 0x00 = 2g
    Wire.endTransmission(true);
    
    // Configure digital low pass filter
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1A);  // CONFIG register
    Wire.write(0x03);  // 0x03 = 44Hz DLPF
    Wire.endTransmission(true);
    
    return true;
}

void SensorFusion::update() {
    uint32_t now = millis();
    float dt = (now - _lastUpdateTime) / 1000.0f; // seconds
    
    // Avoid division by zero on first call
    if (_lastUpdateTime == 0) {
        dt = 0.01f; // 10ms default
    }
    
    _lastUpdateTime = now;
    
    // Update FSR derivatives for slip detection
    updateFsrDerivatives(dt);
    
    // Update IMU data if available
    if (_imuPresent) {
        updateIMU();
        updateOrientation(dt);
    }
    
    // Assess slip risk based on all sensor data
    assessSlipRisk();
    
    // Calculate appropriate force multiplier
    calculateForceMultiplier();
}

void SensorFusion::updateFsrDerivatives(float dt) {
    // Calculate derivative and filter FSR values
    for (uint8_t i = 0; i < _fsrCount; i++) {
        // Calculate raw derivative (dF/dt)
        _fsrDerivatives[i] = (_fsrValues[i] - _fsrLastValues[i]) / dt;
        _fsrLastValues[i] = _fsrValues[i];
        
        // Apply exponential smoothing filter to FSR values
        const float alpha = 0.3f; // Smoothing factor
        _fsrFiltered[i] = alpha * _fsrValues[i] + (1.0f - alpha) * _fsrFiltered[i];
    }
}

void SensorFusion::updateIMU() {
    if (!_imuPresent) return;
    
    // Read raw values from MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);  // Starting register (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, (uint8_t)14, (uint8_t)true); // Read 14 bytes
    
    // Combine high and low bytes
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    int16_t temp = (Wire.read() << 8) | Wire.read(); // Temperature (unused)
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();
    
    // Convert to physical units
    // For ±2g range: 16384 LSB/g
    _accel[0] = ax / 16384.0f;
    _accel[1] = ay / 16384.0f;
    _accel[2] = az / 16384.0f;
    
    // For ±250 deg/s range: 131 LSB/(deg/s)
    _gyro[0] = gx / 131.0f;
    _gyro[1] = gy / 131.0f;
    _gyro[2] = gz / 131.0f;
}

void SensorFusion::updateOrientation(float dt) {
    if (!_imuPresent) return;
    
    // Simple complementary filter for orientation
    // Calculate angles from accelerometer (gravity vector)
    float accelPitch = atan2(_accel[1], sqrt(_accel[0] * _accel[0] + _accel[2] * _accel[2])) * RAD_TO_DEG;
    float accelRoll = atan2(-_accel[0], _accel[2]) * RAD_TO_DEG;
    
    // Integrate gyro rates
    _orientation[0] += _gyro[0] * dt; // Roll
    _orientation[1] += _gyro[1] * dt; // Pitch
    _orientation[2] += _gyro[2] * dt; // Yaw (gyro only)
    
    // Apply complementary filter (fusion)
    const float alpha = 0.98f;
    _orientation[0] = alpha * _orientation[0] + (1.0f - alpha) * accelRoll;
    _orientation[1] = alpha * _orientation[1] + (1.0f - alpha) * accelPitch;
    
    // Keep yaw in range [0, 360)
    while (_orientation[2] < 0) _orientation[2] += 360.0f;
    while (_orientation[2] >= 360.0f) _orientation[2] -= 360.0f;
}

void SensorFusion::assessSlipRisk() {
    float risk = 0.0f;
    
    // 1. Check for negative FSR derivatives (sign of slip)
    float maxNegDerivative = 0.0f;
    for (uint8_t i = 0; i < _fsrCount; i++) {
        if (_fsrFiltered[i] > 0.05f && _fsrDerivatives[i] < 0) {
            // Filter out noise by requiring some contact force
            float negDeriv = -_fsrDerivatives[i];
            if (negDeriv > maxNegDerivative) {
                maxNegDerivative = negDeriv;
            }
        }
    }
    
    // Convert to risk factor (0.0-0.5 range)
    if (maxNegDerivative > 0) {
        risk += constrain(maxNegDerivative * 2.0f, 0.0f, 0.5f);
    }
    
    // 2. Check orientation if IMU is present
    if (_imuPresent) {
        // Higher risk when hand is tilted down (negative pitch)
        if (_orientation[1] < -30.0f) {
            float pitchRisk = map(_orientation[1], -30.0f, -90.0f, 0.0f, 0.5f);
            pitchRisk = constrain(pitchRisk, 0.0f, 0.5f);
            risk += pitchRisk;
        }
        
        // Higher risk with extreme roll angle
        float absRoll = abs(_orientation[0]);
        if (absRoll > 45.0f) {
            float rollRisk = map(absRoll, 45.0f, 90.0f, 0.0f, 0.3f);
            rollRisk = constrain(rollRisk, 0.0f, 0.3f);
            risk += rollRisk;
        }
    }
    
    // Constrain total risk to [0, 1] range
    _slipRisk = constrain(risk, 0.0f, 1.0f);
}

void SensorFusion::calculateForceMultiplier() {
    // Base multiplier is 1.0
    float multiplier = 1.0f;
    
    // Increase force when slip risk is high
    if (_slipRisk > 0.2f) {
        multiplier += _slipRisk * 0.8f;
    }
    
    // Adjust based on orientation if IMU present
    if (_imuPresent) {
        // Add force when palm is facing down (risk of dropping)
        if (_orientation[1] < -30.0f) {
            float pitchFactor = map(_orientation[1], -30.0f, -90.0f, 0.0f, 0.5f);
            pitchFactor = constrain(pitchFactor, 0.0f, 0.5f);
            multiplier += pitchFactor;
        }
    }
    
    // Constrain to reasonable range
    _forceMultiplier = constrain(multiplier, 0.8f, 2.0f);
}

void SensorFusion::setFSRValues(const float* values, uint8_t count) {
    count = min(count, MAX_FSR_COUNT);
    _fsrCount = count;
    
    for (uint8_t i = 0; i < count; i++) {
        _fsrValues[i] = values[i];
    }
}

void SensorFusion::setEMGValues(const float* values, uint8_t count) {
    count = min(count, MAX_EMG_COUNT);
    _emgCount = count;
    
    for (uint8_t i = 0; i < count; i++) {
        _emgValues[i] = values[i];
    }
}

void SensorFusion::getOrientation(float* roll, float* pitch, float* yaw) const {
    if (roll) *roll = _orientation[0];
    if (pitch) *pitch = _orientation[1];
    if (yaw) *yaw = _orientation[2];
}

float SensorFusion::getSlipRisk() const {
    return _slipRisk;
}

float SensorFusion::getForceMultiplier() const {
    return _forceMultiplier;
}

bool SensorFusion::hasIMU() const {
    return _imuPresent;
}

float SensorFusion::getFsrDerivative(uint8_t index) const {
    return (index < _fsrCount) ? _fsrDerivatives[index] : 0.0f;
}

bool SensorFusion::detectSlip(uint8_t index, float threshold) const {
    if (index >= _fsrCount) return false;
    
    // Negative derivative indicates slip
    return (_fsrDerivatives[index] < threshold && _fsrValues[index] > 0.05f);
}