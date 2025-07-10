/**
 * @file SensorFusion.cpp
 * @brief Enhanced sensor fusion implementation with IMU integration
 * @version 2.0
 */

#include "SensorFusion.h"

SensorFusion::SensorFusion() 
    : _imuPresent(false),
      _numFingers(0),
      _numEmgChannels(0),
      _slipRisk(0.0f),
      _gripForceMultiplier(1.0f),
      _activityContext(0),
      _isMoving(false),
      _movementMagnitude(0.0f),
      _lastUpdateTime(0)
{
    // Initialize arrays
    memset(_accel, 0, sizeof(_accel));
    memset(_gyro, 0, sizeof(_gyro));
    memset(_orientation, 0, sizeof(_orientation));
    
    memset(_fsrValues, 0, sizeof(_fsrValues));
    memset(_prevFsrValues, 0, sizeof(_prevFsrValues));
    memset(_fsrDerivatives, 0, sizeof(_fsrDerivatives));
    
    memset(_emgFeatures, 0, sizeof(_emgFeatures));
}

bool SensorFusion::begin() {
    // Initialize I2C for IMU
    Wire.begin();
    
    // Initialize IMU if available
    _imuPresent = initIMU();
    
    return true;
}

bool SensorFusion::initIMU() {
    #ifdef USE_MPU6050
    // Initialize MPU6050
    _mpu.initialize();
    
    // Test connection
    if (!_mpu.testConnection()) {
        return false;
    }
    
    // Configure MPU6050
    _mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);    // ±2g
    _mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);    // ±250 deg/s
    _mpu.setDLPFMode(MPU6050_DLPF_BW_20);              // 20 Hz low-pass filter
    
    return true;
    #else
    return false;
    #endif
}

void SensorFusion::update() {
    uint32_t currentTime = millis();
    float dt = (currentTime - _lastUpdateTime) / 1000.0f;
    
    // Avoid division by zero on first call
    if (_lastUpdateTime == 0) dt = 0.01f;
    _lastUpdateTime = currentTime;
    
    // Update FSR derivatives
    for (uint8_t i = 0; i < _numFingers; i++) {
        _fsrDerivatives[i] = (_fsrValues[i] - _prevFsrValues[i]) / dt;
        _prevFsrValues[i] = _fsrValues[i];
    }
    
    // Update IMU data if available
    if (_imuPresent) {
        updateIMU();
        updateOrientation(dt);
    }
    
    // Detect movement based on IMU
    detectMovement();
    
    // Perform risk assessment based on sensor fusion
    assessSlipRisk();
    
    // Determine activity context
    determineActivityContext();
    
    // Calculate appropriate grip force
    _gripForceMultiplier = calculateGripForceMultiplier();
}

void SensorFusion::updateIMU() {
    #ifdef USE_MPU6050
    if (!_imuPresent) return;
    
    // Read raw values from MPU6050
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    _mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert to physical units
    // Accelerometer: ±2g range = 16384 units/g
    _accel[0] = ax / 16384.0f;  // X acceleration in g
    _accel[1] = ay / 16384.0f;  // Y acceleration in g
    _accel[2] = az / 16384.0f;  // Z acceleration in g
    
    // Gyroscope: ±250 deg/s range = 131 units/(deg/s)
    _gyro[0] = gx / 131.0f;     // X rotation in deg/s
    _gyro[1] = gy / 131.0f;     // Y rotation in deg/s
    _gyro[2] = gz / 131.0f;     // Z rotation in deg/s
    #endif
}

void SensorFusion::updateOrientation(float dt) {
    if (!_imuPresent) return;
    
    // Complementary filter for orientation estimation
    // This is a simplified implementation - consider using a proper AHRS algorithm
    // like Madgwick or Mahony for better results
    
    // Calculate angles from accelerometer (gravity vector)
    float accelPitch = atan2(_accel[1], sqrt(_accel[0] * _accel[0] + _accel[2] * _accel[2])) * RAD_TO_DEG;
    float accelRoll = atan2(-_accel[0], _accel[2]) * RAD_TO_DEG;
    
    // Integrate gyro rates
    _orientation[0] += _gyro[0] * dt;  // Roll
    _orientation[1] += _gyro[1] * dt;  // Pitch
    _orientation[2] += _gyro[2] * dt;  // Yaw (gyro only)
    
    // Apply complementary filter (fusion)
    const float alpha = 0.98f;
    _orientation[0] = alpha * _orientation[0] + (1-alpha) * accelRoll;
    _orientation[1] = alpha * _orientation[1] + (1-alpha) * accelPitch;
    
    // Normalize yaw to 0-360 degrees
    while (_orientation[2] < 0) _orientation[2] += 360.0f;
    while (_orientation[2] >= 360.0f) _orientation[2] -= 360.0f;
}

void SensorFusion::setEMGFeatures(const float* emgFeatures, uint8_t numChannels) {
    _numEmgChannels = min(numChannels, (uint8_t)MAX_EMG_CHANNELS);
    
    for (uint8_t i = 0; i < _numEmgChannels; i++) {
        _emgFeatures[i] = emgFeatures[i];
    }
}

void SensorFusion::setFSRValues(const float* fsrValues, uint8_t numFingers) {
    _numFingers = min(numFingers, (uint8_t)MAX_FINGERS);
    
    for (uint8_t i = 0; i < _numFingers; i++) {
        _fsrValues[i] = fsrValues[i];
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

float SensorFusion::getGripForceMultiplier() const {
    return _gripForceMultiplier;
}

uint8_t SensorFusion::getActivityContext() const {
    return _activityContext;
}

bool SensorFusion::hasIMU() const {
    return _imuPresent;
}

void SensorFusion::detectMovement() {
    if (!_imuPresent) {
        _isMoving = false;
        _movementMagnitude = 0.0f;
        return;
    }
    
    // Calculate total acceleration magnitude
    float accelMagnitude = sqrt(
        _accel[0] * _accel[0] +
        _accel[1] * _accel[1] +
        _accel[2] * _accel[2]
    );
    
    // Calculate deviation from gravity (1.0g)
    float accelDeviation = abs(accelMagnitude - 1.0f);
    
    // Calculate angular velocity magnitude
    float gyroMagnitude = sqrt(
        _gyro[0] * _gyro[0] +
        _gyro[1] * _gyro[1] +
        _gyro[2] * _gyro[2]
    );
    
    // Combine for overall movement magnitude
    _movementMagnitude = accelDeviation * 3.0f + gyroMagnitude * 0.01f;
    
    // Detect movement with hysteresis
    if (_movementMagnitude > 0.2f) {
        _isMoving = true;
    } else if (_movementMagnitude < 0.1f) {
        _isMoving = false;
    }
}

void SensorFusion::assessSlipRisk() {
    float risk = 0.0f;
    
    // 1. Check orientation (higher risk when palm faces down)
    if (_imuPresent) {
        // Pitch gives palm orientation relative to ground
        float pitchRisk = 0.0f;
        
        // Higher risk when palm faces down (-90°)
        if (_orientation[1] < -45.0f) {
            pitchRisk = map(_orientation[1], -45.0f, -90.0f, 0.0f, 0.5f);
        }
        
        // Higher risk with extreme roll angles (±90°)
        float rollRisk = 0.0f;
        if (abs(_orientation[0]) > 45.0f) {
            rollRisk = map(abs(_orientation[0]), 45.0f, 90.0f, 0.0f, 0.3f);
        }
        
        risk += max(pitchRisk, rollRisk);
    }
    
    // 2. Check FSR derivatives (negative = slipping)
    float maxNegativeDerivative = 0.0f;
    for (uint8_t i = 0; i < _numFingers; i++) {
        if (_fsrValues[i] > 0.1f && _fsrDerivatives[i] < 0) {
            maxNegativeDerivative = min(maxNegativeDerivative, _fsrDerivatives[i]);
        }
    }
    
    if (maxNegativeDerivative < 0) {
        // Convert negative derivative to risk factor (more negative = higher risk)
        risk += min(-maxNegativeDerivative * 2.0f, 0.5f);
    }
    
    // 3. Add risk from movement
    if (_isMoving) {
        risk += min(_movementMagnitude * 0.5f, 0.3f);
    }
    
    // Clamp to 0-1 range
    _slipRisk = constrain(risk, 0.0f, 1.0f);
}

void SensorFusion::determineActivityContext() {
    // Default: idle
    uint8_t context = 0;
    
    // Check if holding an object
    bool isHolding = false;
    uint8_t activeFingers = 0;
    for (uint8_t i = 0; i < _numFingers; i++) {
        if (_fsrValues[i] > 0.2f) {
            activeFingers++;
        }
    }
    
    if (activeFingers >= 2) {
        isHolding = true;
        context = 1;  // Holding object
    }
    
    // Check if moving
    if (isHolding && _isMoving && _movementMagnitude > 0.3f) {
        context = 2;  // Moving with object
    }
    
    _activityContext = context;
}

float SensorFusion::calculateGripForceMultiplier() const {
    // Base multiplier
    float multiplier = 1.0f;
    
    // Adjust for slip risk
    if (_slipRisk > 0.2f) {
        multiplier += _slipRisk * 0.8f;  // Up to +0.8 (1.8x force)
    }
    
    // Adjust for activity context
    switch (_activityContext) {
        case 0:  // Idle
            multiplier *= 0.8f;  // Reduce force
            break;
        case 2:  // Moving with object
            multiplier *= 1.2f;  // Increase force
            break;
        default:
            break;
    }
    
    // Low battery case (would be provided by PowerMonitor)
    // if (batteryLow) multiplier = min(multiplier, 1.0f);
    
    // Clamp to reasonable range
    return constrain(multiplier, 0.5f, 2.0f);
}

void SensorFusion::printStatus(Stream& stream) const {
    stream.print(F("Activity: "));
    switch (_activityContext) {
        case 0: stream.print(F("Idle")); break;
        case 1: stream.print(F("Holding")); break;
        case 2: stream.print(F("Moving")); break;
    }
    
    stream.print(F(" | Slip Risk: "));
    stream.print(_slipRisk, 2);
    
    stream.print(F(" | Force Mult: "));
    stream.print(_gripForceMultiplier, 2);
    
    if (_imuPresent) {
        stream.print(F(" | Orient[R,P]: "));
        stream.print(_orientation[0], 1); // Roll
        stream.print(F(","));
        stream.print(_orientation[1], 1); // Pitch
    }
    
    stream.println();
}