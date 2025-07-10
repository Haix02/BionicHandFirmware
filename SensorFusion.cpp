/**
 * @file SensorFusion.cpp
 * @brief Implementation of enhanced sensor fusion
 * @version 3.0
 */

#include "SensorFusion.h"
#include <MPU6050.h> // I2C IMU library

SensorFusion::SensorFusion()
    : _imuPresent(false), _totalGripForce(0.0f), _stableGrip(false),
      _emgActivity(0.0f), _slipRisk(0.0f), _recommendedForceScale(1.0f),
      _gripStability(1.0f), _activityContext(0)
{
    // Initialize arrays
    memset(_accelData, 0, sizeof(_accelData));
    memset(_gyroData, 0, sizeof(_gyroData));
    memset(_orientation, 0, sizeof(_orientation));
    memset(_fsrValues, 0, sizeof(_fsrValues));
    memset(_emgFeatures, 0, sizeof(_emgFeatures));
}

bool SensorFusion::begin() {
    // Initialize IMU if present
    _imuPresent = initIMU();
    
    return true;
}

bool SensorFusion::initIMU() {
    // Initialize I2C
    Wire.begin();
    
    // Try to initialize MPU6050
    MPU6050 mpu;
    Wire.beginTransmission(MPU6050_ADDRESS);
    if (Wire.endTransmission() != 0) {
        // IMU not found
        return false;
    }
    
    // IMU found, initialize
    mpu.initialize();
    
    // Configure for appropriate sensitivity
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    
    // Set digital low pass filter
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);
    
    return true;
}

void SensorFusion::update() {
    // Update all sensor fusion components
    
    // 1. Update IMU data if available
    if (_imuPresent) {
        updateIMU();
        calculateOrientation();
    }
    
    // 2. Process and fuse all sensor data
    assessSlipRisk();
    assessGripStability();
    classifyActivity();
    calculateRecommendedForce();
}

void SensorFusion::updateIMU() {
    MPU6050 mpu;
    
    // Read raw accel/gyro values
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert to physical units
    // Accelerometer: ±2g range, 16-bit ADC = 16384 units per g
    _accelData[0] = ax / 16384.0f;
    _accelData[1] = ay / 16384.0f;
    _accelData[2] = az / 16384.0f;
    
    // Gyroscope: ±250°/s range, 16-bit ADC = 131 units per °/s
    _gyroData[0] = gx / 131.0f;
    _gyroData[1] = gy / 131.0f;
    _gyroData[2] = gz / 131.0f;
}

void SensorFusion::calculateOrientation() {
    // Simple complementary filter for orientation estimation
    static uint32_t lastUpdate = 0;
    uint32_t now = micros();
    float dt = (now - lastUpdate) / 1000000.0f; // Convert to seconds
    lastUpdate = now;
    
    // Prevent large dt on first call
    if (dt > 0.1f) dt = 0.01f;
    
    // Calculate roll and pitch from accelerometer (gravity vector)
    float accel_roll = atan2(_accelData[1], _accelData[2]) * 180.0f / PI;
    float accel_pitch = atan2(-_accelData[0], sqrt(_accelData[1] * _accelData[1] + 
                                                 _accelData[2] * _accelData[2])) * 180.0f / PI;
    
    // Integrate gyro data
    _orientation[0] += _gyroData[0] * dt; // Roll
    _orientation[1] += _gyroData[1] * dt; // Pitch
    _orientation[2] += _gyroData[2] * dt; // Yaw (gyro only, no reference)
    
    // Complementary filter: combine accelerometer and gyro
    const float alpha = 0.98f;
    _orientation[0] = alpha * _orientation[0] + (1.0f - alpha) * accel_roll;
    _orientation[1] = alpha * _orientation[1] + (1.0f - alpha) * accel_pitch;
    
    // Normalize yaw to 0-360°
    while (_orientation[2] < 0) _orientation[2] += 360.0f;
    while (_orientation[2] >= 360.0f) _orientation[2] -= 360.0f;
}

void SensorFusion::setEMGFeatures(const float* emgFeatures, uint8_t numFeatures) {
    // Copy EMG feature data
    numFeatures = min(numFeatures, NUM_EMG_CHANNELS);
    memcpy(_emgFeatures, emgFeatures, numFeatures * sizeof(float));
    
    // Calculate overall EMG activity level
    _emgActivity = 0.0f;
    for (uint8_t i = 0; i < numFeatures; i++) {
        _emgActivity += _emgFeatures[i];
    }
    _emgActivity /= numFeatures;
}

void SensorFusion::setFSRValues(const float* fsrValues, uint8_t numFSRs) {
    // Copy FSR data
    numFSRs = min(numFSRs, NUM_FINGERS);
    memcpy(_fsrValues, fsrValues, numFSRs * sizeof(float));
    
    // Calculate total grip force
    _totalGripForce = 0.0f;
    for (uint8_t i = 0; i < numFSRs; i++) {
        _totalGripForce += _fsrValues[i];
    }
    
    // Determine if grip is stable
    // A grip is considered stable if at least two fingers have significant force
    uint8_t activeFingers = 0;
    for (uint8_t i = 0; i < numFSRs; i++) {
        if (_fsrValues[i] > 0.2f) activeFingers++;
    }
    _stableGrip = (activeFingers >= 2);
}

void SensorFusion::getHandOrientation(float* roll, float* pitch, float* yaw) {
    *roll = _orientation[0];
    *pitch = _orientation[1];
    *yaw = _orientation[2];
}

float SensorFusion::getSlipRisk() const {
    return _slipRisk;
}

float SensorFusion::getRecommendedGripForce() const {
    return _recommendedForceScale;
}

float SensorFusion::getGripStability() const {
    return _gripStability;
}

uint8_t SensorFusion::getActivityContext() const {
    return _activityContext;
}

void SensorFusion::assessSlipRisk() {
    // Calculate slip risk based on multiple factors:
    // 1. Hand orientation relative to gravity
    // 2. Current grip force
    // 3. Detected movement/vibration
    
    float riskScore = 0.0f;
    
    // Orientation risk: higher risk when hand is tilted with fingers pointing down
    if (_imuPresent) {
        // Calculate tilt angle relative to gravity
        float tiltAngle = abs(_orientation[1]); // pitch (forward/back tilt)
        
        // Higher risk when hand is tilted downward
        if (tiltAngle > 45.0f) {
            riskScore += map(tiltAngle, 45.0f, 90.0f, 0.0f, 0.5f);
        }
        
        // Also consider roll (side-to-side tilt)
        float rollAngle = abs(_orientation[0]);
        if (rollAngle > 45.0f) {
            riskScore += map(rollAngle, 45.0f, 90.0f, 0.0f, 0.3f);
        }
    }
    
    // Grip force risk: higher risk with lower grip force
    if (_totalGripForce < 0.5f && _stableGrip) {
        riskScore += map(_totalGripForce, 0.0f, 0.5f, 0.5f, 0.0f);
    }
    
    // Movement risk: higher risk during rapid movement
    if (_imuPresent) {
        float angularVelocity = sqrt(_gyroData[0]*_gyroData[0] + 
                                    _gyroData[1]*_gyroData[1] + 
                                    _gyroData[2]*_gyroData[2]);
        if (angularVelocity > 20.0f) {
            riskScore += map(angularVelocity, 20.0f, 100.0f, 0.0f, 0.3f);
        }
    }
    
    // Cap at 1.0
    _slipRisk = constrain(riskScore, 0.0f, 1.0f);
}

void SensorFusion::assessGripStability() {
    // Assess grip stability based on:
    // 1. FSR pressure distribution
    // 2. Variation in pressure
    // 3. Number of contact points
    
    float stabilityScore = 1.0f;
    
    // Count active contact points
    uint8_t contactPoints = 0;
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
        if (_fsrValues[i] > 0.1f) contactPoints++;
    }
    
    // More contact points = more stable
    if (contactPoints < 2) {
        stabilityScore *= 0.5f;
    }
    
    // Calculate pressure distribution evenness
    if (contactPoints >= 2) {
        float mean = _totalGripForce / contactPoints;
        float variance = 0.0f;
        
        for (uint8_t i = 0; i < NUM_FINGERS; i++) {
            if (_fsrValues[i] > 0.1f) {
                variance += (_fsrValues[i] - mean) * (_fsrValues[i] - mean);
            }
        }
        variance /= contactPoints;
        
        // Higher variance = less stable grip
        float variancePenalty = constrain(variance * 2.0f, 0.0f, 0.5f);
        stabilityScore *= (1.0f - variancePenalty);
    }
    
    // Consider EMG activity - high activity might indicate unstable grip
    if (_emgActivity > 0.7f) {
        stabilityScore *= 0.8f;
    }
    
    _gripStability = stabilityScore;
}

void SensorFusion::classifyActivity() {
    // Simple activity context classification
    // 0: Rest/idle
    // 1: Active grip (holding)
    // 2: Dynamic manipulation
    // 3: Lifting/moving
    // 4: Precision grip
    
    // Default to idle
    uint8_t context = 0;
    
    // Check grip state
    if (_totalGripForce > 0.3f) {
        // Some kind of grip is active
        context = 1;
        
        // Check for precision grip pattern (typically index+thumb)
        if (_fsrValues[0] > 0.3f && _fsrValues[1] > 0.3f && 
            _fsrValues[2] < 0.2f && _fsrValues[3] < 0.2f) {
            context = 4; // Precision grip
        }
        
        // Check for lifting/moving (grip + acceleration)
        if (_imuPresent) {
            float accelMagnitude = sqrt(_accelData[0]*_accelData[0] + 
                                       _accelData[1]*_accelData[1] + 
                                       _accelData[2]*_accelData[2]);
            if (accelMagnitude > 1.2f) { // More than 1.2g
                context = 3; // Lifting/moving
            }
        }
        
        // Check for dynamic manipulation (grip + changing finger forces)
        static float prevForces[NUM_FINGERS] = {0};
        float forceChangeSum = 0;
        for (uint8_t i = 0; i < NUM_FINGERS; i++) {
            forceChangeSum += abs(_fsrValues[i] - prevForces[i]);
            prevForces[i] = _fsrValues[i];
        }
        
        if (forceChangeSum > 0.3f) {
            context = 2; // Dynamic manipulation
        }
    }
    
    _activityContext = context;
}

void SensorFusion::calculateRecommendedForce() {
    // Base force is 1.0 (normal)
    float force = 1.0f;
    
    // Adjust based on slip risk
    if (_slipRisk > 0.3f) {
        force += _slipRisk * 0.5f; // Up to 50% more force when slip risk is high
    }
    
    // Adjust based on activity context
    switch (_activityContext) {
        case 0: // Idle
            force *= 0.7f; // Reduce force when idle
            break;
        case 2: // Dynamic manipulation
            force *= 0.9f; // Slightly reduce for dexterity
            break;
        case 3: // Lifting/moving
            force *= 1.2f; // Increase for security during movement
            break;
        case 4: // Precision grip
            force *= 0.8f; // Reduce for precision
            break;
    }
    
    _recommendedForceScale = constrain(force, 0.6f, 1.5f);
}

void SensorFusion::printDebug(Stream& s) {
    s.print(F("Slip Risk: ")); s.print(_slipRisk, 2);
    s.print(F(", Grip Force: ")); s.print(_totalGripForce, 2);
    s.print(F(", Stability: ")); s.print(_gripStability, 2);
    s.print(F(", Context: ")); s.print(_activityContext);
    
    if (_imuPresent) {
        s.print(F(", Orient: ["));
        s.print(_orientation[0], 1); s.print(F(","));
        s.print(_orientation[1], 1); s.print(F(","));
        s.print(_orientation[2], 1); s.print(F("]"));
    }
}