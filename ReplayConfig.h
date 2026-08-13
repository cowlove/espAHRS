#pragma once

#include "AircraftAHRS.h"
#include "HardwareAbstraction.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

// One central replay configuration surface.  Sweep tools can pass repeated
// --param name=value arguments without teaching the replay loop about each
// individual AHRS subsystem.
struct ReplayConfig {
    AircraftAHRS::Config ahrs;
    float g5HeadingOffsetDeg = 0.0f;
    // Added to raw G5 pitch before comparisons/CSV output.  This normalizes a
    // known reference-instrument trim error without contaminating sensor
    // mounting offsets or gyro calibration.
    // Run-1 straight-and-level G5 pitch averaged -4.641758 degrees.  Treat
    // that as reference-instrument trim so replay comparisons use a level
    // zero without folding the G5 error into aircraft sensor calibration.
    float g5PitchBiasDeg = 4.641758f;
    float g5TimeOffsetMs = 20.0f;
    float sensorPitchOffsetDeg = 11.7f;
    float sensorRollOffsetDeg = 6.0f;
    float sensorYawOffsetDeg = 0.0f;
    float accelInputScale = 1.0f;
    uint8_t selectedImuSource = 0;
    uint8_t selectedCompassSource = 0;
    float rawGyroBiasDegSec[3] = {0.0f, 0.0f, 0.0f};
    float rawGyroAxisSign[3] = {1.0f, 1.0f, 1.0f};

    explicit ReplayConfig(const HalHardwareProfile &hardware) {
        const HalSensorCalibration &calibration = hardware.calibration;
        sensorPitchOffsetDeg = calibration.sensorPitchOffsetDeg;
        sensorRollOffsetDeg = calibration.sensorRollOffsetDeg;
        sensorYawOffsetDeg = calibration.sensorYawOffsetDeg;
        for (int axis = 0; axis < 3; ++axis) {
            rawGyroBiasDegSec[axis] = calibration.gyroBiasDegSec[axis];
            rawGyroAxisSign[axis] = calibration.gyroAxisSign[axis];
        }
        if (calibration.applyAccelBias) {
            ahrs.accelBiasXMps2 = calibration.accelBiasMps2[0];
            ahrs.accelBiasYMps2 = calibration.accelBiasMps2[1];
            ahrs.accelBiasZMps2 = calibration.accelBiasMps2[2];
        }
    }

    bool set(const char *name, float value) {
        struct Field { const char *name; float *value; } fields[] = {
            {"yaw_correction_sec", &ahrs.yawCorrectionTimeSec},
            {"roll_correction_sec", &ahrs.rollCorrectionTimeSec},
            {"pitch_correction_sec", &ahrs.pitchCorrectionTimeSec},
            {"gps_derivative_sec", &ahrs.gpsDerivativeTimeSec},
            {"gps_longitudinal_accel_filter_sec", &ahrs.gpsLongitudinalAccelerationFilterTimeSec},
            {"gps_longitudinal_accel_compensation_gain", &ahrs.gpsLongitudinalAccelerationCompensationGain},
            {"gps_longitudinal_accel_max_bank_deg", &ahrs.gpsLongitudinalAccelerationMaximumBankDeg},
            {"magnetic_derivative_sec", &ahrs.magneticDerivativeTimeSec},
            {"yaw_gyro_derivative_sec", &ahrs.yawGyroDerivativeTimeSec},
            {"fused_heading_filter_sec", &ahrs.fusedHeadingFilterTimeSec},
            {"gps_heading_speed_threshold_mps", &ahrs.gpsHeadingSpeedThresholdMps},
            {"gps_heading_weight", &ahrs.gpsHeadingWeight},
            {"g5_pitch_bias_deg", &g5PitchBiasDeg},
            {"gyro_bias_x_deg_sec", &rawGyroBiasDegSec[0]},
            {"gyro_bias_y_deg_sec", &rawGyroBiasDegSec[1]},
            {"gyro_bias_z_deg_sec", &rawGyroBiasDegSec[2]},
            {"gyro_axis_sign_x", &rawGyroAxisSign[0]},
            {"gyro_axis_sign_y", &rawGyroAxisSign[1]},
            {"gyro_axis_sign_z", &rawGyroAxisSign[2]},
            {"gyro_gain_x", &ahrs.gyroGainX},
            {"gyro_gain_y", &ahrs.gyroGainY},
            {"gyro_gain_z", &ahrs.gyroGainZ},
            {"gyro_rate_limit_deg_sec", &ahrs.gyroRateLimitDegSec},
            {"gyro_integration_dt_sec", &ahrs.gyroIntegrationDtSec},
            {"accel_bias_x_mps2", &ahrs.accelBiasXMps2},
            {"accel_bias_y_mps2", &ahrs.accelBiasYMps2},
            {"accel_bias_z_mps2", &ahrs.accelBiasZMps2},
            {"vertical_rate_filter_sec", &ahrs.verticalRateFilterTimeSec},
            {"angle_of_attack_deg", &ahrs.angleOfAttackDeg},
            {"gps_timeout_sec", &ahrs.gpsTimeoutSec},
            {"accel_correction_sec", &ahrs.accelCorrectionTimeSec},
            {"pitch_gravity_correction_sec", &ahrs.pitchGravityCorrectionTimeSec},
            {"accel_filter_sec", &ahrs.accelFilterTimeSec},
            {"accel_tolerance_mps2", &ahrs.accelMagnitudeToleranceMps2},
            {"gps_turn_rate_bank_weight", &ahrs.gpsTurnRateBankWeight},
            {"mag_turn_rate_bank_weight", &ahrs.magTurnRateBankWeight},
            {"yaw_gyro_turn_rate_bank_weight", &ahrs.yawGyroTurnRateBankWeight},
            {"accelerometer_roll_weight", &ahrs.accelerometerRollWeight},
            {"fused_turn_rate_bank_weight", &ahrs.fusedTurnRateBankWeight},
            {"dip_ahrs_roll_weight", &ahrs.dipAhrsRollWeight},
            {"maximum_bank_target_deg", &ahrs.maximumBankTargetDeg},
            {"magnetic_declination_deg", &ahrs.magneticDeclinationDeg},
            {"magnetic_inclination_deg", &ahrs.magneticInclinationDeg},
            {"magnetic_field_magnitude_tolerance", &ahrs.magneticFieldMagnitudeTolerance},
            {"magnetic_roll_max_disagreement_deg", &ahrs.magneticRollMaximumDisagreementDeg},
            {"magnetic_roll_min_geometry", &ahrs.magneticRollMinimumGeometry},
            {"min_ground_speed_mps", &ahrs.minimumGroundSpeedMps},
            {"baro_alt_filter_sec", &ahrs.baroAltitudeFilterTimeSec},
            {"baro_rate_filter_sec", &ahrs.baroRateFilterTimeSec},
            {"baro_gps_bias_sec", &ahrs.baroGpsBiasTimeSec},
            {"baro_timeout_sec", &ahrs.baroTimeoutSec},
        };
        for (const auto &field : fields) if (std::strcmp(name, field.name) == 0) {
            *field.value = value; return true;
        }
        if (std::strcmp(name, "g5_heading_offset_deg") == 0) {
            g5HeadingOffsetDeg = value; return true;
        }
        if (std::strcmp(name, "g5_time_offset_ms") == 0) {
            g5TimeOffsetMs = value; return true;
        }
        if (std::strcmp(name, "sensor_pitch_offset_deg") == 0) {
            sensorPitchOffsetDeg = value; return true;
        }
        if (std::strcmp(name, "sensor_roll_offset_deg") == 0) {
            sensorRollOffsetDeg = value; return true;
        }
        if (std::strcmp(name, "sensor_yaw_offset_deg") == 0) {
            sensorYawOffsetDeg = value; return true;
        }
        if (std::strcmp(name, "accel_input_scale") == 0) {
            accelInputScale = value; return true;
        }
        if (std::strcmp(name, "imu_source") == 0 && value >= 0.0f && value < 4.0f) {
            selectedImuSource = static_cast<uint8_t>(value); return true;
        }
        if (std::strcmp(name, "compass_source") == 0 && value >= 0.0f && value < 4.0f) {
            selectedCompassSource = static_cast<uint8_t>(value); return true;
        }
        return false;
    }

    static void list() {
        std::puts("tunable parameters: yaw_correction_sec roll_correction_sec "
                   "pitch_correction_sec gps_derivative_sec gps_longitudinal_accel_filter_sec "
                   "gps_longitudinal_accel_compensation_gain angle_of_attack_deg "
                   "gps_longitudinal_accel_max_bank_deg "
                   "magnetic_derivative_sec yaw_gyro_derivative_sec "
                   "fused_heading_filter_sec gps_heading_speed_threshold_mps "
                   "gps_heading_weight "
                   "gyro_bias_x_deg_sec gyro_bias_y_deg_sec gyro_bias_z_deg_sec "
                   "gyro_axis_sign_x gyro_axis_sign_y gyro_axis_sign_z "
                   "gyro_rate_limit_deg_sec "
                   "gyro_integration_dt_sec "
                   "accel_bias_x_mps2 accel_bias_y_mps2 accel_bias_z_mps2 "
                   "vertical_rate_filter_sec "
                   "gps_timeout_sec accel_correction_sec accel_filter_sec "
                   "pitch_gravity_correction_sec "
                   "accel_tolerance_mps2 gps_turn_rate_bank_weight "
                   "mag_turn_rate_bank_weight yaw_gyro_turn_rate_bank_weight "
                   "accelerometer_roll_weight fused_turn_rate_bank_weight "
                   "dip_ahrs_roll_weight "
                   "maximum_bank_target_deg magnetic_declination_deg "
                   "magnetic_inclination_deg magnetic_field_magnitude_tolerance "
                   "magnetic_roll_max_disagreement_deg magnetic_roll_min_geometry "
                   "min_ground_speed_mps baro_alt_filter_sec "
                   "baro_rate_filter_sec baro_gps_bias_sec baro_timeout_sec "
                   "g5_heading_offset_deg g5_pitch_bias_deg g5_time_offset_ms "
                   "sensor_pitch_offset_deg sensor_roll_offset_deg sensor_yaw_offset_deg accel_input_scale "
                   "imu_source compass_source");
    }
};
