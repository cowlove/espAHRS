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
    float g5TimeOffsetMs = 20.0f;
    float sensorPitchOffsetDeg = 11.7f;
    float sensorRollOffsetDeg = 6.0f;
    float accelInputScale = 1.0f;

    ReplayConfig() {
        constexpr HalHardwareProfile hardware = makeGeekS3Profile();
        const HalSensorCalibration &calibration = hardware.calibration;
        sensorPitchOffsetDeg = calibration.sensorPitchOffsetDeg;
        sensorRollOffsetDeg = calibration.sensorRollOffsetDeg;
        ahrs.gyroBiasXDegSec = calibration.gyroBiasDegSec[0];
        ahrs.gyroBiasYDegSec = calibration.gyroBiasDegSec[1];
        ahrs.gyroBiasZDegSec = calibration.gyroBiasDegSec[2];
        ahrs.gyroAxisSignX = calibration.gyroAxisSign[0];
        ahrs.gyroAxisSignY = calibration.gyroAxisSign[1];
        ahrs.gyroAxisSignZ = calibration.gyroAxisSign[2];
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
            {"fused_heading_filter_sec", &ahrs.fusedHeadingFilterTimeSec},
            {"gps_heading_speed_threshold_mps", &ahrs.gpsHeadingSpeedThresholdMps},
            {"gps_heading_weight", &ahrs.gpsHeadingWeight},
            {"gyro_bias_x_deg_sec", &ahrs.gyroBiasXDegSec},
            {"gyro_bias_y_deg_sec", &ahrs.gyroBiasYDegSec},
            {"gyro_bias_z_deg_sec", &ahrs.gyroBiasZDegSec},
            {"gyro_axis_sign_x", &ahrs.gyroAxisSignX},
            {"gyro_axis_sign_y", &ahrs.gyroAxisSignY},
            {"gyro_axis_sign_z", &ahrs.gyroAxisSignZ},
            {"accel_bias_x_mps2", &ahrs.accelBiasXMps2},
            {"accel_bias_y_mps2", &ahrs.accelBiasYMps2},
            {"accel_bias_z_mps2", &ahrs.accelBiasZMps2},
            {"vertical_rate_filter_sec", &ahrs.verticalRateFilterTimeSec},
            {"vertical_accel_tolerance_mps2", &ahrs.verticalAccelerationToleranceMps2},
            {"vertical_smoothness_window_sec", &ahrs.verticalSmoothnessWindowSec},
            {"angle_of_attack_deg", &ahrs.angleOfAttackDeg},
            {"gps_timeout_sec", &ahrs.gpsTimeoutSec},
            {"accel_correction_sec", &ahrs.accelCorrectionTimeSec},
            {"pitch_gravity_correction_sec", &ahrs.pitchGravityCorrectionTimeSec},
            {"accel_filter_sec", &ahrs.accelFilterTimeSec},
            {"accel_tolerance_mps2", &ahrs.accelMagnitudeToleranceMps2},
            {"accelerometer_roll_weight", &ahrs.accelerometerRollWeight},
            {"turn_bank_weight", &ahrs.turnBankWeight},
            {"maximum_bank_target_deg", &ahrs.maximumBankTargetDeg},
            {"magnetic_declination_deg", &ahrs.magneticDeclinationDeg},
            {"magnetic_inclination_deg", &ahrs.magneticInclinationDeg},
            {"magnetic_roll_correction_sec", &ahrs.magneticRollCorrectionTimeSec},
            {"magnetic_roll_weight", &ahrs.magneticRollWeight},
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
        if (std::strcmp(name, "accel_input_scale") == 0) {
            accelInputScale = value; return true;
        }
        return false;
    }

    static void list() {
        std::puts("tunable parameters: yaw_correction_sec roll_correction_sec "
                   "pitch_correction_sec gps_derivative_sec angle_of_attack_deg "
                   "fused_heading_filter_sec gps_heading_speed_threshold_mps "
                   "gps_heading_weight "
                   "gyro_bias_x_deg_sec gyro_bias_y_deg_sec gyro_bias_z_deg_sec "
                   "gyro_axis_sign_x gyro_axis_sign_y gyro_axis_sign_z "
                   "accel_bias_x_mps2 accel_bias_y_mps2 accel_bias_z_mps2 "
                   "vertical_rate_filter_sec vertical_accel_tolerance_mps2 "
                   "vertical_smoothness_window_sec "
                   "gps_timeout_sec accel_correction_sec accel_filter_sec "
                   "pitch_gravity_correction_sec "
                   "accel_tolerance_mps2 accelerometer_roll_weight turn_bank_weight "
                   "maximum_bank_target_deg magnetic_declination_deg "
                   "magnetic_inclination_deg magnetic_roll_correction_sec "
                   "magnetic_roll_weight magnetic_field_magnitude_tolerance "
                   "magnetic_roll_max_disagreement_deg magnetic_roll_min_geometry "
                   "min_ground_speed_mps baro_alt_filter_sec "
                   "baro_rate_filter_sec baro_gps_bias_sec baro_timeout_sec "
                   "g5_heading_offset_deg g5_time_offset_ms "
                   "sensor_pitch_offset_deg sensor_roll_offset_deg accel_input_scale");
    }
};
