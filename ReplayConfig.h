#pragma once

#include "AircraftAHRS.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

// One central replay configuration surface.  Sweep tools can pass repeated
// --param name=value arguments without teaching the replay loop about each
// individual AHRS subsystem.
struct ReplayConfig {
    AircraftAHRS::Config ahrs;
    float g5HeadingOffsetDeg = 0.0f;

    bool set(const char *name, float value) {
        struct Field { const char *name; float *value; } fields[] = {
            {"yaw_correction_sec", &ahrs.yawCorrectionTimeSec},
            {"roll_correction_sec", &ahrs.rollCorrectionTimeSec},
            {"pitch_correction_sec", &ahrs.pitchCorrectionTimeSec},
            {"gps_derivative_sec", &ahrs.gpsDerivativeTimeSec},
            {"angle_of_attack_deg", &ahrs.angleOfAttackDeg},
            {"gps_timeout_sec", &ahrs.gpsTimeoutSec},
            {"accel_correction_sec", &ahrs.accelCorrectionTimeSec},
            {"accel_filter_sec", &ahrs.accelFilterTimeSec},
            {"accel_tolerance_mps2", &ahrs.accelMagnitudeToleranceMps2},
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
        return false;
    }

    static void list() {
        std::puts("tunable parameters: yaw_correction_sec roll_correction_sec "
                   "pitch_correction_sec gps_derivative_sec angle_of_attack_deg "
                   "gps_timeout_sec accel_correction_sec accel_filter_sec "
                   "accel_tolerance_mps2 min_ground_speed_mps baro_alt_filter_sec "
                   "baro_rate_filter_sec baro_gps_bias_sec baro_timeout_sec "
                   "g5_heading_offset_deg");
    }
};
