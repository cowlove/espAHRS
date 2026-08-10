#pragma once

#include <math.h>

// Pitch-constrained magnetic-vector bank observation.
//
// This header deliberately contains only the geomagnetic-dip geometry that is
// intended to become the core distinguishing observation in DipAHRS.  It does
// not calibrate/remap the magnetometer, propagate attitude, apply correction
// gains, fuse multiple compasses, or use GPS/air data.  Callers supply one
// calibrated magnetic vector in aircraft body axes and a trusted pitch.
class DipAHRS {
public:
    struct Config {
        // Local field direction in the navigation frame.  Declination is
        // east-positive; inclination/dip is down-positive in NED.
        float magneticDeclinationDeg = 0.0f;
        float magneticInclinationDeg = 0.0f;

        // Reject solutions whose magnetic vector changes too little under a
        // roll rotation.  The normalized geometry is in [0, 1].
        float minimumRollGeometry = 0.25f;

        // Ellipsoid calibration and sensor noise can move a normalized vector
        // slightly outside the exact trigonometric solution.  This margin
        // permits a small clamp while rejecting grossly inconsistent inputs.
        float cosineTolerance = 0.15f;
    };

    struct Observation {
        float rollDeg = 0.0f;
        float headingDeg = 0.0f;
        float rollGeometry = 0.0f;
        float inputMagnitude = 0.0f;
        unsigned char branch = 0;
    };

    // Solve the two attitudes that map the configured Earth magnetic-field
    // vector to the measured body vector at the supplied pitch, then choose
    // the solution nearest the prior continuous attitude.
    //
    // Body axes are aircraft x-forward, y-right, z-down.  The magnetic vector
    // may have any magnitude but must already be calibrated and frame-aligned.
    static bool observe(float bodyX, float bodyY, float bodyZ,
                        float trustedPitchDeg,
                        float priorRollDeg,
                        float priorHeadingDeg,
                        const Config &config,
                        Observation &observation) {
        const float magnitude = sqrtf(bodyX * bodyX + bodyY * bodyY +
                                      bodyZ * bodyZ);
        if (magnitude < 1.0e-6f) return false;
        bodyX /= magnitude;
        bodyY /= magnitude;
        bodyZ /= magnitude;

        const float pitch = trustedPitchDeg * kDegToRad;
        const float declination = config.magneticDeclinationDeg * kDegToRad;
        const float inclination = config.magneticInclinationDeg * kDegToRad;
        const float horizontal = cosf(inclination);
        const float down = sinf(inclination);
        const float denominator = cosf(pitch) * horizontal;
        if (fabsf(denominator) < 1.0e-4f) return false;

        float cosine = (bodyX + sinf(pitch) * down) / denominator;
        const float tolerance = fmaxf(0.0f, config.cosineTolerance);
        if (cosine < -1.0f - tolerance || cosine > 1.0f + tolerance) {
            return false;
        }
        cosine = fmaxf(-1.0f, fminf(1.0f, cosine));
        const float delta = acosf(cosine);

        bool haveCandidate = false;
        float bestScore = 0.0f;
        for (unsigned char branch = 0; branch < 2; ++branch) {
            const float heading = declination + (branch == 0 ? delta : -delta);
            const float horizontalAlongHeading =
                horizontal * cosf(heading - declination);
            const float yBeforeRoll =
                horizontal * sinf(declination - heading);
            const float zBeforeRoll =
                sinf(pitch) * horizontalAlongHeading + cosf(pitch) * down;
            const float geometry =
                sqrtf(yBeforeRoll * yBeforeRoll + zBeforeRoll * zBeforeRoll);
            if (geometry < config.minimumRollGeometry) continue;

            const float roll = atan2f(bodyY * zBeforeRoll -
                                          bodyZ * yBeforeRoll,
                                      bodyY * yBeforeRoll +
                                          bodyZ * zBeforeRoll);
            const float candidateRollDeg = wrap180(roll * kRadToDeg);
            const float candidateHeadingDeg = wrap360(heading * kRadToDeg);

            // Pitch plus one 3-D reference vector leaves two discrete
            // solutions.  Continuity selects the physical branch without GPS.
            const float score =
                fabsf(wrap180(candidateRollDeg - priorRollDeg)) +
                fabsf(wrap180(candidateHeadingDeg - priorHeadingDeg));
            if (!haveCandidate || score < bestScore) {
                bestScore = score;
                observation.rollDeg = candidateRollDeg;
                observation.headingDeg = candidateHeadingDeg;
                observation.rollGeometry = geometry;
                observation.inputMagnitude = magnitude;
                observation.branch = branch;
                haveCandidate = true;
            }
        }
        return haveCandidate;
    }

private:
    static constexpr float kDegToRad = 0.01745329251994329577f;
    static constexpr float kRadToDeg = 57.29577951308232088f;

    static float wrap180(float degrees) {
        while (degrees > 180.0f) degrees -= 360.0f;
        while (degrees <= -180.0f) degrees += 360.0f;
        return degrees;
    }

    static float wrap360(float degrees) {
        while (degrees >= 360.0f) degrees -= 360.0f;
        while (degrees < 0.0f) degrees += 360.0f;
        return degrees;
    }
};
