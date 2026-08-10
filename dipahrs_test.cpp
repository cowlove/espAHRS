#include "DipAHRS.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

namespace {
constexpr float kDegToRad = 0.01745329251994329577f;

float angleError(float actual, float expected) {
    float error = actual - expected;
    while (error > 180.0f) error -= 360.0f;
    while (error <= -180.0f) error += 360.0f;
    return fabsf(error);
}

void synthesizeBodyField(float declinationDeg, float inclinationDeg,
                         float rollDeg, float pitchDeg, float headingDeg,
                         float &x, float &y, float &z) {
    const float declination = declinationDeg * kDegToRad;
    const float inclination = inclinationDeg * kDegToRad;
    const float roll = rollDeg * kDegToRad;
    const float pitch = pitchDeg * kDegToRad;
    const float heading = headingDeg * kDegToRad;
    const float horizontal = cosf(inclination);
    const float down = sinf(inclination);
    const float horizontalAlongHeading =
        horizontal * cosf(heading - declination);
    const float yBeforeRoll = horizontal * sinf(declination - heading);
    const float zBeforeRoll =
        sinf(pitch) * horizontalAlongHeading + cosf(pitch) * down;
    x = cosf(pitch) * horizontalAlongHeading - sinf(pitch) * down;
    y = cosf(roll) * yBeforeRoll + sinf(roll) * zBeforeRoll;
    z = -sinf(roll) * yBeforeRoll + cosf(roll) * zBeforeRoll;
}

void checkKnownAttitude(float rollDeg, float pitchDeg, float headingDeg) {
    DipAHRS::Config config;
    config.magneticDeclinationDeg = 14.89224f;
    config.magneticInclinationDeg = 68.75569f;
    float x, y, z;
    synthesizeBodyField(config.magneticDeclinationDeg,
                        config.magneticInclinationDeg,
                        rollDeg, pitchDeg, headingDeg, x, y, z);
    DipAHRS::Observation observation;
    assert(DipAHRS::observe(x, y, z, pitchDeg, rollDeg, headingDeg,
                            config, observation));
    assert(angleError(observation.rollDeg, rollDeg) < 0.001f);
    assert(angleError(observation.headingDeg, headingDeg) < 0.001f);
    assert(observation.rollGeometry >= config.minimumRollGeometry);
    assert(fabsf(observation.inputMagnitude - 1.0f) < 0.001f);
}
}

int main() {
    checkKnownAttitude(0.0f, 0.0f, 0.0f);
    checkKnownAttitude(25.0f, 8.0f, 42.0f);
    checkKnownAttitude(-32.0f, -6.0f, 315.0f);

    DipAHRS::Config config;
    DipAHRS::Observation observation;
    assert(!DipAHRS::observe(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                             config, observation));
    puts("DipAHRS geometry tests passed");
    return 0;
}
