# Flight-test follow-up

## T-Beam steep-turn pitch growth

The current T-Beam replay calibration is promising for steady flight and
straight climbs: the first run reaches approximately 0.46° roll MAE and 0.49°
pitch MAE after applying the separate sensor remaps, pitch-gyro sign fix,
fine offsets, and gyro biases.

Run 3 (`fusion-7854.bin`) is the primary steep-turn reproduction case. Its
pitch error grows substantially while the final pitch correction target remains
near the expected zero-degree reference. This suggests that the correction
target is not the source of the error. The leading hypothesis is an improperly
correlated or rotated gyro component—possibly turn-rate cross-coupling entering
the pitch integration during steep turns.

Follow-up investigation:

- Compare raw sensor gyro axes and aircraft-frame rates during the steep-turn
  intervals, especially the yaw/roll components projected into pitch.
- Verify the full body-rate-to-Euler-rate transform and its sign convention
  near large bank angles.
- Check whether the current independent gyro-axis remap is physically
  consistent with the accelerometer remap and mounting rotation.
- Separate gyro integration error from GPS/accelerometer correction behavior
  using run 3 as the regression log.

Do not retune the static offsets or biases from run 3 alone until this dynamic
cross-coupling issue is understood.
