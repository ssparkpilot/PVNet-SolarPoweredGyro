
// ============================================================
// BOX PID CONTROLLER
// ============================================================

float calculateBoxController(float dt) {
  // The target angle is zero because re-zero defines the current position
  // as the desired orientation.
  const float targetAngleRad = 0.0f;

  float error = targetAngleRad - boxAngleRad;

  integralError += error * dt;
  integralError = clampValue(
      integralError,
      -integralLimit,
      integralLimit
  );

  /*
    Gyro rate is used directly as the derivative measurement.

    Since target rate is zero:
      derivative error = 0 - measured rate

    This avoids differentiating a noisy integrated angle.
  */
  float proportionalTerm = boxKp * error;
  float integralTerm = boxKi * integralError;
  float derivativeTerm = boxKd * (-boxRateRadS);

  float output =
      proportionalTerm +
      integralTerm +
      derivativeTerm;

  output *= CONTROL_DIRECTION;

  return clampValue(
      output,
      -controlVoltageLimit,
      controlVoltageLimit
  );
}