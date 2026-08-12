
// ============================================================
// UTILITY FUNCTIONS
// ============================================================

float radiansToDegrees(float radians) {
  return radians * 180.0f / PI;
}

float clampValue(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }

  if (value > maximum) {
    return maximum;
  }

  return value;
}

void resetController() {
  integralError = 0.0f;
  previousError = 0.0f;
  commandedMotorVoltage = 0.0f;
}

void rezeroBox() {
  boxAngleRad = 0.0f;
  resetController();
}