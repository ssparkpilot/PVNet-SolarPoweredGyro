
// ============================================================
// CONTROL LOOP
// ============================================================

void runStabilizationController(float dt) {
  float wheelSpeed = motor.shaft_velocity;

  bool wheelOverspeed =
      fabsf(wheelSpeed) >= MAX_WHEEL_SPEED_RAD_S;

  if (!stabilizationEnabled || emergencyStop || wheelOverspeed) {
    commandedMotorVoltage = 0.0f;
    motor.move(0.0f);
    resetController();

    if (wheelOverspeed) {
      stabilizationEnabled = false;
      emergencyStop = true;

      Serial.println(
          "Emergency stop: reaction wheel overspeed."
      );
    }

    return;
  }

  commandedMotorVoltage = calculateBoxController(dt);
  motor.move(commandedMotorVoltage);
}
