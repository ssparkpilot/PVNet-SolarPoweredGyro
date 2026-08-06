
// ============================================================
// MOTOR SETUP
// ============================================================

void configureMotor() {
  motorSensor.init(&Wire);
  motor.linkSensor(&motorSensor);

  driver.voltage_power_supply = DRIVER_SUPPLY_VOLTAGE;
  driver.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  driver.pwm_frequency = 25000;
  driver.init();

  motor.linkDriver(&driver);

  /*
    Direct torque command is appropriate for a reaction wheel.

    In voltage torque mode, motor voltage approximates torque. A driver with
    current sensing would provide more accurate torque control.
  */
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motor.velocity_limit = MAX_WHEEL_SPEED_RAD_S;

  // Sensor filtering used internally by SimpleFOC.
  motor.LPF_velocity.Tf = 0.01f;

  // Velocity PID is not the main box controller, but these values may still
  // be used by some monitoring or configuration paths.
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 2.0f;
  motor.PID_velocity.D = 0.0f;

  motor.init();

  Serial.println("Starting FOC alignment.");
  Serial.println("Keep the reaction wheel clear.");

  int result = motor.initFOC();

  if (result == 0) {
    Serial.println("FOC initialization failed.");
    emergencyStop = true;
    motor.disable();
  } else {
    Serial.println("FOC initialized.");
    motor.move(0.0f);
  }
}