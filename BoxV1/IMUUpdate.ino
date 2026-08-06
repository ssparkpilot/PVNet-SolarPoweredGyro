
// ============================================================
// IMU UPDATE
// ============================================================

void updateImu(float dt) {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temperature;

  mpu.getEvent(&accel, &gyro, &temperature);

  rawGyroZRadS = GYRO_DIRECTION * gyro.gyro.z;

  float correctedRate = rawGyroZRadS - gyroBiasZRadS;

  filteredGyroZRadS +=
      GYRO_FILTER_ALPHA *
      (correctedRate - filteredGyroZRadS);

  boxRateRadS = filteredGyroZRadS;

  /*
    Adaptive gyro-bias estimation:

    When the box is nearly stationary and motor torque is low, the measured
    rotation is assumed to be residual gyro bias. The estimator then adjusts
    the bias very slowly.

    This helps temperature-related drift but does not create an absolute yaw
    reference.
  */
  bool probablyStationary =
      fabsf(correctedRate) < STATIONARY_RATE_THRESHOLD &&
      fabsf(commandedMotorVoltage) < STATIONARY_CONTROL_THRESHOLD;

  if (probablyStationary) {
    gyroBiasZRadS +=
        BIAS_LEARNING_RATE *
        (rawGyroZRadS - gyroBiasZRadS);
  }

  // Integrate angular velocity into relative angular position.
  boxAngleRad += boxRateRadS * dt;

  if (rezeroRequested) {
    rezeroRequested = false;
    rezeroBox();
  }
}