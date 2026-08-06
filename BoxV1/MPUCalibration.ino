
// ============================================================
// MPU6050 CALIBRATION
// ============================================================

void calibrateGyro() {
  Serial.println();
  Serial.println("Calibrating MPU6050 gyro.");
  Serial.println("Do not move the box.");

  constexpr int sampleCount = 2000;
  double sumZ = 0.0;

  for (int i = 0; i < sampleCount; i++) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temperature;

    mpu.getEvent(&accel, &gyro, &temperature);

    sumZ += gyro.gyro.z;
    delay(2);
  }

  gyroBiasZRadS = static_cast<float>(sumZ / sampleCount);

  filteredGyroZRadS = 0.0f;
  rawGyroZRadS = 0.0f;
  boxRateRadS = 0.0f;
  boxAngleRad = 0.0f;

  Serial.print("Initial gyro Z bias: ");
  Serial.print(gyroBiasZRadS, 7);
  Serial.println(" rad/s");
}