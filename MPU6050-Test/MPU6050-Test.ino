#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

TwoWire MPUWire = TwoWire(1);
Adafruit_MPU6050 mpu;

#define MPU_SDA 21
#define MPU_SCL 22

void setup() {
  Serial.begin(115200);
  delay(1000);

  MPUWire.begin(MPU_SDA, MPU_SCL, 400000);

  if (!mpu.begin(0x68, &MPUWire)) {
    Serial.println("MPU6050 not found");
    while (1) delay(10);
  }

  Serial.println("MPU6050 connected");
}

void loop() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  Serial.print("Gyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" | Gyro Y: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" | Gyro Z: ");
  Serial.println(gyro.gyro.z);

  delay(100);
}