#include <SimpleFOC.h>
#include <Wire.h>

// Motor: 7 pole pairs
BLDCMotor motor = BLDCMotor(7);

// SimpleFOCmini 3PWM driver pins
BLDCDriver3PWM driver = BLDCDriver3PWM(15, 2, 4, 5);

// AS5600 magnetic encoder over I2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// Serial command interface
Commander command = Commander(Serial);

// Target velocity in rad/s
float target_velocity = 5.0;

// Command: type T<number>
// Example: T10  -> 10 rad/s
// Example: T-5  -> reverse at 5 rad/s
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting SimpleFOC ESP32 velocity control...");

  // I2C pins for ESP32
  Wire.begin(21, 22);

  // Initialize encoder
  sensor.init();
  motor.linkSensor(&sensor);

  // Driver setup
  driver.voltage_power_supply = 12.0;  // Change if your motor supply is different
  driver.voltage_limit = 6.0;          // Start low for safety
  driver.init();

  motor.linkDriver(&driver);

  // Control mode
  motor.controller = MotionControlType::velocity;

  // FOC modulation
  motor.foc_modulation = FOCModulationType::SinePWM;

  // Velocity PID tuning - may need adjustment
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 2.0;
  motor.PID_velocity.D = 0.0;

  // Low-pass filter for velocity
  motor.LPF_velocity.Tf = 0.01;

  // Limits
  motor.voltage_limit = 6.0;
  motor.velocity_limit = 50.0;  // rad/s

  // Initialize motor
  motor.init();

  // Align encoder and start FOC
  motor.initFOC();

  // Serial command
  command.add('T', doTarget, "target velocity rad/s");

  Serial.println("Motor ready.");
  Serial.println("Use Serial Monitor commands like:");
  Serial.println("T5    -> 5 rad/s");
  Serial.println("T10   -> 10 rad/s");
  Serial.println("T-5   -> reverse");
  Serial.println("T0    -> stop");
}

void loop() {
  motor.loopFOC();

  motor.move(target_velocity);

  command.run();
}