#include <SimpleFOC.h>

// -----------------------------
// ESP32 pins - change to match your wiring
// -----------------------------
#define IN1_PIN 15
#define IN2_PIN 2
#define IN3_PIN 4
#define EN_PIN 5

// -----------------------------
// Motor and driver setup
// -----------------------------
// 7 pole-pair BLDC motor
BLDCMotor motor = BLDCMotor(7);

// SimpleFOCmini: IN1, IN2, IN3, EN
BLDCDriver3PWM driver = BLDCDriver3PWM(IN1_PIN, IN2_PIN, IN3_PIN, EN_PIN);

// Target velocity in rad/s
float target_velocity = 0.0;

// Commander serial interface
Commander command = Commander(Serial);

// Serial command: T<number>
// Example: T5 sets 5 rad/s
// Example: T-5 reverses direction
// Example: T0 stops
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}

// Optional command to change voltage limit from serial
// Example: L3 sets motor voltage limit to 3V
void doVoltageLimit(char* cmd) {
  command.scalar(&motor.voltage_limit, cmd);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 + SimpleFOCmini open-loop velocity control");
  Serial.println("NO encoder required.");

  // -----------------------------
  // Driver setup
  // -----------------------------
  driver.voltage_power_supply = 12.0;  // Your motor supply voltage
  driver.voltage_limit = 6.0;          // Max driver output voltage
  driver.pwm_frequency = 25000;        // Good starting value for ESP32

  if (!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }

  // Link motor and driver
  motor.linkDriver(&driver);

  // -----------------------------
  // Open-loop velocity mode
  // -----------------------------
  motor.controller = MotionControlType::velocity_openloop;

  // Start low for safety
  motor.voltage_limit = 2.0;    // Motor voltage limit in volts
  motor.velocity_limit = 20.0;  // Max target velocity in rad/s

  // Initialize motor
  if (!motor.init()) {
    Serial.println("Motor init failed!");
    return;
  }

  // Serial commands
  command.add('T', doTarget, "target velocity rad/s");
  command.add('L', doVoltageLimit, "motor voltage limit volts");

  Serial.println("Motor ready.");
  Serial.println("Commands:");
  Serial.println("  T2    -> 2 rad/s");
  Serial.println("  T10   -> 10 rad/s");
  Serial.println("  T-5   -> reverse at -5 rad/s");
  Serial.println("  T0    -> stop");
  Serial.println("  L3    -> set motor voltage limit to 3V");
  Serial.println();
}

void loop() {
  // In open-loop mode, this can still be called.
  // It will not use an encoder because none is linked.
  motor.loopFOC();

  // Spin at the commanded open-loop velocity
  motor.move(target_velocity);

  // Handle serial commands
  command.run();
}