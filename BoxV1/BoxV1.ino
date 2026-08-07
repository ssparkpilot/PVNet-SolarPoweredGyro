/*
  ESP32 Reaction-Wheel Box Stabilizer
  -----------------------------------
  Hardware:
    - ESP32
    - SimpleFOCmini 3-PWM driver
    - BLDC motor with heavy reaction wheel
    - AS5600 magnetic motor encoder
    - MPU6050 IMU

  Control:
    - MPU6050 gyro Z estimates box rotation.
    - Outer PID generates motor torque/voltage.
    - Motor reaction torque rotates the box toward its zero orientation.
    - Web interface provides enable, disable, re-zero, gain adjustment,
      telemetry, and emergency stop.

  IMPORTANT:
    The MPU6050 cannot measure absolute yaw. Gyroscope integration will
    eventually drift. A magnetometer or other external reference is needed
    for indefinite absolute heading stabilization.
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SimpleFOC.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ============================================================
// USER CONFIGURATION
// ============================================================

// Leave these blank until router information is known.
const char* WIFI_SSID     = "PVNET Guest";
const char* WIFI_PASSWORD = "Promenade";

// Choose an unused address inside your router's subnet.
// Example only: 192.168.1.80
IPAddress LOCAL_IP(10, 0, 2, 45);

// Enter your router information.
IPAddress GATEWAY(10, 0, 2, 1);
IPAddress SUBNET(255, 255, 255, 0);
IPAddress PRIMARY_DNS(1, 1, 1, 3);
IPAddress SECONDARY_DNS(1, 0, 0, 3);

// Verify this for your motor.
constexpr int MOTOR_POLE_PAIRS = 7;

// SimpleFOCmini pins
constexpr int MOTOR_IN1_PIN = 15;
constexpr int MOTOR_IN2_PIN = 2;
constexpr int MOTOR_IN3_PIN = 4;
constexpr int MOTOR_EN_PIN  = 5;

// I2C pins
constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;

// Motor supply voltage.
// Set this to the actual motor-driver supply voltage.
constexpr float DRIVER_SUPPLY_VOLTAGE = 12.0f;

// Maximum voltage that the controller may apply to the motor.
// Start low during testing.
constexpr float MOTOR_VOLTAGE_LIMIT = 2.0f;

// Reaction-wheel overspeed limit.
constexpr float MAX_WHEEL_SPEED_RAD_S = 100.0f;

// Change this to -1 if the box moves in the wrong direction.
constexpr float CONTROL_DIRECTION = 1.0f;

// Set false if the MPU6050 Z axis has the opposite sign.
constexpr float GYRO_DIRECTION = 1.0f;

// ============================================================
// DEVICES
// ============================================================

Adafruit_MPU6050 mpu;
WebServer server(80);

// AS5600:
// address = 0x36
// resolution = 12 bits
// angle register = 0x0E
// bits used in MSB = 4
MagneticSensorI2C motorSensor =
    MagneticSensorI2C(0x36, 12, 0x0E, 4);

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

BLDCDriver3PWM driver = BLDCDriver3PWM(
    MOTOR_IN1_PIN,
    MOTOR_IN2_PIN,
    MOTOR_IN3_PIN,
    MOTOR_EN_PIN
);

// ============================================================
// CONTROL VARIABLES
// ============================================================

volatile bool stabilizationEnabled = false;
volatile bool emergencyStop = false;

// Estimated box state
float boxAngleRad = 0.0f;
float boxRateRadS = 0.0f;

// Raw and estimated gyro values
float rawGyroZRadS = 0.0f;
float gyroBiasZRadS = 0.0f;

// Filtered gyro value
float filteredGyroZRadS = 0.0f;

// PID gains for box angular position
float boxKp = 1.8f;
float boxKi = 0.08f;
float boxKd = 0.35f;

// PID state
float integralError = 0.0f;
float previousError = 0.0f;
float commandedMotorVoltage = 0.0f;

// Limits
float integralLimit = 1.5f;
float controlVoltageLimit = MOTOR_VOLTAGE_LIMIT;

// Timing
uint32_t previousControlMicros = 0;
uint32_t previousImuMicros = 0;
uint32_t previousTelemetryMillis = 0;

constexpr uint32_t CONTROL_PERIOD_US = 2000;  // 500 Hz
constexpr uint32_t IMU_PERIOD_US = 2000;      // 500 Hz

// Gyro filter. Larger value = more responsive but noisier.
constexpr float GYRO_FILTER_ALPHA = 0.20f;

// Adaptive bias parameters
constexpr float BIAS_LEARNING_RATE = 0.0008f;
constexpr float STATIONARY_RATE_THRESHOLD = 0.025f;
constexpr float STATIONARY_CONTROL_THRESHOLD = 0.15f;

// Re-zero request flag
volatile bool rezeroRequested = false;

// ============================================================
// WEB PAGE
// ============================================================

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Reaction Wheel Stabilizer</title>

  <style>
    body {
      font-family: Arial, sans-serif;
      background: #111827;
      color: #f3f4f6;
      max-width: 850px;
      margin: auto;
      padding: 20px;
    }

    .card {
      background: #1f2937;
      border-radius: 12px;
      padding: 18px;
      margin-bottom: 16px;
    }

    h1, h2 {
      margin-top: 0;
    }

    .value {
      font-size: 2rem;
      font-weight: bold;
    }

    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(190px, 1fr));
      gap: 12px;
    }

    button {
      border: none;
      border-radius: 8px;
      padding: 12px 18px;
      margin: 5px;
      font-size: 1rem;
      cursor: pointer;
    }

    .enable {
      background: #10b981;
      color: white;
    }

    .disable {
      background: #f59e0b;
      color: white;
    }

    .zero {
      background: #3b82f6;
      color: white;
    }

    .stop {
      background: #dc2626;
      color: white;
    }

    input {
      width: 100%;
      box-sizing: border-box;
      padding: 10px;
      margin-top: 5px;
      margin-bottom: 12px;
      border-radius: 6px;
      border: 1px solid #4b5563;
      background: #111827;
      color: white;
    }

    .status-enabled {
      color: #34d399;
    }

    .status-disabled {
      color: #fbbf24;
    }

    .status-stop {
      color: #f87171;
    }

    .angle-bar {
      position: relative;
      height: 28px;
      background: #374151;
      border-radius: 14px;
      overflow: hidden;
    }

    .angle-marker {
      position: absolute;
      width: 8px;
      height: 28px;
      background: #60a5fa;
      left: 50%;
      transform: translateX(-50%);
    }

    .labels {
      display: flex;
      justify-content: space-between;
      font-size: 0.8rem;
      color: #9ca3af;
      margin-top: 4px;
    }
  </style>
</head>

<body>
  <h1>Reaction Wheel Stabilizer</h1>

  <div class="card">
    <div id="status" class="value">Connecting...</div>

    <button class="enable" onclick="sendCommand('/api/enable')">
      Enable
    </button>

    <button class="disable" onclick="sendCommand('/api/disable')">
      Disable
    </button>

    <button class="zero" onclick="sendCommand('/api/rezero')">
      Re-zero Box
    </button>

    <button class="stop" onclick="sendCommand('/api/stop')">
      Emergency Stop
    </button>

    <button onclick="sendCommand('/api/clearstop')">
      Clear Stop
    </button>
  </div>

  <div class="grid">
    <div class="card">
      <h2>Box angle</h2>
      <div class="value"><span id="angle">0.00</span>&deg;</div>
    </div>

    <div class="card">
      <h2>Box angular rate</h2>
      <div class="value"><span id="rate">0.00</span>&deg;/s</div>
    </div>

    <div class="card">
      <h2>Wheel speed</h2>
      <div class="value"><span id="wheel">0.00</span> rad/s</div>
    </div>

    <div class="card">
      <h2>Motor command</h2>
      <div class="value"><span id="voltage">0.00</span> V</div>
    </div>
  </div>

  <div class="card">
    <h2>Orientation scale</h2>

    <div class="angle-bar">
      <div id="marker" class="angle-marker"></div>
    </div>

    <div class="labels">
      <span>-180&deg;</span>
      <span>0&deg;</span>
      <span>+180&deg;</span>
    </div>
  </div>

  <div class="card">
    <h2>Controller tuning</h2>

    <label>Kp</label>
    <input id="kp" type="number" step="0.01">

    <label>Ki</label>
    <input id="ki" type="number" step="0.001">

    <label>Kd</label>
    <input id="kd" type="number" step="0.01">

    <label>Maximum motor voltage</label>
    <input id="limit" type="number" step="0.1">

    <button class="zero" onclick="updateGains()">
      Apply Settings
    </button>
  </div>

<script>
  let firstUpdate = true;

  async function sendCommand(path) {
    try {
      await fetch(path, {method: 'POST'});
      await updateStatus();
    } catch (error) {
      console.error(error);
    }
  }

  async function updateGains() {
    const params = new URLSearchParams({
      kp: document.getElementById('kp').value,
      ki: document.getElementById('ki').value,
      kd: document.getElementById('kd').value,
      limit: document.getElementById('limit').value
    });

    await fetch('/api/settings?' + params.toString(), {
      method: 'POST'
    });
  }

  async function updateStatus() {
    try {
      const response = await fetch('/api/status');
      const data = await response.json();

      document.getElementById('angle').textContent =
        data.angle_deg.toFixed(2);

      document.getElementById('rate').textContent =
        data.rate_deg_s.toFixed(2);

      document.getElementById('wheel').textContent =
        data.wheel_rad_s.toFixed(2);

      document.getElementById('voltage').textContent =
        data.motor_voltage.toFixed(2);

      const status = document.getElementById('status');

      if (data.emergency_stop) {
        status.textContent = 'EMERGENCY STOP';
        status.className = 'value status-stop';
      } else if (data.enabled) {
        status.textContent = 'Stabilization enabled';
        status.className = 'value status-enabled';
      } else {
        status.textContent = 'Stabilization disabled';
        status.className = 'value status-disabled';
      }

      const constrainedAngle =
        Math.max(-180, Math.min(180, data.angle_deg));

      const markerPercent =
        ((constrainedAngle + 180) / 360) * 100;

      document.getElementById('marker').style.left =
        markerPercent + '%';

      if (firstUpdate) {
        document.getElementById('kp').value = data.kp;
        document.getElementById('ki').value = data.ki;
        document.getElementById('kd').value = data.kd;
        document.getElementById('limit').value = data.limit;
        firstUpdate = false;
      }
    } catch (error) {
      document.getElementById('status').textContent =
        'Connection lost';
    }
  }

  setInterval(updateStatus, 200);
  updateStatus();
</script>
</body>
</html>
)rawliteral";

// ============================================================
// ARDUINO SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Reaction-wheel stabilizer starting.");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 was not found.");

    while (true) {
      delay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  calibrateGyro();
  configureMotor();
  connectWiFi();
  configureWebServer();

  previousControlMicros = micros();
  previousImuMicros = previousControlMicros;

  Serial.println("System ready.");
  Serial.println("Stabilization starts disabled.");
}

// ============================================================
// ARDUINO LOOP
// ============================================================

void loop() {
  /*
    These must run as frequently as possible. Do not add long delays here.
  */
  motor.loopFOC();

  uint32_t nowMicros = micros();

  if (
      static_cast<uint32_t>(
          nowMicros - previousImuMicros
      ) >= IMU_PERIOD_US
  ) {
    float dt =
        static_cast<float>(
            nowMicros - previousImuMicros
        ) * 1.0e-6f;

    previousImuMicros = nowMicros;

    // Reject abnormal timing caused by startup or blocking operations.
    dt = clampValue(dt, 0.0005f, 0.02f);

    updateImu(dt);
  }

  if (
      static_cast<uint32_t>(
          nowMicros - previousControlMicros
      ) >= CONTROL_PERIOD_US
  ) {
    float dt =
        static_cast<float>(
            nowMicros - previousControlMicros
        ) * 1.0e-6f;

    previousControlMicros = nowMicros;

    dt = clampValue(dt, 0.0005f, 0.02f);

    runStabilizationController(dt);
  }

  server.handleClient();

  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t previousReconnectMillis = 0;

    if (millis() - previousReconnectMillis > 5000) {
      previousReconnectMillis = millis();
      WiFi.reconnect();
    }
  }

  if (millis() - previousTelemetryMillis >= 500) {
    previousTelemetryMillis = millis();

    Serial.print("Angle: ");
    Serial.print(radiansToDegrees(boxAngleRad), 2);

    Serial.print(" deg, rate: ");
    Serial.print(radiansToDegrees(boxRateRadS), 2);

    Serial.print(" deg/s, wheel: ");
    Serial.print(motor.shaft_velocity, 2);

    Serial.print(" rad/s, command: ");
    Serial.print(commandedMotorVoltage, 2);

    Serial.println(" V");
  }
}