#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

#include <SimpleFOC.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ============================================================
// USER SETTINGS
// ============================================================

// WI-FI configuration
const char* WIFI_SSID     = "PVNET Guest";
const char* WIFI_PASSWORD = "Promenade";

// Fill these in using values appropriate for your network.
//
// Example only:
// ESP32 IP:  192.168.1.50
// Gateway:   192.168.1.1
// Subnet:    255.255.255.0
//
// Leave the values below blank until you know your router's
// address range and subnet configuration.

const char* STATIC_IP_TEXT = "10.0.2.45";
const char* GATEWAY_TEXT   = "10.0.2.1";
const char* SUBNET_TEXT    = "255.255.255.0";
const char* PRIMARY_DNS_TEXT   = "1.1.1.3";
const char* SECONDARY_DNS_TEXT = "1.0.0.3";

// Motor configuration
constexpr int MOTOR_POLE_PAIRS = 7;

// SimpleFOCMini pins
constexpr int PIN_IN1 = 15;
constexpr int PIN_IN2 = 2;
constexpr int PIN_IN3 = 4;
constexpr int PIN_EN  = 5;

// Shared I2C bus for AS5600 and MPU6050
constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;

// Motor and power settings
constexpr float MOTOR_SUPPLY_VOLTAGE = 12.0f;

// Begin conservatively and increase only if necessary.
constexpr float MOTOR_VOLTAGE_LIMIT = 4.0f;

// Maximum speed before applying the browser multiplier.
constexpr float BASE_MAX_SPEED = 20.0f;

// Browser multiplier limits
constexpr float MIN_MULTIPLIER = 0.0f;
constexpr float MAX_MULTIPLIER = 2.0f;

// MPU6050 gyro integration
constexpr uint32_t IMU_UPDATE_INTERVAL_US = 2000;  // 500 Hz target

// ============================================================
// OBJECTS
// ============================================================

TwoWire sensorBus = TwoWire(0);

MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(
  PIN_IN1,
  PIN_IN2,
  PIN_IN3,
  PIN_EN
);

Adafruit_MPU6050 mpu;
WebServer server(80);

// ============================================================
// SHARED STATE
// ============================================================

float imuAngleDegrees = 0.0f;
float gyroZBias = 0.0f;

float speedMultiplier = 1.0f;
float mappedBaseSpeed = 0.0f;
float targetMotorSpeed = 0.0f;

uint32_t previousImuTimeUs = 0;
uint32_t lastSerialPrintMs = 0;

bool motorReady = false;
bool imuReady = false;

// ============================================================
// WEB PAGE
// ============================================================

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport"
        content="width=device-width, initial-scale=1.0">

  <title>ESP32 Gyroscopic Motor Control</title>

  <style>
    :root {
      color-scheme: dark;
      font-family: Arial, Helvetica, sans-serif;
    }

    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      min-height: 100vh;
      background: #10141c;
      color: #f3f6fa;
      display: flex;
      justify-content: center;
      padding: 24px;
    }

    .container {
      width: 100%;
      max-width: 720px;
    }

    h1 {
      margin-top: 0;
      text-align: center;
      font-size: 1.8rem;
    }

    .card {
      background: #1b2230;
      border: 1px solid #303b4e;
      border-radius: 16px;
      padding: 22px;
      margin-bottom: 18px;
      box-shadow: 0 8px 25px rgba(0,0,0,0.25);
    }

    .angle-value {
      text-align: center;
      font-size: 3.5rem;
      font-weight: bold;
      margin: 12px 0;
    }

    .dial {
      width: 230px;
      height: 230px;
      margin: 20px auto;
      border-radius: 50%;
      border: 10px solid #343e50;
      background:
        radial-gradient(circle at center,
          #232c3b 0%,
          #18202d 68%,
          #121822 100%);
      position: relative;
    }

    .dial::after {
      content: "";
      width: 18px;
      height: 18px;
      background: white;
      border-radius: 50%;
      position: absolute;
      left: 50%;
      top: 50%;
      transform: translate(-50%, -50%);
    }

    .needle {
      width: 5px;
      height: 90px;
      background: #57b7ff;
      border-radius: 5px;
      position: absolute;
      left: calc(50% - 2.5px);
      bottom: 50%;
      transform-origin: bottom center;
      transform: rotate(0deg);
      transition: transform 0.08s linear;
    }

    .marks {
      position: absolute;
      inset: 0;
      pointer-events: none;
    }

    .mark {
      position: absolute;
      font-size: 0.85rem;
      color: #bfc8d6;
    }

    .mark-0 {
      top: 6px;
      left: 50%;
      transform: translateX(-50%);
    }

    .mark-90 {
      right: 8px;
      top: 50%;
      transform: translateY(-50%);
    }

    .mark-180 {
      bottom: 6px;
      left: 50%;
      transform: translateX(-50%);
    }

    .mark-270 {
      left: 8px;
      top: 50%;
      transform: translateY(-50%);
    }

    label {
      display: block;
      margin-bottom: 10px;
      font-weight: bold;
    }

    input[type="range"] {
      width: 100%;
    }

    input[type="number"] {
      width: 110px;
      background: #121822;
      color: white;
      border: 1px solid #46536a;
      border-radius: 8px;
      padding: 9px;
      font-size: 1rem;
    }

    button {
      display: block;
      width: 100%;
      border: none;
      border-radius: 10px;
      background: #2789d8;
      color: white;
      padding: 13px 18px;
      margin-top: 16px;
      font-size: 1rem;
      font-weight: bold;
      cursor: pointer;
    }

    button:hover {
      background: #329ce9;
    }

    button.stop {
      background: #c84545;
    }

    button.stop:hover {
      background: #df5252;
    }

    .row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 12px;
      margin: 12px 0;
    }

    .data-name {
      color: #b8c2d2;
    }

    .data-value {
      font-weight: bold;
    }

    .status {
      text-align: center;
      color: #9ca8ba;
      min-height: 20px;
      margin-top: 12px;
    }

    .formula {
      color: #b9c3d2;
      line-height: 1.5;
      font-size: 0.95rem;
    }
  </style>
</head>

<body>
<div class="container">
  <h1>Gyroscopic Motor Control</h1>

  <div class="card">
    <div class="angle-value">
      <span id="angle">0.0</span>&deg;
    </div>

    <div class="dial">
      <div id="needle" class="needle"></div>

      <div class="marks">
        <span class="mark mark-0">0&deg;</span>
        <span class="mark mark-90">90&deg;</span>
        <span class="mark mark-180">180&deg;</span>
        <span class="mark mark-270">270&deg;</span>
      </div>
    </div>

    <button onclick="zeroImu()">Re-zero IMU</button>
  </div>

  <div class="card">
    <label for="multiplier">
      Speed multiplier:
      <span id="multiplierText">1.00</span>&times;
    </label>

    <input
      id="multiplier"
      type="range"
      min="0"
      max="2"
      value="1"
      step="0.05"
      oninput="multiplierChanged(this.value)"
    >

    <div class="row">
      <span class="data-name">Exact value</span>
      <input
        id="multiplierNumber"
        type="number"
        min="0"
        max="2"
        value="1"
        step="0.05"
        onchange="numberChanged(this.value)"
      >
    </div>

    <button class="stop" onclick="stopMotor()">
      Stop Motor
    </button>
  </div>

  <div class="card">
    <div class="row">
      <span class="data-name">Mapped base speed</span>
      <span class="data-value">
        <span id="baseSpeed">0.00</span> rad/s
      </span>
    </div>

    <div class="row">
      <span class="data-name">Commanded speed</span>
      <span class="data-value">
        <span id="targetSpeed">0.00</span> rad/s
      </span>
    </div>

    <div class="row">
      <span class="data-name">Measured motor speed</span>
      <span class="data-value">
        <span id="measuredSpeed">0.00</span> rad/s
      </span>
    </div>

    <div class="formula">
      0&deg; → 0 rad/s<br>
      180&deg; → 20 rad/s<br>
      360&deg; → 0 rad/s<br><br>
      Final command = mapped speed × multiplier
    </div>

    <div id="status" class="status"></div>
  </div>
</div>

<script>
  let updateTimer = null;
  let multiplierTimer = null;

  function setStatus(message) {
    document.getElementById("status").textContent = message;

    window.setTimeout(() => {
      if (document.getElementById("status").textContent === message) {
        document.getElementById("status").textContent = "";
      }
    }, 1800);
  }

  function updateDisplay(data) {
    const angle = Number(data.angle);
    const multiplier = Number(data.multiplier);

    document.getElementById("angle").textContent =
      angle.toFixed(1);

    document.getElementById("needle").style.transform =
      `rotate(${angle}deg)`;

    document.getElementById("baseSpeed").textContent =
      Number(data.baseSpeed).toFixed(2);

    document.getElementById("targetSpeed").textContent =
      Number(data.targetSpeed).toFixed(2);

    document.getElementById("measuredSpeed").textContent =
      Number(data.measuredSpeed).toFixed(2);

    if (document.activeElement.id !== "multiplier" &&
        document.activeElement.id !== "multiplierNumber") {
      document.getElementById("multiplier").value = multiplier;
      document.getElementById("multiplierNumber").value =
        multiplier.toFixed(2);
      document.getElementById("multiplierText").textContent =
        multiplier.toFixed(2);
    }
  }

  async function getStatus() {
    try {
      const response = await fetch("/status", {
        cache: "no-store"
      });

      if (!response.ok) {
        throw new Error("HTTP error");
      }

      const data = await response.json();
      updateDisplay(data);
    } catch (error) {
      document.getElementById("status").textContent =
        "Connection to ESP32 lost";
    }
  }

  function multiplierChanged(value) {
    const number = Number(value);

    document.getElementById("multiplierText").textContent =
      number.toFixed(2);

    document.getElementById("multiplierNumber").value =
      number.toFixed(2);

    clearTimeout(multiplierTimer);

    multiplierTimer = setTimeout(() => {
      setMultiplier(number);
    }, 120);
  }

  function numberChanged(value) {
    let number = Number(value);

    if (!Number.isFinite(number)) {
      number = 1.0;
    }

    number = Math.max(0, Math.min(2, number));

    document.getElementById("multiplier").value = number;
    document.getElementById("multiplierNumber").value =
      number.toFixed(2);

    document.getElementById("multiplierText").textContent =
      number.toFixed(2);

    setMultiplier(number);
  }

  async function setMultiplier(value) {
    try {
      const response = await fetch(
        `/setMultiplier?value=${encodeURIComponent(value)}`,
        { cache: "no-store" }
      );

      if (!response.ok) {
        throw new Error("HTTP error");
      }

      const data = await response.json();
      updateDisplay(data);
      setStatus("Multiplier updated");
    } catch (error) {
      setStatus("Could not update multiplier");
    }
  }

  async function zeroImu() {
    try {
      const response = await fetch("/zero", {
        method: "POST",
        cache: "no-store"
      });

      if (!response.ok) {
        throw new Error("HTTP error");
      }

      const data = await response.json();
      updateDisplay(data);
      setStatus("IMU re-zeroed");
    } catch (error) {
      setStatus("Could not re-zero IMU");
    }
  }

  function stopMotor() {
    document.getElementById("multiplier").value = 0;
    document.getElementById("multiplierNumber").value = "0.00";
    document.getElementById("multiplierText").textContent = "0.00";
    setMultiplier(0);
  }

  updateTimer = setInterval(getStatus, 100);
  getStatus();
</script>
</body>
</html>
)rawliteral";

// ============================================================
// ANGLE AND SPEED FUNCTIONS
// ============================================================

float normalizeDegrees(float angle) {
  angle = fmodf(angle, 360.0f);

  if (angle < 0.0f) {
    angle += 360.0f;
  }

  return angle;
}

float calculateBaseSpeed(float angleDegrees) {
  const float angle = normalizeDegrees(angleDegrees);

  if (angle <= 180.0f) {
    // 0 degrees -> 0 rad/s
    // 180 degrees -> 20 rad/s
    return angle * (BASE_MAX_SPEED / 180.0f);
  }

  // 180 degrees -> 20 rad/s
  // 360 degrees -> 0 rad/s
  return (360.0f - angle) * (BASE_MAX_SPEED / 180.0f);
}

void updateTargetMotorSpeed() {
  mappedBaseSpeed = calculateBaseSpeed(imuAngleDegrees);

  targetMotorSpeed = mappedBaseSpeed * speedMultiplier;

  // Prevent invalid values from reaching the controller.
  if (!isfinite(targetMotorSpeed)) {
    targetMotorSpeed = 0.0f;
  }

  targetMotorSpeed = constrain(
    targetMotorSpeed,
    0.0f,
    BASE_MAX_SPEED * MAX_MULTIPLIER
  );
}

// ============================================================
// MPU6050 FUNCTIONS
// ============================================================

void calibrateGyroZ() {
  Serial.println("Calibrating MPU6050 gyro.");
  Serial.println("Keep the IMU completely still...");

  constexpr int SAMPLE_COUNT = 1000;

  double sum = 0.0;
  int successfulSamples = 0;

  sensors_event_t acceleration;
  sensors_event_t gyro;
  sensors_event_t temperature;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    if (mpu.getEvent(&acceleration, &gyro, &temperature)) {
      sum += gyro.gyro.z;
      successfulSamples++;
    }

    delay(2);
  }

  if (successfulSamples > 0) {
    gyroZBias = static_cast<float>(
      sum / successfulSamples
    );
  } else {
    gyroZBias = 0.0f;
  }

  imuAngleDegrees = 0.0f;
  previousImuTimeUs = micros();

  Serial.print("Gyro Z bias: ");
  Serial.print(gyroZBias, 7);
  Serial.println(" rad/s");
  Serial.println("IMU angle set to 0 degrees.");
}

void updateImu() {
  if (!imuReady) {
    return;
  }

  const uint32_t currentTimeUs = micros();
  const uint32_t elapsedUs =
    currentTimeUs - previousImuTimeUs;

  if (elapsedUs < IMU_UPDATE_INTERVAL_US) {
    return;
  }

  previousImuTimeUs = currentTimeUs;

  const float deltaTimeSeconds =
    elapsedUs * 0.000001f;

  // Reject unusually long integration intervals.
  if (deltaTimeSeconds <= 0.0f ||
      deltaTimeSeconds > 0.1f) {
    return;
  }

  sensors_event_t acceleration;
  sensors_event_t gyro;
  sensors_event_t temperature;

  if (!mpu.getEvent(
        &acceleration,
        &gyro,
        &temperature)) {
    return;
  }

  // Adafruit gyro output is radians per second.
  float correctedGyroZ =
    gyro.gyro.z - gyroZBias;

  // Small deadband reduces stationary drift.
  if (fabsf(correctedGyroZ) < 0.008f) {
    correctedGyroZ = 0.0f;
  }

  const float degreesPerSecond =
    correctedGyroZ * RAD_TO_DEG;

  imuAngleDegrees +=
    degreesPerSecond * deltaTimeSeconds;

  imuAngleDegrees =
    normalizeDegrees(imuAngleDegrees);

  updateTargetMotorSpeed();
}

// ============================================================
// JSON AND WEB HANDLERS
// ============================================================

String createStatusJson() {
  float measuredVelocity = 0.0f;

  if (motorReady) {
    measuredVelocity = motor.shaft_velocity;
  }

  String json;
  json.reserve(220);

  json += "{";

  json += "\"angle\":";
  json += String(imuAngleDegrees, 2);

  json += ",\"multiplier\":";
  json += String(speedMultiplier, 3);

  json += ",\"baseSpeed\":";
  json += String(mappedBaseSpeed, 3);

  json += ",\"targetSpeed\":";
  json += String(targetMotorSpeed, 3);

  json += ",\"measuredSpeed\":";
  json += String(measuredVelocity, 3);

  json += ",\"imuReady\":";
  json += imuReady ? "true" : "false";

  json += ",\"motorReady\":";
  json += motorReady ? "true" : "false";

  json += "}";

  return json;
}

void sendJsonStatus() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", createStatusJson());
}

void handleRoot() {
  server.send_P(
    200,
    "text/html; charset=utf-8",
    INDEX_HTML
  );
}

void handleSetMultiplier() {
  if (!server.hasArg("value")) {
    server.send(
      400,
      "application/json",
      "{\"error\":\"Missing value parameter\"}"
    );
    return;
  }

  const float requestedValue =
    server.arg("value").toFloat();

  speedMultiplier = constrain(
    requestedValue,
    MIN_MULTIPLIER,
    MAX_MULTIPLIER
  );

  updateTargetMotorSpeed();

  Serial.print("Speed multiplier changed to: ");
  Serial.println(speedMultiplier, 2);

  sendJsonStatus();
}

void handleZeroImu() {
  imuAngleDegrees = 0.0f;
  previousImuTimeUs = micros();

  updateTargetMotorSpeed();

  Serial.println("IMU orientation re-zeroed.");

  sendJsonStatus();
}

void handleNotFound() {
  server.send(
    404,
    "application/json",
    "{\"error\":\"Not found\"}"
  );
}

void configureWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, sendJsonStatus);
  server.on(
    "/setMultiplier",
    HTTP_GET,
    handleSetMultiplier
  );
  server.on("/zero", HTTP_POST, handleZeroImu);
  server.onNotFound(handleNotFound);

  server.begin();

  Serial.println("HTTP server started.");
}

// ============================================================
// HARDWARE INITIALIZATION
// ============================================================

bool initializeImu() {
  Serial.println("Starting MPU6050...");

  if (!mpu.begin(0x68, &sensorBus)) {
    Serial.println("ERROR: MPU6050 not detected.");
    return false;
  }

  mpu.setAccelerometerRange(
    MPU6050_RANGE_8_G
  );

  mpu.setGyroRange(
    MPU6050_RANGE_500_DEG
  );

  mpu.setFilterBandwidth(
    MPU6050_BAND_21_HZ
  );

  delay(100);

  calibrateGyroZ();

  Serial.println("MPU6050 ready.");
  return true;
}

bool initializeMotor() {
  Serial.println("Starting AS5600 encoder...");

  encoder.init(&sensorBus);

  Serial.println("Starting SimpleFOC driver...");

  driver.voltage_power_supply =
    MOTOR_SUPPLY_VOLTAGE;

  driver.voltage_limit =
    MOTOR_VOLTAGE_LIMIT;

  if (!driver.init()) {
    Serial.println("ERROR: Driver initialization failed.");
    return false;
  }

  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);

  // Closed-loop velocity control.
  motor.controller =
    MotionControlType::velocity;

  motor.torque_controller =
    TorqueControlType::voltage;

  // Motor output limits
  motor.voltage_limit =
    MOTOR_VOLTAGE_LIMIT;

  motor.velocity_limit =
    BASE_MAX_SPEED * MAX_MULTIPLIER;

  // -----------------------------
  // Velocity PID starting values
  // -----------------------------
  motor.PID_velocity.P = 0.20f;
  motor.PID_velocity.I = 2.00f;
  motor.PID_velocity.D = 0.001f;

  // Limits PID voltage output.
  motor.PID_velocity.limit =
    MOTOR_VOLTAGE_LIMIT;

  // Output ramp limit in volts per second.
  motor.PID_velocity.output_ramp =
    500.0f;

  // Low-pass filtering of measured velocity.
  motor.LPF_velocity.Tf = 0.02f;

  // Space-vector PWM is usually efficient and smooth.
  motor.foc_modulation =
    FOCModulationType::SpaceVectorPWM;

  Serial.println("Initializing motor...");
  motor.init();

  Serial.println("Aligning encoder and motor...");
  Serial.println("The motor may move during this step.");

  if (!motor.initFOC()) {
    Serial.println("ERROR: FOC initialization failed.");
    return false;
  }

  targetMotorSpeed = 0.0f;
  motor.move(0.0f);

  Serial.println("Motor and FOC ready.");
  return true;
}

bool parseNetworkAddress(
  const char* text,
  IPAddress& address,
  const char* addressName
) {
  if (text == nullptr || strlen(text) == 0) {
    Serial.print("ERROR: ");
    Serial.print(addressName);
    Serial.println(" has not been entered.");
    return false;
  }

  if (!address.fromString(text)) {
    Serial.print("ERROR: Invalid ");
    Serial.print(addressName);
    Serial.print(": ");
    Serial.println(text);
    return false;
  }

  return true;
}

void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  IPAddress staticIP;
  IPAddress gateway;
  IPAddress subnet;
  IPAddress primaryDNS;
  IPAddress secondaryDNS;

  bool networkSettingsValid = true;

  networkSettingsValid &=
    parseNetworkAddress(
      STATIC_IP_TEXT,
      staticIP,
      "static IP address"
    );

  networkSettingsValid &=
    parseNetworkAddress(
      GATEWAY_TEXT,
      gateway,
      "gateway address"
    );

  networkSettingsValid &=
    parseNetworkAddress(
      SUBNET_TEXT,
      subnet,
      "subnet mask"
    );

  networkSettingsValid &=
    parseNetworkAddress(
      PRIMARY_DNS_TEXT,
      primaryDNS,
      "primary DNS address"
    );

  networkSettingsValid &=
    parseNetworkAddress(
      SECONDARY_DNS_TEXT,
      secondaryDNS,
      "secondary DNS address"
    );

  if (!networkSettingsValid) {
    Serial.println();
    Serial.println(
      "Wi-Fi was not started because the static network "
      "settings are incomplete or invalid."
    );
    return;
  }

  /*
   * WiFi.config() must be called before WiFi.begin().
   *
   * Parameters:
   *   local IP
   *   gateway
   *   subnet mask
   *   primary DNS
   *   secondary DNS
   */
  if (!WiFi.config(
        staticIP,
        gateway,
        subnet,
        primaryDNS,
        secondaryDNS)) {
    Serial.println(
      "ERROR: Static IP configuration failed."
    );
    return;
  }

  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(WIFI_SSID);

  Serial.print("Requested static IP: ");
  Serial.println(staticIP);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const uint32_t startTime = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");

    if (millis() - startTime > 20000) {
      Serial.println();
      Serial.println("Wi-Fi connection timed out.");
      return;
    }
  }

  Serial.println();
  Serial.println("Wi-Fi connected.");

  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());

  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());

  Serial.print("Subnet mask: ");
  Serial.println(WiFi.subnetMask());

  Serial.print("Open this address in your browser: http://");
  Serial.println(WiFi.localIP());
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1200);

  Serial.println();
  Serial.println("================================");
  Serial.println("ESP32 Gyroscopic Motor Control");
  Serial.println("================================");

  // AS5600 and MPU6050 share this bus.
  sensorBus.begin(I2C_SDA, I2C_SCL);
  sensorBus.setClock(400000);

  imuReady = initializeImu();

  if (!imuReady) {
    Serial.println(
      "Motor will remain stopped because the IMU failed."
    );
  }

  motorReady = initializeMotor();

  if (!motorReady) {
    Serial.println(
      "Motor control initialization failed."
    );
  }

  connectToWiFi();

  if (WiFi.status() == WL_CONNECTED) {
    configureWebServer();
  }

  updateTargetMotorSpeed();

  Serial.println("Setup complete.");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  /*
   * SimpleFOC's loopFOC() must run as frequently as possible.
   * Avoid delay() in this loop.
   */
  if (motorReady) {
    motor.loopFOC();

    if (imuReady) {
      motor.move(targetMotorSpeed);
    } else {
      motor.move(0.0f);
    }
  }

  updateImu();

  /*
   * handleClient() should return quickly, allowing loopFOC()
   * to continue running frequently.
   */
  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }

  // Serial diagnostics at 5 Hz.
  const uint32_t currentMillis = millis();

  if (currentMillis - lastSerialPrintMs >= 200) {
    lastSerialPrintMs = currentMillis;

    Serial.print("IMU: ");
    Serial.print(imuAngleDegrees, 1);
    Serial.print(" deg | base: ");
    Serial.print(mappedBaseSpeed, 2);
    Serial.print(" rad/s | multiplier: ");
    Serial.print(speedMultiplier, 2);
    Serial.print(" | target: ");
    Serial.print(targetMotorSpeed, 2);

    if (motorReady) {
      Serial.print(" | measured: ");
      Serial.print(motor.shaft_velocity, 2);
    }

    Serial.println();
  }
}