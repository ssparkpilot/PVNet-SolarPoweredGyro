
// ============================================================
// WEB HANDLERS
// ============================================================

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  String json;
  json.reserve(320);

  json += "{";

  json += "\"enabled\":";
  json += stabilizationEnabled ? "true" : "false";

  json += ",\"emergency_stop\":";
  json += emergencyStop ? "true" : "false";

  json += ",\"angle_deg\":";
  json += String(radiansToDegrees(boxAngleRad), 4);

  json += ",\"rate_deg_s\":";
  json += String(radiansToDegrees(boxRateRadS), 4);

  json += ",\"wheel_rad_s\":";
  json += String(motor.shaft_velocity, 4);

  json += ",\"motor_voltage\":";
  json += String(commandedMotorVoltage, 4);

  json += ",\"gyro_bias\":";
  json += String(gyroBiasZRadS, 7);

  json += ",\"kp\":";
  json += String(boxKp, 4);

  json += ",\"ki\":";
  json += String(boxKi, 4);

  json += ",\"kd\":";
  json += String(boxKd, 4);

  json += ",\"limit\":";
  json += String(controlVoltageLimit, 3);

  json += "}";

  server.send(200, "application/json", json);
}

void handleEnable() {
  if (!emergencyStop) {
    rezeroBox();
    stabilizationEnabled = true;
    Serial.println("Stabilization enabled.");
  }

  server.send(200, "text/plain", "OK");
}

void handleDisable() {
  stabilizationEnabled = false;
  commandedMotorVoltage = 0.0f;
  motor.move(0.0f);
  resetController();

  Serial.println("Stabilization disabled.");

  server.send(200, "text/plain", "OK");
}

void handleRezero() {
  rezeroRequested = true;
  server.send(200, "text/plain", "OK");
}

void handleEmergencyStop() {
  emergencyStop = true;
  stabilizationEnabled = false;
  commandedMotorVoltage = 0.0f;
  motor.move(0.0f);
  motor.disable();
  resetController();

  Serial.println("Emergency stop activated.");

  server.send(200, "text/plain", "STOPPED");
}

void handleClearEmergencyStop() {
  stabilizationEnabled = false;
  emergencyStop = false;
  commandedMotorVoltage = 0.0f;
  resetController();

  motor.enable();

  Serial.println("Emergency stop cleared.");

  server.send(200, "text/plain", "OK");
}

void handleSettings() {
  if (server.hasArg("kp")) {
    boxKp = clampValue(
        server.arg("kp").toFloat(),
        0.0f,
        20.0f
    );
  }

  if (server.hasArg("ki")) {
    boxKi = clampValue(
        server.arg("ki").toFloat(),
        0.0f,
        10.0f
    );
  }

  if (server.hasArg("kd")) {
    boxKd = clampValue(
        server.arg("kd").toFloat(),
        0.0f,
        10.0f
    );
  }

  if (server.hasArg("limit")) {
    controlVoltageLimit = clampValue(
        server.arg("limit").toFloat(),
        0.0f,
        MOTOR_VOLTAGE_LIMIT
    );
  }

  resetController();

  Serial.println("Controller settings updated.");

  server.send(200, "text/plain", "OK");
}

void configureWebServer() {
  server.on("/", HTTP_GET, handleRoot);

  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/enable", HTTP_POST, handleEnable);
  server.on("/api/disable", HTTP_POST, handleDisable);
  server.on("/api/rezero", HTTP_POST, handleRezero);
  server.on("/api/stop", HTTP_POST, handleEmergencyStop);
  server.on("/api/clearstop", HTTP_POST, handleClearEmergencyStop);
  server.on("/api/settings", HTTP_POST, handleSettings);

  server.on("/test", HTTP_GET, []() {
    Serial.println("Received request for /test");
    server.send(200, "text/plain", "ESP32 HTTP SERVER WORKS!");
  });

  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });

  server.begin();

  Serial.println("HTTP server started!");

  Serial.print("Website: http://");
  Serial.println(WiFi.localIP());

  Serial.print("Test: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/test");
}