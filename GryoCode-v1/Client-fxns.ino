void updateSensorValue() {
  unsigned long currentTime = millis();

  if (currentTime - previousSensorUpdate >= sensorUpdateInterval) { // millisecond update the length of sensorUpdateInterval
    previousSensorUpdate = currentTime;

    // Replace this with your actual sensor-reading code.
    //IMU data reception
      sensors_event_t accel, gyro, temp;
      mpu.getEvent(&accel, &gyro, &temp);
      sensorValueX = gyro.gyro.x;
      sensorValueY = gyro.gyro.y;
      sensorValueZ = gyro.gyro.z;
  }
}


void handleWebClient() {
  NetworkClient client = server.accept();

  if (!client) {
    return;
  }

  unsigned long timeoutStart = millis();

  // Wait briefly for the browser to send its request.
  while (!client.available() && client.connected()) {
    if (millis() - timeoutStart > 1000) {
      client.stop();
      return;
    }

    delay(1);
  }

  String requestLine = client.readStringUntil('\r');
  client.readStringUntil('\n');

  // Discard the remaining HTTP headers.
  while (client.connected()) {
    String headerLine = client.readStringUntil('\n');

    // A blank line marks the end of the HTTP headers.
    if (headerLine == "\r" || headerLine.length() == 0) {
      break;
    }
  }

  if (requestLine.indexOf("GET /data ") >= 0) {
    sendData(client);
  } else {
    sendWebPage(client);
  }

  client.stop();
}

void sendData(NetworkClient& client) { //This is what is actually sent to the webpage. Modify the values beneath "data sent here" to modify the visuals
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();

  //data sent here
  client.print("GyroX: ");
  client.println(sensorValueX, 2);
  client.print("GyroY: ");
  client.println(sensorValueY, 2);
  client.print("GyroZ: ");
  client.println(sensorValueZ, 2);
}

void sendWebPage(NetworkClient& client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Cache-Control: no-store");
  client.println("Connection: close");
  client.println();

  client.println(R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">

  <title>ESP32 Live Data</title>

  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      margin-top: 60px;
    }

    #value {
      font-size: 48px;
      font-weight: bold;
    }
  </style>
</head>

<body>
  <h1>Gyroscope Data</h1>

  <div id="value">
    Loading...
  </div>

  <script>
    async function updateValue() {
      try {
        const response = await fetch("/data", {
          cache: "no-store"
        });

        if (!response.ok) {
          throw new Error("HTTP error " + response.status);
        }

        const value = await response.text();
        document.getElementById("value").textContent = value;
      } catch (error) {
        console.error(error);
        document.getElementById("value").textContent =
          "Connection error";
      }
    }

    // Request a value immediately.
    updateValue();

    // Then request a new value every 500 milliseconds.
    setInterval(updateValue, 500);
  </script>
</body>
</html>
)rawliteral");
}

void sendNotFound(NetworkClient& client) {
  client.println("HTTP/1.1 404 Not Found");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.println("Page not found");
}
