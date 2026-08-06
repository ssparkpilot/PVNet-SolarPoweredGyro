
// ============================================================
// WI-FI
// ============================================================

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (!WiFi.config(
          LOCAL_IP,
          GATEWAY,
          SUBNET,
          PRIMARY_DNS,
          SECONDARY_DNS)) {
    Serial.println("Static IP configuration failed.");
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Wi-Fi connected.");

  Serial.print("Open this address: http://");
  Serial.println(WiFi.localIP());
}