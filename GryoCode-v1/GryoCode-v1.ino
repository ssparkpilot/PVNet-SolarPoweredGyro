//Version 1 of GryoCode
//Attempts to:
//Turn motor on/off via wifi
//Set motor speed via wifi
//See IMU configurations via wifi

#include <WiFi.h> //ESP32 Wifi library
#include <SimpleFOC.h> //Motor driver library
#include <Wire.h> //I2C bus library
#include <Adafruit_MPU6050.h> //IMU library
#include <Adafruit_Sensor.h> //IMU library

//WIFI section
//PVNET Wifi. ESP32 will connect to this. Control device will need to be connected to this network as well
const char *ssid = "PVNET Guest";
const char *password = "Promenade";

// Static IP configuration
IPAddress local_IP(10, 0, 2, 45);      // Desired IP address for ESP32
IPAddress gateway(10, 0, 2, 1);        // Router IP address
IPAddress subnet(255, 255, 255, 0);       // Subnet mask
IPAddress primaryDNS(1, 1, 1, 3);         // Optional
IPAddress secondaryDNS(1, 0, 0, 3);       // Optional

NetworkServer server(80); //Creates a network server on the ESP32 (port 80) whihc listens for connection requests

//MOTOR section
BLDCMotor motor = BLDCMotor(7); //Create motor object
BLDCDriver3PWM driver = BLDCDriver3PWM(15, 2, 4, 5); //SimpleFOCmini 3PWM driver pins
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C); // AS5600 magnetic encoder over I2C

float target_velo = 0; //Initial target motor velocity

//IMU section
TwoWire MPUWire = TwoWire(1);
Adafruit_MPU6050 mpu;

float sensorValueX = 0.0;
float sensorValueY = 0.0;
float sensorValueZ = 0.0;

#define MPU_SDA 32
#define MPU_SCL 33

//TIME UPDATE section
unsigned long previousSensorUpdate = 0;
const unsigned long sensorUpdateInterval = 100;


void setup() {
  Serial.begin(115200);
  pinMode(5, OUTPUT); //Temporary LED

  //IMU Setup
  MPUWire.begin(MPU_SDA, MPU_SCL, 400000);

  if (!mpu.begin(0x68, &MPUWire)) {
    Serial.println("MPU6050 not found");
    while (1) delay(10);
  }

  Serial.println("MPU6050 connected");

  // Configure the static IP BEFORE connecting to Wi-Fi
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Static IP configuration failed!");
  }

  // Connect to the Wi-Fi network
  WiFi.begin(ssid, password);

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin();
}

void loop() {
  //Server connection and webpage management
  updateSensorValue();
  handleWebClient();
    
}
