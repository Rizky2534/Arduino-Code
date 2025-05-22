#include <Wire.h>
#include <Adafruit_INA219.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ArtronShop_BH1750.h>
#include <ThingSpeak.h>
#include <HTTPClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_MLX90614.h>

Adafruit_INA219 ina219_pv(0x45);
Adafruit_INA219 ina219_bat(0x41);
Adafruit_INA219 ina219_load(0x40);

ArtronShop_BH1750 bh1750(0x23, &Wire);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

const int proteksi_pv = 12;
const int proteksi_bat = 13;
const int proteksi_load = 14;

const float overVoltageThreshold = 14.6;
const float overCurrentThreshold = 1500.0;
const float underVoltageThreshold = 10.9;
const float intervalRecoveryThreshold = 0.5;
const float temperatureThreshold = 53.0;

unsigned long lastReadTime = 0;
unsigned long lastDisplayTime = 0;
unsigned long lastsendiotTime = 0;
const int readInterval = 70;
const int displayInterval = 5000;
const int sendiotInterval = 15000;

float PVVoltage, batteryVoltage, LampVoltage;
float voltage_pv, voltage_bat, voltage_load;
float current_mA_1, current_mA_2, current_mA_3;
float temperature, ambient, lightIntensity;
float estimatedSOC;

bool batteryOvervoltage = false;
bool batteryUndervoltage = false;
bool batteryOvercurrent = false;
bool temperatureOverThreshold = false;

String success;

const char* serverName = "https://script.google.com/macros/s/AKfycbwpo0mW9zCeTdfEHbCtlsNqqbHir41bfLzmGV-XTkhkjrWkUefl0TCrv0LLJIlEksE/exec";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 25200, 60000);

const char* ssid = "vivo 1938";
const char* password = "sijitekowolu";
const char* writeAPIKey = "0BJP7SWF0S163E0K";
const long channelID = 2783708;

WiFiClient client;

const float ocvTable[] = {
  14.40, 14.35, 14.30, 14.25, 14.20, 14.15, 14.10, 14.05, 14.00, 13.95,
  13.90, 13.85, 13.80, 13.75, 13.70, 13.65, 13.60, 13.55, 13.50, 13.45,
  13.40, 13.35, 13.30, 13.25, 13.20, 13.15, 13.10, 13.05, 13.00, 12.95,
  12.90, 12.85, 12.80, 12.75, 12.70, 12.65, 12.60, 12.55, 12.50, 12.45,
  12.40, 12.35, 12.30, 12.25, 12.20, 12.15, 12.10, 12.05, 12.00, 11.95,
  11.90, 11.85, 11.80, 11.75, 11.70, 11.65, 11.60, 11.55, 11.50, 11.45,
  11.40, 11.35, 11.30, 11.25, 11.20, 11.15, 11.10, 11.05, 11.00, 10.95,
  10.90, 10.85, 10.80, 10.75, 10.70, 10.65, 10.60, 10.55, 10.50, 10.45,
  10.40
}; 

const float socTable[] = {
  100, 99, 98, 97, 96, 95, 94, 93, 92, 91,
  90, 89, 88, 87, 86, 85, 84, 83, 82, 81,
  80, 79, 78, 77, 76, 75, 74, 73, 72, 71,
  70, 69, 68, 67, 66, 65, 64, 63, 62, 61,
  60, 59, 58, 57, 56, 55, 54, 53, 52, 51,
  50, 49, 48, 47, 46, 45, 44, 43, 42, 41,
  40, 39, 38, 37, 36, 35, 34, 33, 32, 31,
  30, 29, 28, 27, 26, 25, 24, 23, 22, 21,
  20, 19, 18, 17, 16, 15, 14, 13, 12, 11,
  10, 9, 8, 7, 6, 5, 4, 3, 2, 1,
  0
};

const int tableSize = sizeof(ocvTable) / sizeof(ocvTable[0]);

void setup(void) {
  Serial.begin(115200);
  Wire.begin();
  WiFi.begin(ssid, password);

  if (!ina219_pv.begin()) {
    Serial.println("Failed to find INA219 at 0x40");
    while (1);
  }
  if (!ina219_bat.begin()) {
    Serial.println("Failed to find INA219 at 0x41");
    while (1);
  }
  if (!ina219_load.begin()) {
    Serial.println("Failed to find INA219 at 0x45");
    while (1);
  }

  pinMode(proteksi_pv, OUTPUT);
  digitalWrite(proteksi_pv, HIGH);

  pinMode(proteksi_bat, OUTPUT);
  digitalWrite(proteksi_bat, HIGH);

  pinMode(proteksi_load, OUTPUT);
  digitalWrite(proteksi_load, HIGH);

  ina219_pv.setCalibration_32V_2A();
  ina219_bat.setCalibration_32V_2A();
  ina219_load.setCalibration_32V_2A();

  WiFi.mode(WIFI_STA);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to Wi-Fi...");
  }
  Serial.println("Wi-Fi connected!");
  ThingSpeak.begin(client);

  // Initialize BH1750
  if (!bh1750.begin() || !mlx.begin()) {
    Serial.println("Sensor initialization failed!");
    while (1);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
}

void readSensor() {
  float shuntVoltage_1 = ina219_pv.getShuntVoltage_mV() / 1000.0;
  current_mA_1 = ina219_pv.getCurrent_mA();
  voltage_pv = ina219_pv.getBusVoltage_V();
  PVVoltage = voltage_pv + (shuntVoltage_1 / 1000);

  float shuntVoltage_2 = ina219_bat.getShuntVoltage_mV() / 1000.0;
  current_mA_2 = ina219_bat.getCurrent_mA();
  voltage_bat = ina219_bat.getBusVoltage_V();
  batteryVoltage = voltage_bat + (shuntVoltage_2 / 1000);

  float shuntVoltage_3 = ina219_load.getShuntVoltage_mV() / 1000.0;
  current_mA_3 = ina219_load.getCurrent_mA();
  voltage_load = ina219_load.getBusVoltage_V();
  LampVoltage = voltage_load + (shuntVoltage_3 / 1000);

  lightIntensity = bh1750.light();
  ambient = mlx.readAmbientTempC();
  temperature = mlx.readObjectTempC();
}

void applyProtection() {
  if (batteryVoltage > overVoltageThreshold) {
    digitalWrite(proteksi_pv, LOW); 
    digitalWrite(proteksi_bat, HIGH); 
    digitalWrite(proteksi_load, HIGH); 
    batteryOvervoltage = true;
    batteryUndervoltage = false;
    batteryOvercurrent = false;
    temperatureOverThreshold = false;
  }
  
  else if (batteryVoltage < underVoltageThreshold) {
    digitalWrite(proteksi_load, LOW);
    digitalWrite(proteksi_pv, HIGH);
    digitalWrite(proteksi_bat, HIGH);
    batteryUndervoltage = true;
    batteryOvervoltage = false;
    batteryOvercurrent = false;
    temperatureOverThreshold = false;
  }

  else if (batteryUndervoltage && batteryVoltage < (underVoltageThreshold + intervalRecoveryThreshold)) {
    digitalWrite(proteksi_load, LOW);
    digitalWrite(proteksi_pv, HIGH);
    digitalWrite(proteksi_bat, HIGH);
  } 

  else if (batteryUndervoltage && batteryVoltage >= (underVoltageThreshold + intervalRecoveryThreshold)) {
    digitalWrite(proteksi_load, HIGH);
    digitalWrite(proteksi_pv, HIGH);
    digitalWrite(proteksi_bat, HIGH);
    batteryUndervoltage = false;
  }

  else if (current_mA_2 > overCurrentThreshold) {
    if (digitalRead(proteksi_pv) == HIGH) {
      digitalWrite(proteksi_pv, LOW); 
      digitalWrite(proteksi_bat, HIGH);
    } else {
      digitalWrite(proteksi_bat, LOW);
    }
    batteryOvercurrent = true;
    digitalWrite(proteksi_load, HIGH);
    batteryUndervoltage = false;
  }
  
  else if (temperature > temperatureThreshold) {
    digitalWrite(proteksi_bat, LOW);
    temperatureOverThreshold = true;
  }

  else {
    digitalWrite(proteksi_pv, HIGH);
    digitalWrite(proteksi_bat, HIGH);
    digitalWrite(proteksi_load, HIGH);
    batteryOvervoltage = false;
    batteryOvercurrent = false;
    batteryUndervoltage = false;
    temperatureOverThreshold = false;
  }
}

void displayData() {
  Serial.print("Photovoltaic Voltage: "); Serial.print(PVVoltage); Serial.println(" V");
  Serial.print("Photovoltaic Current: "); Serial.print(current_mA_1); Serial.println(" mA");
  Serial.println();

  Serial.print("Battery Voltage: "); Serial.print(batteryVoltage); Serial.println(" V");
  Serial.print("Battery Current: "); Serial.print(current_mA_2); Serial.println(" mA");
  Serial.println();

  Serial.print("Lamp Voltage: "); Serial.print(LampVoltage); Serial.println(" V");
  Serial.print("Lamp Current: "); Serial.print(current_mA_3); Serial.println(" mA");
  Serial.println();

  Serial.print("Battery Temp: "); Serial.print(temperature); Serial.println(" C");

  Serial.print("Light: ");
  Serial.print(lightIntensity);
  Serial.println(" lx");
  Serial.print("Battery Temp = "); Serial.print(temperature); Serial.println(" *C");

  estimatedSOC = estimateSoC(batteryVoltage);
  Serial.print("Estimated SoC: "); Serial.print(estimatedSOC); Serial.println(" %");
  
  if (batteryOvervoltage) {
    Serial.println("Overvoltage detected on Battery! MOSFET turned off.");
  }
  if (batteryOvercurrent) {
    Serial.println("Overcurrent detected! MOSFET turned off.");
  }
  if (batteryUndervoltage) {
    Serial.println("Undervoltage detected on Battery! MOSFET turned off.");
  }
  if (temperatureOverThreshold) {
    Serial.println("Over Temperature detected on Battery! MOSFET turned off.");
  } else {
    Serial.println ("Battery operating within safe limits.");
  }
  
  Serial.println();
}

float estimateSoC(float batteryVoltage) {
  // If battery voltage is below 10.40 V, set SoC to 0%
  if (batteryVoltage < 10.40) {
    return 0;
  }

  // Check for out-of-range voltage conditions
  if (batteryVoltage >= ocvTable[0]) return socTable[0];               // Maximum SoC (100%)
  if (batteryVoltage <= ocvTable[tableSize - 1]) return socTable[tableSize - 1]; // Minimum SoC (0%)

  // Find the two closest points in ocvTable for interpolation
  for (int i = 0; i < tableSize - 1; i++) {
    if (batteryVoltage <= ocvTable[i] && batteryVoltage > ocvTable[i + 1]) {
      float voltageDifference = ocvTable[i] - ocvTable[i + 1];
      float socDifference = socTable[i] - socTable[i + 1];
      
      // Linear interpolation
      float fraction = (batteryVoltage - ocvTable[i + 1]) / voltageDifference;
      return socTable[i + 1] + fraction * socDifference;
    }
  }
  return socTable[tableSize - 1]; // Fallback for unexpected cases
}

void senftoGsheet() {
  if (WiFi.status() == WL_CONNECTED) { // Check if ESP32 is connected to WiFi
    HTTPClient http;
    http.begin(serverName);
    http.addHeader("Content-Type", "application/json");
    timeClient.update();
    String formattedTime = timeClient.getFormattedTime();

    // Use global variables directly
    String jsonData = "{\"method\":\"append\", \"PVVoltage\":" + String(PVVoltage) +
                      ", \"current_mA_1\":" + String(current_mA_1) +
                      ", \"batteryVoltage\":" + String(batteryVoltage) +
                      ", \"current_mA_2\":" + String(current_mA_2) +
                      ", \"LampVoltage\":" + String(LampVoltage) +
                      ", \"current_mA_3\":" + String(current_mA_3) +
                      ", \"temperature\":" + String(temperature) +
                      ", \"lightIntensity\":" + String(lightIntensity) +
                      ", \"estimatedSOC\":" + String(estimatedSOC) +
                      ", \"timestamp\":\"" + formattedTime + "\"" + "}";

    int httpResponseCode = http.POST(jsonData);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(httpResponseCode);
      Serial.println(response);
    } else {
      Serial.println("Error on sending POST: " + String(httpResponseCode));
    }
    http.end(); // Close connection
  } else {
    Serial.println("WiFi Disconnected");
  }
}

void sendIoTData() {
  ThingSpeak.setField(1, current_mA_1);
  ThingSpeak.setField(2, PVVoltage);
  ThingSpeak.setField(3, current_mA_2);
  ThingSpeak.setField(4, batteryVoltage);
  ThingSpeak.setField(5, current_mA_3);
  ThingSpeak.setField(6, LampVoltage);
  ThingSpeak.setField(7, temperature);
  ThingSpeak.setField(8, lightIntensity);
  ThingSpeak.writeFields(channelID, writeAPIKey);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastReadTime >= readInterval) {
    lastReadTime = currentMillis;
    readSensor();
    applyProtection();
  }
  if (currentMillis - lastDisplayTime >= displayInterval) {
    lastDisplayTime = currentMillis;
    displayData();
  }
  if (currentMillis - lastsendiotTime >= sendiotInterval) {
    lastsendiotTime = currentMillis;
    sendIoTData();
    senftoGsheet();
  }
}
