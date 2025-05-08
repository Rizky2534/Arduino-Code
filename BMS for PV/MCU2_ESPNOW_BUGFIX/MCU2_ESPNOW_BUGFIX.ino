#include <WiFi.h>
#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArtronShop_BH1750.h>
#include <ThingSpeak.h>
#include <HTTPClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_MLX90614.h>

unsigned long lastReadTime = 0;
unsigned long lastDisplayTime = 0;
unsigned long lastDeliveredData = 0;
unsigned long lastsendiotTime = 0;
const int readInterval = 70;
const int displayInterval = 5000;
const int deliveredData = 100;
const int sendiotInterval = 15000;

float PVVoltage, batteryVoltage, LampVoltage;
float Vref;
float current_mA_1, current_mA_2, current_mA_3;
float temperature, ambient, lightIntensity;
float estimatedSOC;

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

uint8_t broadcastAddress[] = {0x08, 0xA6, 0xF7, 0x22, 0x7F, 0x3C}; // 0x10, 0x06, 0x1C, 0x85, 0xC9, 0x10 || 0x08, 0xA6, 0xF7, 0x22, 0x7F, 0x3C

typedef struct received_message {
  float PVVoltage;
  float current_mA_1;
  float batteryVoltage;
  float current_mA_2;
  float LampVoltage;
  float current_mA_3;
  float Vref;
} received_message;

received_message receivedData;

typedef struct {
  float temperature;
} sensor_message;

sensor_message sensorData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  PVVoltage = receivedData.PVVoltage;
  current_mA_1 = receivedData.current_mA_1;
  batteryVoltage = receivedData.batteryVoltage;
  current_mA_2 = receivedData.current_mA_2;
  LampVoltage = receivedData.LampVoltage;
  current_mA_3 = receivedData.current_mA_3; 
  Vref = receivedData.Vref;
}

// Initialize sensors with default I2C addresses
ArtronShop_BH1750 bh1750(0x23, &Wire);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP_STA);
  Wire.begin();
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to Wi-Fi...");
  }
  Serial.println("Wi-Fi connected!");
  ThingSpeak.begin(client);

  int channel = WiFi.channel();  // Get the current Wi-Fi channel
  Serial.print("Wi-Fi Channel: ");
  Serial.println(channel);

  // Initialize BH1750
  if (!bh1750.begin() || !mlx.begin()) {
    Serial.println("Sensor initialization failed!");
    while (1);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  esp_now_register_send_cb(OnDataSent);

  // Add peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = channel;  // Match the Wi-Fi channel
  peerInfo.encrypt = false;    // Encryption disabled for simplicity

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error: Failed to add peer.");
  } else {
    Serial.println("Peer added successfully.");
  }
}

void readSensor() {
  lightIntensity = bh1750.light();
  ambient = mlx.readAmbientTempC();
  temperature = mlx.readObjectTempC();
}

void displayData() {
  Serial.print("Vref: "); Serial.print(Vref); Serial.println(" V");
  Serial.print("Photovoltaic Voltage: "); Serial.print(PVVoltage); Serial.println(" V");
  Serial.print("Photovoltaic Current: "); Serial.print(current_mA_1); Serial.println(" mA");
  Serial.println();

  Serial.print("Battery Voltage: "); Serial.print(batteryVoltage); Serial.println(" V");
  Serial.print("Battery Current: "); Serial.print(current_mA_2); Serial.println(" mA");
  Serial.println();

  Serial.print("Lamp Voltage: "); Serial.print(LampVoltage); Serial.println(" V");
  Serial.print("Lamp Current: "); Serial.print(current_mA_3); Serial.println(" mA");
  Serial.println();

  Serial.print("Light: ");
  Serial.print(lightIntensity);
  Serial.println(" lx");
  Serial.print("Battery Temp = "); Serial.print(temperature); Serial.println(" *C");

  estimatedSOC = estimateSoC(batteryVoltage);
  Serial.print("Estimated SoC: "); Serial.print(estimatedSOC); Serial.println(" %");
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

void espnow() {
  sensorData.temperature = temperature;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sensorData, sizeof(sensorData));
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

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastReadTime >= readInterval) {
    lastReadTime = currentMillis;
    readSensor();
  }
  if (currentMillis - lastDisplayTime >= displayInterval) {
    lastDisplayTime = currentMillis;
    displayData();
  }
  if (currentMillis - lastDeliveredData >= deliveredData) {
    lastDeliveredData = currentMillis;
    espnow();
  }
  if (currentMillis - lastsendiotTime >= sendiotInterval) {
    lastsendiotTime = currentMillis;
    sendIoTData();
    senftoGsheet();
  }
}