#include <Wire.h>
#include <Adafruit_INA219.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ADS1115_WE.h> 

Adafruit_INA219 ina219_pv(0x45);
Adafruit_INA219 ina219_bat(0x41);
Adafruit_INA219 ina219_load(0x40);

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
unsigned long lastDeliveredData = 0;
const int readInterval = 70;
const int displayInterval = 5000;
const int deliveredData = 100;

float PVVoltage, batteryVoltage, LampVoltage;
float voltage_pv, voltage_bat, voltage_load;
float current_mA_1, current_mA_2, current_mA_3;
float temperature, humidity, lightIntensity;

bool batteryOvervoltage = false;
bool batteryUndervoltage = false;
bool batteryOvercurrent = false;
bool temperatureOverThreshold = false;

String success;

uint8_t broadcastAddress[] = {0x08, 0xA6, 0xF7, 0x22, 0xA5, 0xE0}; // 08:A6:F7:22:A5:E0 0xD0, 0xEF, 0x76, 0x5C, 0x9C, 0x10  08:A6:F7:22:A5:E0

typedef struct received_message {
  float temperature;
} received_message;

received_message receivedData;

typedef struct sensor_message {
  float PVVoltage;
  float current_mA_1;
  float batteryVoltage;
  float current_mA_2;
  float LampVoltage;
  float current_mA_3;
} sensor_message;

sensor_message sensorData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  temperature = receivedData.temperature;
}

void setup(void) {
  Serial.begin(115200);
  Wire.begin();

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

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  esp_now_register_send_cb(OnDataSent);

  // Add peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Peer added successfully.");
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

void espnow() {
  sensorData.PVVoltage = PVVoltage;
  sensorData.current_mA_1 = current_mA_1;
  sensorData.batteryVoltage = batteryVoltage;
  sensorData.current_mA_2 = current_mA_2;
  sensorData.LampVoltage = LampVoltage;
  sensorData.current_mA_3 = current_mA_3;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sensorData, sizeof(sensorData));
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
  if (currentMillis - lastDeliveredData >= deliveredData) {
    lastDeliveredData = currentMillis;
    espnow();
  }
}

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  voltage = adc.getResult_V();
  return voltage;
}
