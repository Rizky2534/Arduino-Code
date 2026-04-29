#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_MLX90614.h>
#include <esp_now.h>
#include <WiFi.h>

// Sensor objects with addresses
Adafruit_INA219 ina219_bat(0x40);    // INA219 for voltage at address 0x45
Adafruit_INA219 ina219_charge(0x45);   // INA219 for current at address 0x40
Adafruit_MLX90614 mlx = Adafruit_MLX90614();  // MLX90614 default address 0x5A

// Protection relay pin
const int proteksi_bat = 13;

// ESP-NOW configuration
#define CHANNEL 0
uint8_t masterAddress[] = {0x68, 0x25, 0xDD, 0xE7, 0xCB, 0x64};  // UPDATE WITH YOUR MASTER MAC

// Slave ID
const uint8_t SLAVE_ID = 1;

// Simple packet counter
uint32_t packetCounter = 0;  // This will increment on every send attempt

#pragma pack(push, 1)
struct SlaveData {
  uint8_t senderID;        // 1 byte
  float voltage;          // 4 bytes (V)
  float current1;         // 4 bytes (mA - Charge current)
  float current2;         // 4 bytes (mA - Discharge current)
  float temperature;      // 4 bytes (°C)
  uint16_t protectionFlags; // 2 bytes
  uint32_t sequence;      // 4 bytes - THIS INCREMENTS ON EVERY SEND
  uint32_t timestamp;     // 4 bytes
  uint16_t checksum;      // 2 bytes
  // TOTAL: 1+4+4+4+4+2+4+4+2 = 29 bytes
};
#pragma pack(pop)

SlaveData sensorData;

// Timing
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 1000;

// Protection thresholds
const float OVER_VOLTAGE_THRESHOLD = 18.1f;    // 5 × 3.7V
const float UNDER_VOLTAGE_THRESHOLD = 13.9f;   // 5 × 2.5V
const float OVER_CURRENT_CHARGE = 1000.0f;    // 1A = 1000mA
const float OVER_CURRENT_DISCHARGE = 2000.0f; // 2A = 2000mA
const float OVER_TEMP_THRESHOLD = 40.0f;
const float UNDER_TEMP_THRESHOLD = 10.0f;
const float CURRENT_THRESHOLD_UV = 2.0f;      // 2.0mA

// Sensor status
bool ina219_batAvailable = false;
bool ina219_chargeAvailable = false;
bool mlxAvailable = false;

// ==================== I2C SCANNER ====================
void scanI2C() {
    Serial.println("\n=== I2C BUS SCAN ===");
    Serial.println("Scanning for I2C devices...");
    
    byte error, address;
    int foundDevices = 0;
    
    for(address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.printf("  Found device at address 0x%02X", address);
            
            // Identify known devices
            if (address == 0x40) {
                Serial.println("  --> INA219 (Current)");
            } else if (address == 0x45) {
                Serial.println("  --> INA219 (Voltage)");
            } else if (address == 0x5A) {
                Serial.println("  --> MLX90614 (Temperature)");
            } else {
                Serial.println("  --> Unknown device");
            }
            foundDevices++;
        }
    }
    
    if (foundDevices == 0) {
        Serial.println("NO I2C devices found! Check connections:");
        Serial.println("1. Verify SDA and SCL are properly connected");
        Serial.println("2. Check pull-up resistors (4.7kΩ recommended)");
        Serial.println("3. Verify power supply to sensors");
    } else {
        Serial.printf("Total I2C devices found: %d\n", foundDevices);
    }
    Serial.println("=== END I2C SCAN ===\n");
}

// ==================== CHECK SPECIFIC SENSOR ADDRESS ====================
bool checkSensorAddress(uint8_t address, const char* sensorName) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
        Serial.printf("✓ Sensor at 0x%02X (%s) is responding\n", address, sensorName);
        return true;
    } else {
        Serial.printf("✗ Sensor at 0x%02X (%s) NOT FOUND! Error code: %d\n", 
                     address, sensorName, error);
        return false;
    }
}

// ==================== CALLBACK ====================
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    // Just print status, no heavy processing
    Serial.printf("[ESP-NOW] Seq: %u, Send status: %s\n",
                 sensorData.sequence,
                 status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAILED");
}

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n=== BMS SLAVE 1 STARTING ===");
    Serial.println("Pack 1 - Battery Management System");
    Serial.println("===================================\n");
    
    // Initialize I2C
    Wire.begin();
    delay(100);
    
    // Run I2C scanner first to see all connected devices
    scanI2C();
    
    // Initialize protection relay
    pinMode(proteksi_bat, OUTPUT);
    digitalWrite(proteksi_bat, HIGH);
    Serial.println("Protection relay: ON\n");
    
    // List expected sensor addresses
    Serial.println("=== EXPECTED SENSOR ADDRESSES ===");
    Serial.println("1. INA219 (Voltage): 0x45");
    Serial.println("2. INA219 (Current): 0x40");
    Serial.println("3. MLX90614 (Temp):  0x5A");
    Serial.println("===============================\n");
    
    // Check each sensor individually
    Serial.println("=== CHECKING INDIVIDUAL SENSORS ===");
    
    // Check INA219 for voltage (0x45)
    Serial.print("Checking INA219 at 0x45 (Voltage)... ");
    if (checkSensorAddress(0x45, "INA219 Voltage")) {
        if (ina219_bat.begin()) {
            ina219_batAvailable = true;
            ina219_bat.setCalibration_32V_2A();
            Serial.println("  SUCCESS: Initialized and calibrated");
        } else {
            ina219_batAvailable = false;
            Serial.println("  FAILED: Sensor found but initialization failed!");
        }
    } else {
        ina219_batAvailable = false;
        Serial.println("  ACTION: Check connections to INA219 at address 0x45");
    }
    
    // Check INA219 for current (0x40)
    Serial.print("\nChecking INA219 at 0x40 (Current)... ");
    if (checkSensorAddress(0x40, "INA219 Current")) {
        if (ina219_charge.begin()) {
            ina219_chargeAvailable = true;
            ina219_charge.setCalibration_32V_2A();
            Serial.println("  SUCCESS: Initialized and calibrated");
        } else {
            ina219_chargeAvailable = false;
            Serial.println("  FAILED: Sensor found but initialization failed!");
        }
    } else {
        ina219_chargeAvailable = false;
        Serial.println("  ACTION: Check connections to INA219 at address 0x40");
    }
    
    // Check MLX90614 (0x5A)
    Serial.print("\nChecking MLX90614 at 0x5A (Temperature)... ");
    if (checkSensorAddress(0x5A, "MLX90614 Temp")) {
        if (mlx.begin()) {
            mlxAvailable = true;
            Serial.println("  SUCCESS: Initialized");
        } else {
            mlxAvailable = false;
            Serial.println("  FAILED: Sensor found but initialization failed!");
        }
    } else {
        mlxAvailable = false;
        Serial.println("  ACTION: Check connections to MLX90614 at address 0x5A");
    }
    
    Serial.println("\n=== SENSOR SUMMARY ===");
    Serial.printf("INA219 Voltage (0x45): %s\n", 
                  ina219_batAvailable ? "CONNECTED" : "NOT CONNECTED");
    Serial.printf("INA219 Current (0x40): %s\n", 
                  ina219_chargeAvailable ? "CONNECTED" : "NOT CONNECTED");
    Serial.printf("MLX90614 Temp (0x5A):  %s\n", 
                  mlxAvailable ? "CONNECTED" : "NOT CONNECTED");
    Serial.println("=====================\n");
    
    // Initialize ESP-NOW
    Serial.println("Initializing ESP-NOW...");
    
    // Set WiFi mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    
    // Print MAC address
    Serial.print("Slave MAC Address: ");
    Serial.println(WiFi.macAddress());
    Serial.print("Master MAC Address: ");
    for(int i = 0; i < 6; i++) {
        Serial.printf("%02X", masterAddress[i]);
        if(i < 5) Serial.print(":");
    }
    Serial.println();
    
    // Initialize ESP-NOW
    esp_err_t result = esp_now_init();
    if (result != ESP_OK) {
        Serial.printf("ERROR: ESP-NOW initialization failed: %d\n", result);
        Serial.println("Restarting in 5 seconds...");
        delay(5000);
        ESP.restart();
    }
    
    Serial.println("ESP-NOW initialized successfully");
    
    // Register send callback
    esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);
    
    // Add master as peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, masterAddress, 6);
    peerInfo.channel = CHANNEL;
    peerInfo.encrypt = false;
    
    result = esp_now_add_peer(&peerInfo);
    if (result != ESP_OK) {
        Serial.printf("ERROR: Failed to add master peer: %d\n", result);
        Serial.println("Restarting in 5 seconds...");
        delay(5000);
        ESP.restart();
    }
    
    Serial.println("Master peer added successfully");
    Serial.println("\n=== SLAVE READY ===");
    Serial.println("Sending data every second...\n");
    
    // Initialize sensor data structure
    memset(&sensorData, 0, sizeof(SlaveData));
    sensorData.senderID = SLAVE_ID;
    packetCounter = 0;  // Start at 0
}

// ==================== SENSOR READING ====================
void readSensors() {
    static unsigned long lastErrorPrint = 0;
    static bool errorsPrinted[3] = {false, false, false};
    
    // Read INA219 data for voltage
    if (ina219_batAvailable) {
        float busVoltage = ina219_bat.getBusVoltage_V();
        float shuntVoltage_mV = ina219_bat.getShuntVoltage_mV();
        float current_mA_discharge = ina219_bat.getCurrent_mA();
        
        sensorData.voltage = busVoltage + (shuntVoltage_mV / 1000.0f);
        sensorData.current2 = abs(current_mA_discharge);
        errorsPrinted[0] = false;
    } else {
        sensorData.voltage = 0.0f;
        if (!errorsPrinted[0] && (millis() - lastErrorPrint > 5000)) {
            Serial.println("ERROR: INA219 for voltage at address 0x45 is not connected!");
            errorsPrinted[0] = true;
            lastErrorPrint = millis();
        }
    }
    
    // Read INA219 data for current
    if (ina219_chargeAvailable) {
        float current_mA_charge = ina219_charge.getCurrent_mA();
        sensorData.current1 = abs(current_mA_charge);
        errorsPrinted[1] = false;
    } else {
        sensorData.current1 = 0.0f;
        if (!errorsPrinted[1] && (millis() - lastErrorPrint > 5000)) {
            Serial.println("ERROR: INA219 for current at address 0x40 is not connected!");
            errorsPrinted[1] = true;
            lastErrorPrint = millis();
        }
    }
    
    // Read temperature
    if (mlxAvailable) {
        sensorData.temperature = mlx.readObjectTempC();
        errorsPrinted[2] = false;
    } else {
        sensorData.temperature = -999.0f;
        if (!errorsPrinted[2] && (millis() - lastErrorPrint > 5000)) {
            Serial.println("ERROR: MLX90614 at address 0x5A is not connected!");
            errorsPrinted[2] = true;
            lastErrorPrint = millis();
        }
    }
    
    // Update timestamp
    sensorData.timestamp = millis();
}

// ==================== PROTECTION CHECK ====================
void checkProtections() {
    sensorData.protectionFlags = 0;
    
    if (ina219_batAvailable && ina219_chargeAvailable && mlxAvailable) {
        bool underVoltageCondition = false;
        
        if (sensorData.voltage > OVER_VOLTAGE_THRESHOLD) {
            sensorData.protectionFlags |= (1 << 0);
            digitalWrite(proteksi_bat, LOW);
            Serial.println("PROTECTION: Over Voltage!");
        }
        
        if (sensorData.voltage < UNDER_VOLTAGE_THRESHOLD) {
            underVoltageCondition = true;
            sensorData.protectionFlags |= (1 << 1);
            
            if (sensorData.current1 < CURRENT_THRESHOLD_UV) {
                digitalWrite(proteksi_bat, LOW);
                Serial.println("PROTECTION: Under Voltage (current < 0.02mA)!");
            } else {
                digitalWrite(proteksi_bat, HIGH);
                Serial.println("UNDER VOLTAGE DETECTED but current > 0.02mA - MOSFET ON");
            }
        }
        
        if (sensorData.current1 > OVER_CURRENT_CHARGE) {
            sensorData.protectionFlags |= (1 << 2);
            digitalWrite(proteksi_bat, LOW);
            Serial.println("PROTECTION: Over Current Charge!");
        }
        
        if (sensorData.current2 > OVER_CURRENT_DISCHARGE) {
            sensorData.protectionFlags |= (1 << 3);
            digitalWrite(proteksi_bat, LOW);
            Serial.println("PROTECTION: Over Current Discharge!");
        }
        
        if (sensorData.temperature > OVER_TEMP_THRESHOLD) {
            sensorData.protectionFlags |= (1 << 4);
            digitalWrite(proteksi_bat, LOW);
            Serial.println("PROTECTION: Over Temperature!");
        }
        
        if (sensorData.temperature < UNDER_TEMP_THRESHOLD) {
            sensorData.protectionFlags |= (1 << 5);
            digitalWrite(proteksi_bat, LOW);
            Serial.println("PROTECTION: Under Temperature!");
        }
        
        if (sensorData.protectionFlags == 0 || 
            (underVoltageCondition && sensorData.current1 >= CURRENT_THRESHOLD_UV)) {
            digitalWrite(proteksi_bat, HIGH);
        }
    } else {
        digitalWrite(proteksi_bat, LOW);
    }
}

// ==================== CHECKSUM ====================
uint16_t calculateChecksum() {
    uint16_t checksum = 0;
    uint8_t* data = (uint8_t*)&sensorData;
    
    for(size_t i = 0; i < sizeof(SlaveData) - 2; i++) {
        checksum += data[i];
    }
    return checksum;
}

// ==================== SEND DATA ====================
void sendData() {
    // Read sensors
    readSensors();
    
    // Update sensor data
    sensorData.senderID = SLAVE_ID;
    
    // SIMPLE: Increment counter on EVERY send
    packetCounter++;
    sensorData.sequence = packetCounter;
    
    // Update timestamp
    sensorData.timestamp = millis();
    
    // Calculate checksum
    sensorData.checksum = calculateChecksum();
    
    // Print data with sequence number
    if (ina219_batAvailable && ina219_chargeAvailable && mlxAvailable) {
        Serial.printf("[SENDING] Seq: %u, Pack %d: V=%.2fV, I_charge=%.2fmA, I_discharge=%.2fmA, T=%.1f°C, Prot=0x%04X\n",
                      sensorData.sequence, sensorData.senderID, sensorData.voltage, sensorData.current1, 
                      sensorData.current2, sensorData.temperature, sensorData.protectionFlags);
    } else {
        Serial.printf("[SENDING] Seq: %u, Pack %d: V=%.2fV, I=%.2fmA, T=%.1f°C\n",
                      sensorData.sequence, sensorData.senderID,
                      sensorData.voltage, sensorData.current1, sensorData.temperature);
    }
    
    // Send data via ESP-NOW
    esp_err_t result = esp_now_send(masterAddress, (uint8_t*)&sensorData, sizeof(SlaveData));
    
    if (result != ESP_OK) {
        Serial.printf("[ERROR] Send failed immediately: %d\n", result);
    }
}

// ==================== MAIN LOOP ====================
void loop() {
    unsigned long currentMillis = millis();
    
    // Check protections
    checkProtections();
    
    // Send data at regular interval
    if (currentMillis - lastSendTime >= SEND_INTERVAL) {
        lastSendTime = currentMillis;
        sendData();
    }
    
    // Print status every 10 seconds
    static unsigned long lastStatus = 0;
    if (currentMillis - lastStatus >= 10000) {
        Serial.println("\n=== SYSTEM STATUS ===");
        Serial.printf("Pack ID: %d, Packets Sent: %u\n", SLAVE_ID, packetCounter);
        
        if (ina219_batAvailable && ina219_chargeAvailable && mlxAvailable) {
            Serial.printf("All sensors connected! V=%.2fV, I=%.2fmA, T=%.1f°C\n",
                         sensorData.voltage, sensorData.current1, sensorData.temperature);
            Serial.printf("Protection Flags: 0x%04X\n", sensorData.protectionFlags);
            Serial.printf("MOSFET State: %s\n", digitalRead(proteksi_bat) ? "ON" : "OFF");
        } else {
            Serial.println("MISSING SENSORS:");
            if (!ina219_batAvailable) Serial.println("  - INA219 Voltage at 0x45");
            if (!ina219_chargeAvailable) Serial.println("  - INA219 Current at 0x40");
            if (!mlxAvailable) Serial.println("  - MLX90614 Temp at 0x5A");
        }
        Serial.println("===================\n");
        lastStatus = currentMillis;
    }
    
    delay(10);
}