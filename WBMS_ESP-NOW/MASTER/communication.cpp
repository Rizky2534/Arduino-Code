#include "communication.h"
#include <esp_now.h>
#include <WiFi.h>
#include "display.h"
#include "soc_calculation.h"
#include "soh_calculation.h"

// External variables from other files
extern NotificationBuffer notificationBuffer;
extern volatile bool refreshDisplay;
extern volatile bool showNotification;
extern unsigned long notificationStartTime;
extern Notification currentNotification;

BMSCommBuffer commBuffer;

// SIMPLE: Packet loss counters
PacketLossCounters packetLoss;

// Create SOC and SOH calculators for each pack
SOC_Calculator socCalculatorPack1(1.8f);    // 1.8Ah battery
SOC_Calculator socCalculatorPack2(1.8f);    // 1.8Ah battery
SOH_Calculator sohCalculatorPack1(1.8f);    // 1.8Ah battery  
SOH_Calculator sohCalculatorPack2(1.8f);    // 1.8Ah battery

#define CHANNEL 0
uint8_t slave1Address[] = {0xEC, 0xE3, 0x34, 0x8E, 0xFC, 0x7C};
uint8_t slave2Address[] = {0xEC, 0xE3, 0x34, 0x8E, 0xD9, 0x50};

extern PackData pack1Data;
extern PackData pack2Data;
extern bool slave1Connected;
extern bool slave2Connected;
extern volatile bool refreshDisplay;

unsigned long lastDataTime[3] = {0};
unsigned long connectionTimeout = 10000;
unsigned long lastSOCUpdate[3] = {0};
const unsigned long SOC_UPDATE_INTERVAL = 1000;

// Protection flag descriptions
const char* protectionFlagNames[] = {
  "Over Voltage",
  "Under Voltage", 
  "Over Current Charge",
  "Over Current Discharge",
  "Over Temperature",
  "Under Temperature",
  "Short Circuit",
  "FET Over Temperature",
  "Communication Failure",
  "Reserved 1",
  "Reserved 2",
  "Reserved 3",
  "Reserved 4",
  "Reserved 5",
  "System Fault"
};

// ==================== HELPER FUNCTIONS ====================
uint16_t calculateChecksum(uint8_t* data, size_t len) {
    uint16_t checksum = 0;
    for (size_t i = 0; i < len - 2; i++) checksum += data[i];
    return checksum;
}

bool validateData(SlaveData* data) {
    uint16_t calculated = 0;
    uint8_t* raw = (uint8_t*)data;
    
    for(size_t i = 0; i < sizeof(SlaveData) - 2; i++) {
        calculated += raw[i];
    }
    
    if (calculated != data->checksum) {
        Serial.printf("Checksum mismatch: Got 0x%04X, Expected 0x%04X\n", 
                     calculated, data->checksum);
        return false;
    }
    
    if (data->voltage < 0 || data->voltage > 60) {
        Serial.printf("Invalid voltage: %.2f\n", data->voltage);
        return false;
    }
    // Validate both currents
    if (data->current1 < -20000 || data->current1 > 20000) {
        Serial.printf("Invalid charge current: %.0f mA\n", data->current1);
        return false;
    }
    if (data->current2 < -20000 || data->current2 > 20000) {
        Serial.printf("Invalid discharge current: %.0f mA\n", data->current2);
        return false;
    }
    if (data->temperature < -20 || data->temperature > 80) {
        Serial.printf("Invalid temperature: %.1f\n", data->temperature);
        return false;
    }
    return true;
}

void addNotification(const char* message, uint8_t severity, uint8_t packID = 0) {
    Notification notif(message, severity, packID);
    notif.timestamp = millis();
    notificationBuffer.push(notif);
    refreshDisplay = true;
    Serial.printf("[NOTIFICATION] Pack %d: %s (Severity: %d)\n", 
                  packID, message, severity);
}

void handleProtectionNotifications(uint8_t packID, uint16_t protectionFlags) {
    if (protectionFlags == 0) return;
    
    for (int i = 0; i < 15; i++) {
        if (protectionFlags & (1 << i)) {
            uint8_t severity;
            switch (i) {
                case 0:  // OVER_VOLTAGE
                case 1:  // UNDER_VOLTAGE
                case 6:  // SHORT_CIRCUIT
                case 14: // SYSTEM_FAULT
                    severity = 4;  // Critical
                    break;
                case 2:  // OVER_CURRENT_CHG
                case 3:  // OVER_CURRENT_DIS
                case 4:  // OVER_TEMPERATURE
                case 7:  // FET_OVER_TEMP
                    severity = 3;  // Error
                    break;
                case 5:  // UNDER_TEMPERATURE
                case 8:  // COMMUNICATION_FAIL
                    severity = 2;  // Warning
                    break;
                default:
                    severity = 2;
            }
            
            char faultDesc[50];
            snprintf(faultDesc, sizeof(faultDesc), "%s", protectionFlagNames[i]);
            addFault(faultDesc, severity, packID);
            
            char notifMsg[60];
            snprintf(notifMsg, sizeof(notifMsg), "%s - Pack %d", protectionFlagNames[i], packID);
            addNotification(notifMsg, severity, packID);
        }
    }
}

void updateDisplayData(SlaveData* data) {
    PackData* targetData = (data->senderID == 1) ? &pack1Data : &pack2Data;
    int packIndex = (data->senderID == 1) ? 0 : 1;
    
    targetData->voltage = data->voltage;
    targetData->chargeCurrent = data->current1;
    targetData->dischargeCurrent = data->current2;
    targetData->temperature = data->temperature;
    targetData->timestamp = millis();
    targetData->dataValid = true;
    
    unsigned long now = millis();
    bool isResting = (fabs(data->current1) < 100.0f && fabs(data->current2) < 100.0f);
    
    // Calculate net current in A for SOC calculation
    float netCurrentA = (data->current1 - data->current2) / 1000.0f;
    
    if (packIndex == 0) {
        float soc = socCalculatorPack1.estimateSOC(data->voltage, netCurrentA,
                                                  data->temperature, isResting);
        targetData->soc = soc;
        battery1SoC = soc;
        
        refreshDisplay = true;
        
        static float cumulativeAhPack1 = 0.0f;
        cumulativeAhPack1 += fabs(netCurrentA) * (now - lastSOCUpdate[1]) / 3600000.0f;
        
        if (now - lastSOCUpdate[1] > 60000) {
            sohCalculatorPack1.addChargeThroughput(cumulativeAhPack1);
            float soh = sohCalculatorPack1.calculateSOH(0, data->temperature);
            targetData->soh = soh;
            cumulativeAhPack1 = 0.0f;
        }
        
        slave1Connected = true;
        lastDataTime[1] = now;
        lastSOCUpdate[1] = now;
        
        Serial.printf("[UPDATE] Pack 1: SOC=%.1f%%, Display SOC=%.1f%%, SOH=%.1f%%, Charge=%.0fmA, Discharge=%.0fmA\n", 
                     soc, battery1SoC, targetData->soh, targetData->chargeCurrent, targetData->dischargeCurrent);
    } else {
        float soc = socCalculatorPack2.estimateSOC(data->voltage, netCurrentA,
                                                  data->temperature, isResting);
        targetData->soc = soc;
        battery2SoC = soc;
        
        refreshDisplay = true;
        
        static float cumulativeAhPack2 = 0.0f;
        cumulativeAhPack2 += fabs(netCurrentA) * (now - lastSOCUpdate[2]) / 3600000.0f;
        
        if (now - lastSOCUpdate[2] > 60000) {
            sohCalculatorPack2.addChargeThroughput(cumulativeAhPack2);
            float soh = sohCalculatorPack2.calculateSOH(0, data->temperature);
            targetData->soh = soh;
            cumulativeAhPack2 = 0.0f;
        }
        
        slave2Connected = true;
        lastDataTime[2] = now;
        lastSOCUpdate[2] = now;
        
        Serial.printf("[UPDATE] Pack 2: SOC=%.1f%%, Display SOC=%.1f%%, SOH=%.1f%%, Charge=%.0fmA, Discharge=%.0fmA\n", 
                     soc, battery2SoC, targetData->soh, targetData->chargeCurrent, targetData->dischargeCurrent);
    }
    
    if (data->protectionFlags != 0) {
        handleProtectionNotifications(data->senderID, data->protectionFlags);
    }
    
    Serial.printf("Pack %d: V=%.2fV, I_charge=%.0fmA, I_discharge=%.0fmA, SOC=%.1f%%, SOH=%.1f%%, Temp=%.1f°C\n",
                  data->senderID, data->voltage, data->current1, data->current2,
                  targetData->soc, targetData->soh, data->temperature);
}

// ==================== CALLBACK FUNCTIONS ====================
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    Serial.printf("[ESP-NOW] Send to %02X:%02X:%02X:%02X:%02X:%02X: %s\n",
                 mac_addr[0], mac_addr[1], mac_addr[2], 
                 mac_addr[3], mac_addr[4], mac_addr[5],
                 status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAILED");
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    uint8_t* mac = recv_info->src_addr;
    
    if (len != sizeof(SlaveData)) {
        Serial.printf("ERROR: Size mismatch! Got %d bytes, expected %d bytes\n", 
                     len, sizeof(SlaveData));
        return;
    }
    
    SlaveData receivedData;
    memcpy(&receivedData, data, sizeof(SlaveData));
    
    // SIMPLE: Update packet loss counters
    packetLoss.updateCounters(receivedData.senderID, receivedData.sequence);
    
    if (!validateData(&receivedData)) {
        char faultDesc[50];
        snprintf(faultDesc, sizeof(faultDesc), "Invalid data from Pack %d", receivedData.senderID);
        addFault(faultDesc, 2, receivedData.senderID);
        
        char notifMsg[60];
        snprintf(notifMsg, sizeof(notifMsg), "Invalid data from Pack %d", receivedData.senderID);
        addNotification(notifMsg, 2, receivedData.senderID);
        return;
    }
    
    Serial.printf("Valid data from Pack %d: Seq: %u, V=%.2fV, I_charge=%.0fmA, I_discharge=%.0fmA, T=%.1f°C, Prot=0x%04X\n",
                 receivedData.senderID, receivedData.sequence, receivedData.voltage, receivedData.current1, 
                 receivedData.current2, receivedData.temperature, receivedData.protectionFlags);
    
    // Convert to BMSCommPacket and buffer it
    BMSCommPacket packet;
    packet.senderID = receivedData.senderID;
    packet.sequence = receivedData.sequence;
    packet.voltage = receivedData.voltage;
    packet.chargeCurrent = receivedData.current1;
    packet.dischargeCurrent = receivedData.current2;
    packet.temperature = receivedData.temperature;
    packet.timestamp = millis();
    
    // Calculate SOC using net current
    bool isResting = (fabs(receivedData.current1) < 100.0f && fabs(receivedData.current2) < 100.0f);
    float netCurrentA = (receivedData.current1 - receivedData.current2) / 1000.0f;
    
    if (receivedData.senderID == 1) {
        packet.soc = socCalculatorPack1.estimateSOC(receivedData.voltage, 
                                                   netCurrentA, 
                                                   receivedData.temperature, 
                                                   isResting);
        packet.soh = pack1Data.soh;
    } else {
        packet.soc = socCalculatorPack2.estimateSOC(receivedData.voltage, 
                                                   netCurrentA, 
                                                   receivedData.temperature, 
                                                   isResting);
        packet.soh = pack2Data.soh;
    }
    
    // Add to buffer
    commBuffer.push(packet);
    
    // Handle protection notifications
    if (receivedData.protectionFlags != 0) {
        handleProtectionNotifications(receivedData.senderID, receivedData.protectionFlags);
    }
    
    // Update connection status
    if (receivedData.senderID == 1) {
        slave1Connected = true;
        lastDataTime[1] = millis();
    } else if (receivedData.senderID == 2) {
        slave2Connected = true;
        lastDataTime[2] = millis();
    }
    
    // Update display data
    updateDisplayData(&receivedData);
    
    // Print counters every 50 packets
    static uint32_t lastPrint = 0;
    if (packetLoss.packetsReceivedByMaster[receivedData.senderID] % 50 == 0) {
        if (millis() - lastPrint > 5000) {
            packetLoss.printCounters();
            lastPrint = millis();
        }
    }
}

// ==================== BUFFER PROCESSING ====================
void processBufferedPackets() {
    BMSCommPacket packet;
    while (commBuffer.pop(packet)) {
        PackData* targetData = (packet.senderID == 1) ? &pack1Data : &pack2Data;
        
        targetData->voltage = packet.voltage;
        targetData->chargeCurrent = packet.chargeCurrent;
        targetData->dischargeCurrent = packet.dischargeCurrent;
        targetData->temperature = packet.temperature;
        targetData->soc = packet.soc;
        targetData->soh = packet.soh;
        targetData->timestamp = packet.timestamp;
        targetData->dataValid = true;
        
        if (packet.senderID == 1) {
            battery1SoC = packet.soc;
        } else {
            battery2SoC = packet.soc;
        }
        
        refreshDisplay = true;
        
        if (packet.senderID == 1) {
            slave1Connected = true;
            lastDataTime[1] = millis();
        } else {
            slave2Connected = true;
            lastDataTime[2] = millis();
        }
        
        Serial.printf("Processed buffered packet from Pack %d: Seq: %u, SOC=%.1f%%, Charge=%.0fmA, Discharge=%.0fmA\n", 
                     packet.senderID, packet.sequence, packet.soc, packet.chargeCurrent, packet.dischargeCurrent);
    }
}

// ==================== COMMUNICATION FUNCTIONS ====================
void initCommunication() {
    Serial.println("\n=== Initializing ESP-NOW Communication ===");
    
    // Reset packet loss counters
    packetLoss = PacketLossCounters();
    
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    
    Serial.print("Master MAC Address: ");
    Serial.println(WiFi.macAddress());
    
    esp_err_t result = esp_now_init();
    if (result != ESP_OK) {
        Serial.printf("ERROR: ESP-NOW initialization failed: %d\n", result);
        addFault("ESP-NOW init failed", 3, 0);
        addNotification("ESP-NOW init failed", 3, 0);
        return;
    }
    
    Serial.println("ESP-NOW initialized successfully");
    
    esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);
    esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
    
    // Add slave 1 as peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, slave1Address, 6);
    peerInfo.channel = CHANNEL;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("Added Pack 1 peer successfully");
    } else {
        Serial.println("Failed to add Pack 1 peer");
        addFault("Pack 1 peer add failed", 3, 0);
        addNotification("Pack 1 peer add failed", 3, 0);
    }
    
    // Add slave 2 as peer
    memcpy(peerInfo.peer_addr, slave2Address, 6);
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("Added Pack 2 peer successfully");
    } else {
        Serial.println("Failed to add Pack 2 peer (expected if not used)");
    }
    
    Serial.println("=== ESP-NOW Initialization Complete ===\n");
}

void updateCommunication() {
    unsigned long now = millis();
    
    if (slave1Connected && (now - lastDataTime[1] > connectionTimeout)) {
        slave1Connected = false;
        pack1Data.dataValid = false;
        addFault("Pack 1 timeout", 2, 0);
        addNotification("Pack 1 connection lost", 2, 1);
        Serial.println("Pack 1 connection timeout");
        refreshDisplay = true;
    }
    
    if (slave2Connected && (now - lastDataTime[2] > connectionTimeout)) {
        slave2Connected = false;
        pack2Data.dataValid = false;
        addFault("Pack 2 timeout", 2, 0);
        addNotification("Pack 2 connection lost", 2, 2);
        Serial.println("Pack 2 connection timeout");
        refreshDisplay = true;
    }
    
    static unsigned long lastRequest = 0;
    if (now - lastRequest > 5000) {
        if (!slave1Connected || !slave2Connected) {
            uint8_t req[] = {0x01};
            if (!slave1Connected) {
                esp_err_t result = esp_now_send(slave1Address, req, 1);
                Serial.printf("Sent request to Pack 1: %s\n", 
                             result == ESP_OK ? "OK" : "FAILED");
            }
            if (!slave2Connected && memcmp(slave2Address, "\xFF\xFF\xFF\xFF\xFF\xFF", 6) != 0) {
                esp_err_t result = esp_now_send(slave2Address, req, 1);
                Serial.printf("Sent request to Pack 2: %s\n", 
                             result == ESP_OK ? "OK" : "FAILED");
            }
            lastRequest = now;
        }
    }
    
    // Print packet loss counters periodically
    static unsigned long lastLossStats = 0;
    if (now - lastLossStats > 30000) {  // Every 30 seconds
        packetLoss.printCounters();
        lastLossStats = now;
    }
}

void printCommunicationStatus() {
    Serial.println("\n=== COMM STATUS ===");
    Serial.printf("Pack 1: %s, Last: %lus ago, SOC=%.1f%%, SOH=%.1f%%, Charge=%.0fmA, Discharge=%.0fmA\n", 
                  slave1Connected ? "CONNECTED" : "DISCONNECTED", 
                  (millis() - lastDataTime[1]) / 1000,
                  battery1SoC, pack1Data.soh, pack1Data.chargeCurrent, pack1Data.dischargeCurrent);
    Serial.printf("Pack 2: %s, Last: %lus ago, SOC=%.1f%%, SOH=%.1f%%, Charge=%.0fmA, Discharge=%.0fmA\n", 
                  slave2Connected ? "CONNECTED" : "DISCONNECTED", 
                  (millis() - lastDataTime[2]) / 1000,
                  battery2SoC, pack2Data.soh, pack2Data.chargeCurrent, pack2Data.dischargeCurrent);
    
    Serial.printf("Buffer: %d/%d packets\n", commBuffer.available(), commBuffer.capacity());
    
    // Add packet loss counters
    packetLoss.printCounters();
}

// NEW: Function to get latest notification
bool getLatestNotification(Notification& notif) {
    return notificationBuffer.peek(notif, 0);
}

// NEW: Function to clear oldest notification
bool clearOldestNotification() {
    Notification notif;
    return notificationBuffer.pop(notif);
}

// NEW: Function to reset packet loss test
void resetPacketLossTest() {
    packetLoss = PacketLossCounters();
    Serial.println("Packet loss test reset!");
}

// NEW: Function to get packet loss stats for display
float getPacketLossRate(uint8_t packID) {
    return packetLoss.getLossRate(packID);
}

uint32_t getReceivedPackets(uint8_t packID) {
    return packetLoss.packetsReceivedByMaster[packID];
}

uint32_t getMissingPackets(uint8_t packID) {
    return packetLoss.missingPackets[packID];
}

uint32_t getPacketsSentBySlave(uint8_t packID) {
    return packetLoss.packetsSentBySlave[packID];
}