#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include "bms_circular_buffer.h"

#pragma pack(push, 1)
struct SlaveData {
  uint8_t senderID;        // 1 byte
  float voltage;          // 4 bytes (V)
  float current1;         // 4 bytes (mA - Charge current)
  float current2;         // 4 bytes (mA - Discharge current)
  float temperature;      // 4 bytes (°C)
  uint16_t protectionFlags; // 2 bytes
  uint32_t sequence;      // 4 bytes - Sequence number from slave
  uint32_t timestamp;     // 4 bytes
  uint16_t checksum;      // 2 bytes
  // TOTAL: 1+4+4+4+4+2+4+4+2 = 29 bytes
  
  // Protection flags bit positions
  enum ProtectionBits {
    OVER_VOLTAGE       = 0,
    UNDER_VOLTAGE      = 1,
    OVER_CURRENT_CHG   = 2,  // Over current during charging
    OVER_CURRENT_DIS   = 3,  // Over current during discharging
    OVER_TEMPERATURE   = 4,
    UNDER_TEMPERATURE  = 5,
    SHORT_CIRCUIT      = 6,
    FET_OVER_TEMP      = 7,
    COMMUNICATION_FAIL = 8,
    RESERVED_1         = 9,
    RESERVED_2         = 10,
    RESERVED_3         = 11,
    RESERVED_4         = 12,
    RESERVED_5         = 13,
    SYSTEM_FAULT       = 14
  };
  
  // Helper function to check if a protection is triggered
  bool isProtectionTriggered(ProtectionBits bit) const {
    return (protectionFlags & (1 << bit)) != 0;
  }
  
  // Helper function to get protection status as string
  const char* getProtectionStatus() const {
    static char status[100];
    status[0] = '\0';
    
    if (protectionFlags == 0) {
      return "NORMAL";
    }
    
    if (isProtectionTriggered(OVER_VOLTAGE)) strcat(status, "OV ");
    if (isProtectionTriggered(UNDER_VOLTAGE)) strcat(status, "UV ");
    if (isProtectionTriggered(OVER_CURRENT_CHG)) strcat(status, "OCC ");
    if (isProtectionTriggered(OVER_CURRENT_DIS)) strcat(status, "OCD ");
    if (isProtectionTriggered(OVER_TEMPERATURE)) strcat(status, "OT ");
    if (isProtectionTriggered(UNDER_TEMPERATURE)) strcat(status, "UT ");
    if (isProtectionTriggered(SHORT_CIRCUIT)) strcat(status, "SC ");
    if (isProtectionTriggered(FET_OVER_TEMP)) strcat(status, "FET_OT ");
    if (isProtectionTriggered(COMMUNICATION_FAIL)) strcat(status, "COMM ");
    if (isProtectionTriggered(SYSTEM_FAULT)) strcat(status, "SYS ");
    
    // Remove trailing space if present
    int len = strlen(status);
    if (len > 0 && status[len-1] == ' ') {
      status[len-1] = '\0';
    }
    
    return status;
  }
};
#pragma pack(pop)

// SIMPLE COUNTERS FOR PACKET LOSS TESTING
struct PacketLossCounters {
  uint32_t packetsSentBySlave[3];     // Last sequence number from slave (indicates total sent)
  uint32_t packetsReceivedByMaster[3]; // Total packets received by master
  uint32_t lastSequence[3];            // Last sequence received
  uint32_t missingPackets[3];          // Simple count of missing packets
  
  PacketLossCounters() {
    for (int i = 0; i < 3; i++) {
      packetsSentBySlave[i] = 0;
      packetsReceivedByMaster[i] = 0;
      lastSequence[i] = 0;
      missingPackets[i] = 0;
    }
  }
  
  // Simple function to update counters
  void updateCounters(uint8_t packID, uint32_t sequence) {
    if (packID == 0 || packID > 2) return;
    
    packetsReceivedByMaster[packID]++;
    
    // If this is first packet, just store sequence
    if (lastSequence[packID] == 0) {
      lastSequence[packID] = sequence;
      packetsSentBySlave[packID] = sequence;
      Serial.printf("[INIT] Pack %d first sequence: %u\n", packID, sequence);
      return;
    }
    
    // Update packets sent by slave (keep the highest sequence seen)
    if (sequence > packetsSentBySlave[packID]) {
      packetsSentBySlave[packID] = sequence;
    }
    
    // Check for missing packets
    if (sequence > lastSequence[packID] + 1) {
      uint32_t gap = sequence - lastSequence[packID] - 1;
      missingPackets[packID] += gap;
      Serial.printf("[LOSS] Pack %d: Lost %u packets (seq %u to %u)\n", 
                   packID, gap, lastSequence[packID] + 1, sequence - 1);
    } else if (sequence < lastSequence[packID]) {
      Serial.printf("[OUT OF ORDER] Pack %d: Got seq %u, last was %u\n", 
                   packID, sequence, lastSequence[packID]);
    }
    
    lastSequence[packID] = sequence;
  }
  
  // Calculate loss rate
  float getLossRate(uint8_t packID) {
    if (packetsSentBySlave[packID] == 0) return 0;
    return 100.0f * missingPackets[packID] / packetsSentBySlave[packID];
  }
  
  // Print counters
  void printCounters() {
    Serial.println("\n=== PACKET LOSS COUNTERS ===");
    for (int i = 1; i <= 2; i++) {
      Serial.printf("Pack %d:\n", i);
      Serial.printf("  Slave sent (last seq): %u packets\n", packetsSentBySlave[i]);
      Serial.printf("  Master received: %u packets\n", packetsReceivedByMaster[i]);
      Serial.printf("  Missing: %u packets\n", missingPackets[i]);
      Serial.printf("  Expected total: %u\n", packetsSentBySlave[i]);
      Serial.printf("  Loss rate: %.2f%%\n", getLossRate(i));
    }
    Serial.println("============================\n");
  }
};

// Function prototypes
void initCommunication();
void updateCommunication();
void processReceivedData(uint8_t* data, int len, uint8_t senderID);
void printCommunicationStatus();
void processBufferedPackets();
void handleProtectionNotifications(uint8_t packID, uint16_t protectionFlags);

// Global variables
extern BMSCommBuffer commBuffer;
extern bool slave1Connected;
extern bool slave2Connected;
extern PacketLossCounters packetLoss;  // SIMPLE: Counter for packet loss testing

#endif