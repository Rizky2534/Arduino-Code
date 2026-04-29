#ifndef PACKET_LOSS_TEST_H
#define PACKET_LOSS_TEST_H

#include <Arduino.h>

// Simple packet loss test structure (optional - not required for main code)
struct SimplePacketLossTest {
    uint32_t startTime;
    uint32_t expectedPackets;
    uint32_t receivedPackets;
    
    SimplePacketLossTest() {
        reset();
    }
    
    void reset() {
        startTime = millis();
        expectedPackets = 0;
        receivedPackets = 0;
    }
    
    void startTest() {
        reset();
    }
    
    void packetReceived() {
        receivedPackets++;
    }
    
    void updateExpected(uint32_t expected) {
        expectedPackets = expected;
    }
    
    float getLossRate() {
        if (expectedPackets == 0) return 0;
        return 100.0f * (expectedPackets - receivedPackets) / expectedPackets;
    }
    
    void printResults() {
        unsigned long elapsed = (millis() - startTime) / 1000;
        Serial.println("\n=== PACKET LOSS TEST RESULTS ===");
        Serial.printf("Test duration: %lu seconds\n", elapsed);
        Serial.printf("Expected packets: %u\n", expectedPackets);
        Serial.printf("Received packets: %u\n", receivedPackets);
        Serial.printf("Lost packets: %u\n", expectedPackets - receivedPackets);
        Serial.printf("Loss rate: %.2f%%\n", getLossRate());
        Serial.println("==================================\n");
    }
};

#endif