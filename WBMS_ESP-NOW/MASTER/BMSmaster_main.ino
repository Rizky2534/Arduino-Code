#include "display.h"
#include "communication.h"
#include "soc_calculation.h"
#include "soh_calculation.h"
#include <WiFi.h>

// Global SOC/SOH calculator instances
extern SOC_Calculator socCalculatorPack1;
extern SOC_Calculator socCalculatorPack2;
extern SOH_Calculator sohCalculatorPack1;
extern SOH_Calculator sohCalculatorPack2;

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== BMS MASTER STARTING ===");
  
  randomSeed(analogRead(0));
  
  pinMode(BTN_UP,   INPUT_PULLDOWN);
  pinMode(BTN_DOWN, INPUT_PULLDOWN);
  pinMode(BTN_OK,   INPUT_PULLDOWN);
  pinMode(BTN_BACK, INPUT_PULLDOWN);

  displayInit();
  initDisplay();
  
  // Initialize communication
  initCommunication();

  Serial.print("Master MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  // Initialize SOC calculators with smoothing
  socCalculatorPack1.setFilterCoefficient(0.05f);
  socCalculatorPack2.setFilterCoefficient(0.05f);
  
  // Initialize SOH calculators with correct capacity
  sohCalculatorPack1.setCapacity(1.8f);  // Set to 1.8Ah
  sohCalculatorPack2.setCapacity(1.8f);  // Set to 1.8Ah
  
  // Set manufacture date for SOH calculation
  unsigned long oneYearAgo = millis() - (365UL * 24 * 3600 * 1000);
  sohCalculatorPack1.setManufactureDate(oneYearAgo);
  sohCalculatorPack2.setManufactureDate(oneYearAgo);
  
  // Force initial SOH calculation
  float initialSOH1 = sohCalculatorPack1.calculateSOH(0, 25.0f);
  float initialSOH2 = sohCalculatorPack2.calculateSOH(0, 25.0f);
  pack1Data.soh = initialSOH1;
  pack2Data.soh = initialSOH2;
  
  Serial.printf("Initial SOH - Pack 1: %.1f%%, Pack 2: %.1f%%\n", initialSOH1, initialSOH2);
  
  Serial.println("BMS Master Started");
  Serial.println("Ready to receive data from slaves");
  
  // Print OCV table for reference
  Serial.println("Initializing SOC calculation with OCV table:");
  socCalculatorPack1.printOCVTable();
}

void loop() {
  processBufferedPackets();
  
  // Check if display needs refreshing
  if (refreshDisplay) {
    updateDisplay();
    refreshDisplay = false;
  } else {
    updateDisplay();
  }
  
  updateCommunication();
  
  // Backup: Sync display SOC from pack data
  battery1SoC = pack1Data.soc;
  battery2SoC = pack2Data.soc;
  
  // Periodic status print
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint > 10000) {
    printCommunicationStatus();
    
    // Print packet loss counters
    packetLoss.printCounters();
    
    // Print SOC/SOH status
    Serial.println("=== SOC/SOH Status ===");
    Serial.printf("Pack 1: SOC=%.1f%%, Display SOC=%.1f%%, SOH=%.1f%%\n", 
                  pack1Data.soc, battery1SoC, pack1Data.soh);
    Serial.printf("Pack 2: SOC=%.1f%%, Display SOC=%.1f%%, SOH=%.1f%%\n", 
                  pack2Data.soc, battery2SoC, pack2Data.soh);
    
    lastStatusPrint = millis();
  }
  
  delay(10);
}