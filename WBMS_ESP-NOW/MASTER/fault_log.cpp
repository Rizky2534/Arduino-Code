#include "display.h"
#include "bms_circular_buffer.h"
#include <EEPROM.h>

static BMSFaultBuffer faultBuffer;

void initFaultLog() {
  EEPROM.begin(512);
  
  int storedCount = EEPROM.read(0);
  if (storedCount > 20) storedCount = 0;
  
  for (int i = 0; i < storedCount; i++) {
    BMSFaultEntry fault;
    EEPROM.get(1 + i * sizeof(BMSFaultEntry), fault);
    faultBuffer.push(fault);
  }
}

// Helper function to print fault
void printBMSFault(const BMSFaultEntry& fault) {
  Serial.printf("Fault: %s (Severity: %d, Pack: %d, Time: %lu)\n", 
                fault.description, fault.severity, fault.pack, fault.timestamp);
}

void addFault(const char* faultDescription, int severity, int pack) {
  BMSFaultEntry fault(faultDescription, severity, pack);
  faultBuffer.push(fault);
  
  EEPROM.write(0, faultBuffer.available());
  
  for (int i = 0; i < faultBuffer.available(); i++) {
    BMSFaultEntry f;
    if (faultBuffer.peek(f, i)) {
      EEPROM.put(1 + i * sizeof(BMSFaultEntry), f);
    }
  }
  EEPROM.commit();
  
  Serial.print("FAULT BUFFERED: ");
  printBMSFault(fault);
}

void clearFaults() {
  faultBuffer.clear();
  EEPROM.write(0, 0);
  EEPROM.commit();
}

int getFaultCount() {
  return faultBuffer.available();
}

FaultEntry getFault(int index) {
  FaultEntry result;
  BMSFaultEntry fault;
  if (faultBuffer.peek(fault, index)) {
    strncpy(result.description, fault.description, 50);
    result.severity = fault.severity;
    result.pack = fault.pack;
    result.timestamp = fault.timestamp;
    result.active = fault.active;
  } else {
    result.description[0] = '\0';
    result.severity = 0;
    result.pack = 0;
    result.timestamp = 0;
    result.active = false;
  }
  return result;
}

uint16_t getSeverityColor(int severity) {
  switch (severity) {
    case 1: return ST77XX_BLUE;
    case 2: return ST77XX_YELLOW;
    case 3: return ST77XX_ORANGE;
    case 4: return ST77XX_RED;
    default: return ST77XX_WHITE;
  }
}

const char* getPackName(int pack) {
  switch (pack) {
    case 0: return "SYS";
    case 1: return "P1";
    case 2: return "P2";
    default: return "UNK";
  }
}