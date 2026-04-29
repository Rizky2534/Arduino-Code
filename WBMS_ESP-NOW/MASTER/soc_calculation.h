#ifndef SOC_CALCULATION_H
#define SOC_CALCULATION_H

#include <Arduino.h>

// ================= OCV-SOC LOOKUP TABLE =================
// LFP Battery SOC Table (5 cells in series)
// Cell voltage range: 2.8V (0%) to 3.6V (100%)
// Pack voltage range: 14.0V (0%) to 18.0V (100%)
// Note: LFP batteries have a very flat voltage curve between 20%-80% SOC

const float OCV_TABLE_VOLTAGE[] = {
  18.00, 17.95, 17.90, 17.85, 17.80, 17.75, 17.70, 17.65, 17.60, 17.55, 
  17.50, 17.45, 17.40, 17.35, 17.30, 17.25, 17.20, 17.15, 17.10, 17.05, 
  17.00, 16.95, 16.90, 16.85, 16.80, 16.75, 16.70, 16.65, 16.60, 16.55, 
  16.50, 16.45, 16.40, 16.35, 16.30, 16.25, 16.20, 16.15, 16.10, 16.05, 
  16.00, 15.95, 15.90, 15.85, 15.80, 15.75, 15.70, 15.65, 15.60, 15.55, 
  15.50, 15.45, 15.40, 15.35, 15.30, 15.25, 15.20, 15.15, 15.10, 15.05, 
  15.00, 14.95, 14.90, 14.85, 14.80, 14.75, 14.70, 14.65, 14.60, 14.55, 
  14.50, 14.45, 14.40, 14.35, 14.30, 14.25, 14.20, 14.15, 14.10, 14.05, 
  14.00, 13.90, 13.80, 13.70, 13.60, 13.50, 13.40, 13.30, 13.20, 13.10, 
  13.00, 12.90, 12.80, 12.70, 12.60, 12.50, 12.40, 12.30, 12.20, 12.10, 
  12.00  
};

const float OCV_TABLE_SOC[] = {
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

// Size of the OCV table
const int OCV_TABLE_SIZE = sizeof(OCV_TABLE_VOLTAGE) / sizeof(OCV_TABLE_VOLTAGE[0]);

// ================= SOC CALCULATION CLASS =================
class SOC_Calculator {
  private:
    float _lastSOC;
    unsigned long _lastUpdateTime;
    float _coulombCounter;  // Ah
    float _capacity;        // Battery capacity in Ah
    float _tempCompensationFactor;
    
    // Low-pass filter for SOC smoothing
    float _filteredSOC;
    float _alpha;  // Filter coefficient (0.0 to 1.0)
    
    bool _isCharging;
    float _chargeEfficiency;
    float _dischargeEfficiency;
    
    // Current integration tracking
    float _integratedCurrentAh;
    unsigned long _lastIntegrationTime;
    
  public:
    SOC_Calculator(float initialCapacity = 1.8f) :  // Default 1.8Ah battery for your 5S pack
      _lastSOC(50.0f),  // Start at 50%
      _lastUpdateTime(millis()),
      _coulombCounter(0.0f),
      _capacity(initialCapacity),
      _tempCompensationFactor(1.0f),
      _filteredSOC(50.0f),
      _alpha(0.05f),  // Smoothing factor (5% - adapts slowly)
      _isCharging(false),
      _chargeEfficiency(0.98f),    // 98% charging efficiency
      _dischargeEfficiency(0.99f),  // 99% discharging efficiency
      _integratedCurrentAh(0.0f),
      _lastIntegrationTime(millis())
    {}
    
    // Direct adaptation of your estimateSoC function
    float estimateSoCFromOCV(float batteryVoltage) {
      // If battery voltage is below minimum, set SoC to 0%
      if (batteryVoltage < OCV_TABLE_VOLTAGE[OCV_TABLE_SIZE - 1]) {
        return 0.0f;
      }
      
      // Check for out-of-range voltage conditions
      if (batteryVoltage >= OCV_TABLE_VOLTAGE[0]) {
        return OCV_TABLE_SOC[0];  // Maximum SoC (100%)
      }
      
      // Find the two closest points in ocvTable for interpolation
      for (int i = 0; i < OCV_TABLE_SIZE - 1; i++) {
        if (batteryVoltage <= OCV_TABLE_VOLTAGE[i] && batteryVoltage > OCV_TABLE_VOLTAGE[i + 1]) {
          float voltageDifference = OCV_TABLE_VOLTAGE[i] - OCV_TABLE_VOLTAGE[i + 1];
          float socDifference = OCV_TABLE_SOC[i] - OCV_TABLE_SOC[i + 1];
          
          // Linear interpolation (same as your code)
          float fraction = (batteryVoltage - OCV_TABLE_VOLTAGE[i + 1]) / voltageDifference;
          return OCV_TABLE_SOC[i + 1] + fraction * socDifference;
        }
      }
      
      return OCV_TABLE_SOC[OCV_TABLE_SIZE - 1]; // Fallback for unexpected cases
    }
    
    // Enhanced version with temperature compensation
    float calculateSOCfromOCV(float voltage, float temperature = 25.0f) {
      // Temperature compensation (LiFePO4 voltage changes with temperature)
      float tempCompensatedVoltage = voltage;
      
      if (temperature != 25.0f) {
        // LiFePO4 voltage coefficient: approximately -0.3mV/°C per cell
        // For 5 cells in series: -1.5mV/°C for 5S system
        float tempCoefficient = -0.0015f; // V/°C for 5S system
        tempCompensatedVoltage += tempCoefficient * (temperature - 25.0f);
      }
      
      // Get SOC from OCV table
      float rawSOC = estimateSoCFromOCV(tempCompensatedVoltage);
      
      // Apply low-pass filter for smooth SOC display
      _filteredSOC = _filteredSOC + _alpha * (rawSOC - _filteredSOC);
      
      return constrain(_filteredSOC, 0.0f, 100.0f);
    }
    
    // Coulomb counting method - adapted from your current integration
    float updateSOCbyCoulombCounting(float current, unsigned long currentTime) {
      if (_lastIntegrationTime == 0) {
        _lastIntegrationTime = currentTime;
        return _lastSOC;
      }
      
      // Calculate time delta in hours
      float dtHours = (currentTime - _lastIntegrationTime) / 3600000.0f; // ms to hours
      
      if (dtHours <= 0) {
        return _lastSOC;
      }
      
      // Calculate charge/discharge in Ah
      float deltaAh = current * dtHours; // Current is already in A (not mA)
      
      // Apply efficiency factors
      if (current > 0) {  // Charging
        deltaAh *= _chargeEfficiency;
        _isCharging = true;
      } else {  // Discharging
        deltaAh /= _dischargeEfficiency;
        _isCharging = false;
      }
      
      // Update integrated current
      _integratedCurrentAh += deltaAh;
      
      // Update coulomb counter (relative to capacity)
      _coulombCounter = (_integratedCurrentAh / _capacity) * 100.0f;
      
      // Calculate SOC from coulomb counter
      float socFromCC = 50.0f + _coulombCounter; // Assuming starting from 50%
      socFromCC = constrain(socFromCC, 0.0f, 100.0f);
      
      _lastSOC = socFromCC;
      _lastIntegrationTime = currentTime;
      
      return _lastSOC;
    }
    
    // Hybrid SOC estimation (uses OCV when idle, coulomb counting when active)
    float estimateSOC(float voltage, float current, float temperature = 25.0f, bool forceOCV = false) {
      unsigned long now = millis();
      
      // Check if battery is "resting" (low current)
      bool isResting = (fabs(current) < 0.1f);  // Less than 100mA
      
      if (forceOCV || isResting) {
        // Use OCV method when resting or forced
        float ocvSOC = calculateSOCfromOCV(voltage, temperature);
        
        // Reset coulomb counter to match OCV SOC for smooth transition
        _integratedCurrentAh = (ocvSOC - 50.0f) * _capacity / 100.0f;
        _lastSOC = ocvSOC;
        _lastIntegrationTime = now;
      } else {
        // Use coulomb counting for active periods
        _lastSOC = updateSOCbyCoulombCounting(current, now);
      }
      
      return _lastSOC;
    }
    
    // Reset SOC to a specific value
    void resetSOC(float newSOC, float currentVoltage = 0.0f) {
      _lastSOC = constrain(newSOC, 0.0f, 100.0f);
      _filteredSOC = _lastSOC;
      
      if (currentVoltage > 0) {
        // Reset based on voltage
        _integratedCurrentAh = (estimateSoCFromOCV(currentVoltage) - 50.0f) * _capacity / 100.0f;
      } else {
        // Reset based on SOC percentage
        _integratedCurrentAh = (_lastSOC - 50.0f) * _capacity / 100.0f;
      }
    }
    
    // Set battery capacity
    void setCapacity(float capacityAh) {
      _capacity = capacityAh;
      // Adjust integrated current to maintain same SOC
      float socPercent = _lastSOC;
      _integratedCurrentAh = (socPercent - 50.0f) * _capacity / 100.0f;
    }
    
    // Get current SOC
    float getSOC() const {
      return _lastSOC;
    }
    
    // Get filtered SOC
    float getFilteredSOC() const {
      return _filteredSOC;
    }
    
    // Get integrated current in Ah
    float getIntegratedCurrentAh() const {
      return _integratedCurrentAh;
    }
    
    // Set filter coefficient (0.0 to 1.0)
    void setFilterCoefficient(float alpha) {
      _alpha = constrain(alpha, 0.0f, 1.0f);
    }
    
    // Check if battery is charging
    bool isCharging() const {
      return _isCharging;
    }
    
    // Get battery capacity
    float getCapacity() const {
      return _capacity;
    }
    
    // Print OCV table for debugging
    void printOCVTable() {
      Serial.println("=== OCV-SOC Lookup Table (5S LiFePO4 14.5V-18.0V) ===");
      Serial.println("Voltage(V)  ->  SOC(%)");
      Serial.println("-----------------------");
      
      for (int i = 0; i < OCV_TABLE_SIZE; i++) {
        Serial.printf("%6.2f V  -> %5.1f%%\n", 
                     OCV_TABLE_VOLTAGE[i], 
                     OCV_TABLE_SOC[i]);
        
        // Print every 5th entry to keep output manageable
        if ((i + 1) % 5 == 0 && i < OCV_TABLE_SIZE - 1) {
          Serial.println("...");
        }
      }
      Serial.println("================================");
    }
    
    // Print current SOC status
    void printStatus() {
      Serial.println("=== SOC Calculator Status ===");
      Serial.printf("Current SOC: %.1f%%\n", _lastSOC);
      Serial.printf("Filtered SOC: %.1f%%\n", _filteredSOC);
      Serial.printf("Battery Capacity: %.3f Ah\n", _capacity);  // Changed to 3 decimal for 1.8Ah
      Serial.printf("Integrated Current: %.3f Ah\n", _integratedCurrentAh);
      Serial.printf("Is Charging: %s\n", _isCharging ? "Yes" : "No");
      Serial.printf("Filter Coefficient: %.3f\n", _alpha);
      Serial.println("=============================");
    }
    
    // Simple voltage-based SOC (direct port of your function)
    float simpleEstimateSOC(float batteryVoltage) {
      return estimateSoCFromOCV(batteryVoltage);
    }
    
    // Get OCV table voltage at specific SOC percentage
    float getVoltageAtSOC(float soc) {
      soc = constrain(soc, 0.0f, 100.0f);
      
      // Find the two closest SOC points for interpolation
      for (int i = 0; i < OCV_TABLE_SIZE - 1; i++) {
        if (soc <= OCV_TABLE_SOC[i] && soc > OCV_TABLE_SOC[i + 1]) {
          float socDifference = OCV_TABLE_SOC[i] - OCV_TABLE_SOC[i + 1];
          float voltageDifference = OCV_TABLE_VOLTAGE[i] - OCV_TABLE_VOLTAGE[i + 1];
          
          // Linear interpolation
          float fraction = (soc - OCV_TABLE_SOC[i + 1]) / socDifference;
          return OCV_TABLE_VOLTAGE[i + 1] + fraction * voltageDifference;
        }
      }
      
      // Handle edge cases
      if (soc >= OCV_TABLE_SOC[0]) return OCV_TABLE_VOLTAGE[0];
      if (soc <= OCV_TABLE_SOC[OCV_TABLE_SIZE - 1]) return OCV_TABLE_VOLTAGE[OCV_TABLE_SIZE - 1];
      
      return 0.0f;
    }
};

#endif // SOC_CALCULATION_H