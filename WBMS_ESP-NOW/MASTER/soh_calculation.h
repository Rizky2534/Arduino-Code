#ifndef SOH_CALCULATION_H
#define SOH_CALCULATION_H

#include <Arduino.h>

// ================= SOH CALCULATION CLASS =================
class SOH_Calculator {
  private:
    float _nominalCapacity;     // Original battery capacity (Ah)
    float _estimatedCapacity;   // Current estimated capacity (Ah)
    float _soh;                 // State of Health (0-100%)
    
    // Cycle counting
    unsigned int _cycleCount;
    float _cumulativeAhThroughput;
    
    // Time-based aging
    unsigned long _startTime;
    float _calendarAgingFactor;
    
    // Simple capacity fade tracking
    float _capacityFadeRate;  // % per cycle
    
  public:
    SOH_Calculator(float nominalCapacity = 1.8f) :  // CHANGED: Default to 1.8Ah for your battery
      _nominalCapacity(nominalCapacity),
      _estimatedCapacity(nominalCapacity),
      _soh(100.0f),
      _cycleCount(0),
      _cumulativeAhThroughput(0.0f),
      _startTime(millis()),
      _calendarAgingFactor(1.0f),
      _capacityFadeRate(0.03f)  // 0.03% per cycle for LiFePO4
    {}
    
    // Set manufacture date (for simulating aged batteries)
    void setManufactureDate(unsigned long manufactureMillis) {
        _startTime = manufactureMillis;
    }
    
    // Set battery capacity
    void setCapacity(float capacityAh) {
        _nominalCapacity = capacityAh;
        _estimatedCapacity = capacityAh;
        _soh = 100.0f;
    }
    
    // Simple SOH calculation based on cycles and time
    float calculateSOH(float cycles = 0, float temperature = 25.0f) {
      unsigned long currentTime = millis();
      float ageHours = (currentTime - _startTime) / 3600000.0f; // hours
      
      // 1. Cycle-based aging
      if (cycles > 0) {
        _cycleCount = cycles;
      }
      
      float cycleAging = 100.0f - (_cycleCount * _capacityFadeRate);
      cycleAging = max(cycleAging, 80.0f);  // Minimum 80% after cycles
      
      // 2. Calendar aging (LiFePO4: ~2% per year)
      float ageYears = ageHours / (24.0f * 365.0f);
      float calendarAging = 100.0f - (ageYears * 2.0f);
      calendarAging = max(calendarAging, 70.0f);  // Minimum 70% after calendar aging
      
      // 3. Temperature effect
      float tempFactor = 1.0f;
      if (temperature > 40.0f) {
        tempFactor = 1.0f - ((temperature - 40.0f) * 0.001f); // 0.1% degradation per °C above 40
      }
      
      // Combine all factors
      _soh = cycleAging * calendarAging * tempFactor / 100.0f;
      _soh = constrain(_soh, 0.0f, 100.0f);
      
      // Update estimated capacity
      _estimatedCapacity = _nominalCapacity * (_soh / 100.0f);
      
      return _soh;
    }
    
    // Update with capacity measurement
    void updateWithCapacity(float measuredCapacityAh) {
      _estimatedCapacity = measuredCapacityAh;
      _soh = (_estimatedCapacity / _nominalCapacity) * 100.0f;
      _soh = constrain(_soh, 0.0f, 100.0f);
    }
    
    // Add cycle based on charge throughput
    void addChargeThroughput(float ahThroughput) {
      _cumulativeAhThroughput += fabs(ahThroughput);
      
      // Assume one full cycle for every 2*capacity Ah throughput
      float cyclesFromThroughput = _cumulativeAhThroughput / (2.0f * _nominalCapacity);
      
      if (cyclesFromThroughput > _cycleCount) {
        _cycleCount = cyclesFromThroughput;
        calculateSOH(_cycleCount);
      }
    }
    
    // Get current SOH
    float getSOH() const {
      return _soh;
    }
    
    // Get estimated capacity
    float getEstimatedCapacity() const {
      return _estimatedCapacity;
    }
    
    // Get cycle count
    unsigned int getCycleCount() const {
      return _cycleCount;
    }
    
    // Get cumulative Ah throughput
    float getCumulativeAhThroughput() const {
      return _cumulativeAhThroughput;
    }
    
    // Reset calculator
    void reset(float newCapacityAh = 1.8f) {  // CHANGED: Default to 1.8Ah
      _nominalCapacity = newCapacityAh;
      _estimatedCapacity = newCapacityAh;
      _soh = 100.0f;
      _cycleCount = 0;
      _cumulativeAhThroughput = 0.0f;
      _startTime = millis();
      _calendarAgingFactor = 1.0f;
    }
    
    // Print SOH status
    void printStatus() {
      Serial.println("=== SOH Calculator Status ===");
      Serial.printf("SOH: %.1f%%\n", _soh);
      Serial.printf("Nominal Capacity: %.3f Ah\n", _nominalCapacity);
      Serial.printf("Estimated Capacity: %.3f Ah\n", _estimatedCapacity);
      Serial.printf("Cycle Count: %d\n", _cycleCount);
      Serial.printf("Cumulative Throughput: %.3f Ah\n", _cumulativeAhThroughput);
      
      unsigned long ageHours = (millis() - _startTime) / 3600000;
      Serial.printf("Operating Time: %lu hours\n", ageHours);
      Serial.println("=============================");
    }
};

#endif // SOH_CALCULATION_H