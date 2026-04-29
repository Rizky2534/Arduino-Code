#ifndef BMS_CIRCULAR_BUFFER_H
#define BMS_CIRCULAR_BUFFER_H

#include <Arduino.h>
#include <string.h>

// ================= SIMPLIFIED CIRCULAR BUFFER =================
template<typename T, uint16_t BUFFER_SIZE>
class BMSCircularBuffer {
  public:
    // ===== BASIC OPERATIONS =====
    void push(const T& value) {
      if (_available < BUFFER_SIZE) {
        _buffer[_tail] = value;
        _tail = (_tail + 1) % BUFFER_SIZE;
        _available++;
      } else {
        // Buffer full - overwrite oldest
        _buffer[_head] = value;
        _head = (_head + 1) % BUFFER_SIZE;
        _tail = (_tail + 1) % BUFFER_SIZE;
      }
    }
    
    bool pop(T& value) {
      if (_available == 0) return false;
      
      value = _buffer[_head];
      _head = (_head + 1) % BUFFER_SIZE;
      _available--;
      return true;
    }
    
    bool peek(T& value, uint16_t offset = 0) const {
      if (offset >= _available) return false;
      
      uint16_t index = (_head + offset) % BUFFER_SIZE;
      value = _buffer[index];
      return true;
    }
    
    // ===== BUFFER MANAGEMENT =====
    void clear() {
      _head = 0;
      _tail = 0;
      _available = 0;
    }
    
    bool isFull() const { return _available >= BUFFER_SIZE; }
    bool isEmpty() const { return _available == 0; }
    uint16_t available() const { return _available; }
    uint16_t freeSpace() const { return BUFFER_SIZE - _available; }
    uint16_t capacity() const { return BUFFER_SIZE; }

  private:
    T _buffer[BUFFER_SIZE];
    volatile uint16_t _head = 0;
    volatile uint16_t _tail = 0;
    volatile uint16_t _available = 0;
};

// ================= BMS PACKET STRUCTURE =================
struct BMSCommPacket {
  uint8_t senderID;
  uint32_t sequence;
  float voltage;           // V
  float chargeCurrent;     // Charge current in mA
  float dischargeCurrent;  // Discharge current in mA
  float soc;               // %
  float soh;               // %
  float temperature;       // °C
  uint32_t timestamp;
  
  BMSCommPacket() : senderID(0), sequence(0), voltage(0), chargeCurrent(0), 
                    dischargeCurrent(0), soc(0), soh(0), temperature(0), timestamp(0) {
  }
};

typedef BMSCircularBuffer<BMSCommPacket, 20> BMSCommBuffer;

// ================= FAULT ENTRY STRUCTURE =================
struct BMSFaultEntry {
  char description[50];
  uint8_t severity;
  uint8_t pack;
  uint32_t timestamp;
  bool active;
  
  BMSFaultEntry() : severity(0), pack(0), timestamp(0), active(false) {
    description[0] = '\0';
  }
  
  BMSFaultEntry(const char* desc, uint8_t sev, uint8_t p) : 
    severity(sev), pack(p), timestamp(millis()), active(true) {
    strncpy(description, desc, 49);
    description[49] = '\0';
  }
};

typedef BMSCircularBuffer<BMSFaultEntry, 20> BMSFaultBuffer;

// ================= NOTIFICATION BUFFER =================
struct Notification {
  char message[60];
  uint8_t severity;
  uint8_t packID;
  uint32_t timestamp;
  bool active;
  
  Notification() : severity(0), packID(0), timestamp(0), active(false) {
    message[0] = '\0';
  }
  
  Notification(const char* msg, uint8_t sev, uint8_t pack) : 
    severity(sev), packID(pack), timestamp(millis()), active(true) {
    strncpy(message, msg, 59);
    message[59] = '\0';
  }
};

typedef BMSCircularBuffer<Notification, 10> NotificationBuffer;

#endif // BMS_CIRCULAR_BUFFER_H