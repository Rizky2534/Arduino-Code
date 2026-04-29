#ifndef DISPLAY_H
#define DISPLAY_H

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "bms_circular_buffer.h"

struct SlaveData;

struct FaultEntry {
  char description[50];
  uint8_t severity;
  uint8_t pack;
  unsigned long timestamp;
  bool active;
};

struct PackData {
  float voltage;           // V
  float chargeCurrent;     // Charge current in mA
  float dischargeCurrent;  // Discharge current in mA
  float soc;               // %
  float soh;               // %
  float temperature;       // °C
  uint32_t timestamp;
  bool dataValid;
};

extern Adafruit_ST7735 tft;

// Display refresh control
extern volatile bool refreshDisplay;
#define DISPLAY_REFRESH_INTERVAL 1000

// Notification display flag
extern volatile bool showNotification;
extern unsigned long notificationStartTime;
#define NOTIFICATION_DISPLAY_TIME 5000

// Notification buffer
extern NotificationBuffer notificationBuffer;

// Notification functions
bool getLatestNotification(Notification& notif);
bool clearOldestNotification();

extern float battery1SoC;
extern float battery2SoC;
extern float packVoltage[2];
extern float packCurrent[2];
extern float packTemp[2];
extern float packSoH[2];
extern bool slave1Connected;
extern bool slave2Connected;
extern PackData pack1Data;
extern PackData pack2Data;

void displayInit();
void displayOff();
void displayOn();
bool isDisplayOff();

void initDisplay();
void updateDisplay();

void drawHomeScreen();
void drawMenu(const char* items[], int size, int selection, const char* title);
void drawPackDetail(int pack, int item);
void drawFaultLog();
void drawNotification(const Notification& notif);
void clearNotificationDisplay();

void initFaultLog();
void addFault(const char* faultDescription, int severity, int pack = 0);
void clearFaults();
uint16_t getSeverityColor(int severity);
const char* getPackName(int pack);
int getFaultCount();
FaultEntry getFault(int index);

#define BTN_UP    32
#define BTN_DOWN  33
#define BTN_OK    25
#define BTN_BACK  26

#endif