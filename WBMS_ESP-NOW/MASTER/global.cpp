#include "display.h"
#include "bms_circular_buffer.h"

Adafruit_ST7735 tft(15, 2, 4);

// Display refresh flag
volatile bool refreshDisplay = true;

// Notification buffer - defined ONCE here
NotificationBuffer notificationBuffer;

// Notification display variables - defined ONCE here
volatile bool showNotification = false;
unsigned long notificationStartTime = 0;

// Current notification being displayed
Notification currentNotification;

float battery1SoC = 0.0;
float battery2SoC = 0.0;
float packVoltage[2] = {0.0, 0.0};
float packCurrent[2] = {0.0, 0.0};
float packTemp[2] = {0.0, 0.0};
float packSoH[2] = {0.0, 0.0};
bool slave1Connected = false;
bool slave2Connected = false;

PackData pack1Data = {0, 0, 0, 0, 0, 0, 0, false};
PackData pack2Data = {0, 0, 0, 0, 0, 0, 0, false};