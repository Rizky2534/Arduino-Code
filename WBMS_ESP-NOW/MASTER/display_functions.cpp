#include <Arduino.h>
#include "display.h"
#include "communication.h"

// External variables from global.cpp
extern volatile bool showNotification;
extern unsigned long notificationStartTime;
extern NotificationBuffer notificationBuffer;
extern Notification currentNotification;  // ADD THIS LINE

enum ScreenState {
  SCREEN_HOME,
  SCREEN_MAIN_MENU,
  SCREEN_PACK_MENU,
  SCREEN_PACK_DETAIL,
  SCREEN_FAULT_LOG,
  SCREEN_POWER_OFF
};

ScreenState currentScreen = SCREEN_HOME;

int homeSelection     = 0;
int mainMenuSelection = 0;
int packMenuSelection = 0;
int activePack        = 1;

int scrollOffset = 0;
const int MAX_LINES_DISPLAY = 5;
unsigned long scrollDelay = 0;
const unsigned long SCROLL_DELAY = 200;

unsigned long okPressStart = 0;
const unsigned long WAKE_TIME = 5000;

const char* mainMenu[] = {
  "Pack 1",
  "Pack 2",
  "Fault Log"
};
const int mainMenuSize = 3;

const char* packMenu[] = {
  "Pack Voltage",
  "Pack Current",
  "Temperature",
  "System Status",
  "SoH"
};
const int packMenuSize = 5;

String createFaultMessage(FaultEntry fault);
String truncateText(String text, int maxWidth);
void resetScroll();

void initDisplay() {
  initFaultLog();
  
  addFault("System Started", 1, 0);
  
  delay(100);
  
  drawHomeScreen();
}

void resetScroll() {
  scrollOffset = 0;
  scrollDelay = 0;
}

bool handleMenuScrolling(int& selection, int menuSize, int maxDisplayLines) {
  unsigned long currentTime = millis();
  bool redrawNeeded = false;
  
  if (digitalRead(BTN_UP) && currentTime - scrollDelay > SCROLL_DELAY) {
    if (selection > 0) {
      selection--;
      redrawNeeded = true;
      
      if (selection < scrollOffset) {
        scrollOffset = selection;
      }
      
      scrollDelay = currentTime;
    }
  }
  
  if (digitalRead(BTN_DOWN) && currentTime - scrollDelay > SCROLL_DELAY) {
    if (selection < menuSize - 1) {
      selection++;
      redrawNeeded = true;
      
      if (selection >= scrollOffset + maxDisplayLines) {
        scrollOffset = selection - maxDisplayLines + 1;
      }
      
      scrollDelay = currentTime;
    }
  }
  
  return redrawNeeded;
}

bool handleInfoScrolling(int totalLines, int maxDisplayLines) {
  unsigned long currentTime = millis();
  bool redrawNeeded = false;
  
  if (digitalRead(BTN_UP) && currentTime - scrollDelay > SCROLL_DELAY) {
    if (scrollOffset > 0) {
      scrollOffset--;
      redrawNeeded = true;
      scrollDelay = currentTime;
    }
  }
  
  if (digitalRead(BTN_DOWN) && currentTime - scrollDelay > SCROLL_DELAY) {
    if (scrollOffset < totalLines - maxDisplayLines) {
      scrollOffset++;
      redrawNeeded = true;
      scrollDelay = currentTime;
    }
  }
  
  return redrawNeeded;
}

void drawScrollIndicators(bool showUp, bool showDown) {
  if (showUp) {
    tft.fillTriangle(5, 5, 10, 0, 15, 5, ST77XX_YELLOW);
  }
  
  if (showDown) {
    tft.fillTriangle(tft.width() - 15, tft.height() - 5, 
                     tft.width() - 10, tft.height() - 10, 
                     tft.width() - 5, tft.height() - 5, ST77XX_YELLOW);
  }
}

// Draw notification overlay
void drawNotification(const Notification& notif) {
  // Draw semi-transparent background
  tft.fillRect(0, 0, tft.width(), 20, ST77XX_BLACK);
  
  // Draw colored bar based on severity
  uint16_t color;
  switch(notif.severity) {
    case 1: color = ST77XX_BLUE; break;      // Info
    case 2: color = ST77XX_YELLOW; break;    // Warning
    case 3: color = ST77XX_ORANGE; break;    // Error
    case 4: color = ST77XX_RED; break;       // Critical
    default: color = ST77XX_WHITE;
  }
  
  tft.fillRect(0, 0, tft.width(), 2, color);
  
  // Draw notification text
  tft.setTextSize(1);
  tft.setTextColor(color);
  tft.setCursor(2, 5);
  
  // Truncate message if too long
  String message = notif.message;
  if (message.length() > 25) {
    message = message.substring(0, 22) + "...";
  }
  
  tft.print(message);
  
  // Draw pack info
  tft.setCursor(tft.width() - 25, 5);
  if (notif.packID > 0) {
    tft.printf("P%d", notif.packID);
  } else {
    tft.print("SYS");
  }
}

// Clear notification area
void clearNotificationDisplay() {
  tft.fillRect(0, 0, tft.width(), 20, ST77XX_BLACK);
  showNotification = false;
}

void updateDisplay() {
  static unsigned long lastAutoRefresh = 0;
  static bool forceRedraw = true;
  unsigned long now = millis();
  
  // Check for new notifications
  Notification newNotif;
  if (getLatestNotification(newNotif)) {
    if (!showNotification || 
        strcmp(newNotif.message, currentNotification.message) != 0 ||
        newNotif.timestamp > currentNotification.timestamp) {
      
      currentNotification = newNotif;
      showNotification = true;
      notificationStartTime = now;
      clearOldestNotification(); // Remove from buffer after displaying
    }
  }
  
  // Auto-hide notification after 5 seconds
  if (showNotification && (now - notificationStartTime > NOTIFICATION_DISPLAY_TIME)) {
    showNotification = false;
    refreshDisplay = true; // Force redraw of main screen
  }
  
  // Auto-refresh for data screens every second
  if (currentScreen == SCREEN_HOME || currentScreen == SCREEN_PACK_DETAIL) {
    if (now - lastAutoRefresh > DISPLAY_REFRESH_INTERVAL) {
      refreshDisplay = true;
      lastAutoRefresh = now;
    }
  }
  
  // Check for data refresh flag
  if (refreshDisplay || forceRedraw) {
    switch (currentScreen) {
      case SCREEN_HOME:
        drawHomeScreen();
        break;
      case SCREEN_PACK_DETAIL:
        drawPackDetail(activePack, packMenuSelection);
        break;
      // For other screens, they don't need to update with data changes
      // Only home and pack detail screens show real-time data
    }
    
    // Draw notification on top if active
    if (showNotification) {
      drawNotification(currentNotification);
    }
    
    forceRedraw = false;
    refreshDisplay = false;
    return;  // Skip button processing when refreshing from data
  }
  
  if (currentScreen == SCREEN_POWER_OFF) {
    static bool wakeupCompleted = false;
    
    if (digitalRead(BTN_OK) == HIGH) {
      if (okPressStart == 0) {
        okPressStart = millis();
      } else if (millis() - okPressStart >= WAKE_TIME) {
        displayOn();
        currentScreen = SCREEN_HOME;
        homeSelection = 0;
        okPressStart = 0;
        wakeupCompleted = true;
        drawHomeScreen();
      }
    } else {
      if (wakeupCompleted) {
        wakeupCompleted = false;
      }
      okPressStart = 0;
    }
    
    delay(50);
    return;
  }

  if (currentScreen == SCREEN_HOME) {
    if (digitalRead(BTN_UP) || digitalRead(BTN_DOWN)) {
      homeSelection = 1 - homeSelection;
      drawHomeScreen();
      
      // Redraw notification if active
      if (showNotification) {
        drawNotification(currentNotification);
      }
      
      delay(180);
    }

    if (digitalRead(BTN_OK)) {
      if (homeSelection == 0) {
        currentScreen = SCREEN_MAIN_MENU;
        resetScroll();
        drawMenu(mainMenu, mainMenuSize, mainMenuSelection, "MAIN MENU");
      } else {
        displayOff();
        currentScreen = SCREEN_POWER_OFF;
      }
      delay(180);
    }
  }

  else if (currentScreen == SCREEN_MAIN_MENU) {
    if (handleMenuScrolling(mainMenuSelection, mainMenuSize, MAX_LINES_DISPLAY)) {
      drawMenu(mainMenu, mainMenuSize, mainMenuSelection, "MAIN MENU");
    }
  
    if (digitalRead(BTN_OK)) {
      if (mainMenuSelection < 2) {
        activePack = mainMenuSelection + 1;
        packMenuSelection = 0;
        currentScreen = SCREEN_PACK_MENU;
        resetScroll();
        drawMenu(packMenu, packMenuSize, packMenuSelection, "PACK MENU");
      } else {
        currentScreen = SCREEN_FAULT_LOG;
        resetScroll();
        drawFaultLog();
      }
      delay(180);
    }
  
    if (digitalRead(BTN_BACK)) {
      currentScreen = SCREEN_HOME;
      resetScroll();
      drawHomeScreen();
      delay(180);
    }
  }

  else if (currentScreen == SCREEN_PACK_MENU) {
    if (handleMenuScrolling(packMenuSelection, packMenuSize, MAX_LINES_DISPLAY)) {
      drawMenu(packMenu, packMenuSize, packMenuSelection, "PACK MENU");
    }
    
    if (digitalRead(BTN_OK)) {
      currentScreen = SCREEN_PACK_DETAIL;
      drawPackDetail(activePack, packMenuSelection);
      delay(180);
    }
    
    if (digitalRead(BTN_BACK)) {
      currentScreen = SCREEN_MAIN_MENU;
      resetScroll();
      drawMenu(mainMenu, mainMenuSize, mainMenuSelection, "MAIN MENU");
      delay(180);
    }
  }

  else if (currentScreen == SCREEN_PACK_DETAIL) {
    // Screen will auto-refresh via the refreshDisplay flag
    if (digitalRead(BTN_BACK)) {
      currentScreen = SCREEN_PACK_MENU;
      drawMenu(packMenu, packMenuSize, packMenuSelection, "PACK MENU");
      delay(180);
    }
  }
  
  else if (currentScreen == SCREEN_FAULT_LOG) {
    static unsigned long clearPressStart = 0;
    const unsigned long CLEAR_TIME = 3000;
  
    int faultCount = getFaultCount();
  
    if (faultCount > 0) {
      if (handleInfoScrolling(faultCount, MAX_LINES_DISPLAY)) {
        drawFaultLog();
      }
    
      if (digitalRead(BTN_OK)) {
        if (clearPressStart == 0) {
          clearPressStart = millis();
        } else {
          unsigned long holdTime = millis() - clearPressStart;
          if (holdTime < CLEAR_TIME) {
            int progressWidth = map(holdTime, 0, CLEAR_TIME, 0, tft.width() - 10);
            tft.fillRect(5, tft.height() - 18, progressWidth, 3, ST77XX_YELLOW);
          } else {
            clearFaults();
            resetScroll();
            drawFaultLog();
            clearPressStart = 0;
          }
        }
      } else {
        clearPressStart = 0;
      }
    } else {
      if (digitalRead(BTN_OK)) {
        tft.setTextColor(ST77XX_YELLOW);
        tft.setCursor(5, tft.height() - 15);
        tft.print("No faults to clear");
        delay(500);
        drawFaultLog();
      }
    }

    if (digitalRead(BTN_BACK)) {
      currentScreen = SCREEN_MAIN_MENU;
      resetScroll();
      drawMenu(mainMenu, mainMenuSize, mainMenuSelection, "MAIN MENU");
      delay(180);
    }
  }
}

void drawBattery(int x, int y, int w, int h, float soc, bool connected) {
  int terminalW = 6;
  int socFill = map((int)soc, 0, 100, 0, w - 4);

  tft.drawRect(x, y, w, h, ST77XX_WHITE);
  tft.fillRect(x + w, y + h / 4, terminalW, h / 2, ST77XX_WHITE);

  uint16_t fillColor;
  if (!connected) {
    fillColor = ST77XX_ORANGE;
    socFill = 0;
  } else if (soc <= 20) {
    fillColor = ST77XX_RED;
  } else if (soc <= 50) {
    fillColor = ST77XX_YELLOW;
  } else {
    fillColor = ST77XX_GREEN;
  }

  if (connected) {
    tft.fillRect(x + 2, y + 2, socFill, h - 4, fillColor);
  }

  tft.setTextSize(1);
  tft.setCursor(x + w / 2 - 10, y + h / 2 - 4);
  
  if (!connected) {
    tft.setTextColor(ST77XX_WHITE);
    tft.print("NC");
  } else {
    tft.setTextColor(ST77XX_WHITE);
    tft.print((int)soc);
    tft.print("%");
  }
}

void drawHomeScreen() {
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(15, 5);
  tft.print("Battery SoC");

  drawBattery(20, 40, 50, 30, battery1SoC, slave1Connected);
  drawBattery(90, 40, 50, 30, battery2SoC, slave2Connected);

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(25, 75);
  tft.print("Pack 1");
  tft.setCursor(95, 75);
  tft.print("Pack 2");

  tft.setCursor(20, 85);
  tft.setTextColor(slave1Connected ? ST77XX_GREEN : ST77XX_RED);
  tft.print(slave1Connected ? "CONN" : "NC");
  
  tft.setCursor(100, 85);
  tft.setTextColor(slave2Connected ? ST77XX_GREEN : ST77XX_RED);
  tft.print(slave2Connected ? "CONN" : "NC");

  if (homeSelection == 0) {
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(5, 110);
    tft.print(">Menu");
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(90, 110);
    tft.print("PowerOff");
  } else {
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(5, 110);
    tft.print("Menu");
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(85, 110);
    tft.print(">PowerOff");
  }
}

void drawMenu(const char* items[], int size, int selection, const char* title = "MENU") {
  tft.fillScreen(ST77XX_BLACK);
  
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(5, 5);
  tft.print(title);
  
  bool showUp = scrollOffset > 0;
  bool showDown = scrollOffset < size - MAX_LINES_DISPLAY && size > MAX_LINES_DISPLAY;
  drawScrollIndicators(showUp, showDown);
  
  tft.drawFastHLine(0, 15, tft.width(), ST77XX_WHITE);
  
  for (int i = 0; i < MAX_LINES_DISPLAY; i++) {
    int itemIndex = i + scrollOffset;
    if (itemIndex >= size) break;
    
    int yPos = 20 + i * 15;
    
    if (itemIndex == selection) {
      tft.fillRect(5, yPos - 2, tft.width() - 10, 12, ST77XX_YELLOW);
      tft.setTextColor(ST77XX_BLACK);
      tft.setCursor(10, yPos);
      tft.print("> ");
      tft.print(items[itemIndex]);
    } else {
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(10, yPos);
      tft.print("  ");
      tft.print(items[itemIndex]);
    }
  }
  
  tft.drawFastHLine(0, tft.height() - 20, tft.width(), ST77XX_WHITE);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(5, tft.height() - 15);
  
  if (size <= MAX_LINES_DISPLAY) {
    tft.print("Item ");
    tft.print(selection + 1);
    tft.print(" of ");
    tft.print(size);
  } else {
    tft.print("Item ");
    tft.print(selection + 1);
    tft.print("/");
    tft.print(size);
    tft.print("  (");
    tft.print(scrollOffset + 1);
    tft.print("-");
    tft.print(min(scrollOffset + MAX_LINES_DISPLAY, size));
    tft.print(")");
  }
}

void drawPackDetail(int pack, int item) {
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(10, 5);
  tft.print("Pack ");
  tft.print(pack);

  tft.setTextSize(1);
  tft.setCursor(10, 30);
  tft.print(packMenu[item]);

  tft.setCursor(10, 50);

  PackData* realData = (pack == 1) ? &pack1Data : &pack2Data;
  bool connected = (pack == 1) ? slave1Connected : slave2Connected;
  
  if (!connected) {
    tft.setTextColor(ST77XX_RED);
    tft.print("NO CONNECTION");
    tft.setCursor(10, 70);
    tft.print("Check slave device");
    return;
  }
  
  if (!realData->dataValid) {
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("WAITING DATA...");
    return;
  }
  
  tft.setTextColor(ST77XX_WHITE);
  
  switch (item) {
    case 0: 
      tft.print(realData->voltage);
      tft.print(" V"); 
      break;
    case 1:
      tft.print("Charge: ");
      tft.print(realData->chargeCurrent);  // Already in mA
      tft.print(" mA");
      tft.setCursor(10, 70);
      tft.print("Discharge: ");
      tft.print(realData->dischargeCurrent);  // Already in mA
      tft.print(" mA");
      break;
    case 2: 
      tft.print(realData->temperature);
      tft.print(" °C"); 
      break;
    case 3: 
      if (realData->voltage > 0) {
        tft.print("NORMAL"); 
      } else {
        tft.print("NO DATA"); 
      }
      break;
    case 4: 
      tft.print(realData->soh);
      tft.print(" %"); 
      break;
  }
  
  // Show any active protection flags (using mA thresholds)
  if (realData->voltage > 0) {
    int yPos = 90;
    if (realData->voltage > 18.1f) {
      tft.setCursor(10, yPos);
      tft.setTextColor(ST77XX_RED);
      tft.print("OVER VOLTAGE!");
      yPos += 15;
    } else if (realData->voltage < 13.9f) {
      tft.setCursor(10, yPos);
      tft.setTextColor(ST77XX_RED);
      tft.print("UNDER VOLTAGE!");
      yPos += 15;
    }
    if (realData->temperature > 40.0f) {
      tft.setCursor(10, yPos);
      tft.setTextColor(ST77XX_ORANGE);
      tft.print("OVER TEMPERATURE!");
      yPos += 15;
    }
    if (realData->chargeCurrent > 1000.0f) {  // 1000mA threshold
      tft.setCursor(10, yPos);
      tft.setTextColor(ST77XX_ORANGE);
      tft.print("OVER CHARGE CURRENT!");
      yPos += 15;
    }
    if (realData->dischargeCurrent > 2000.0f) {  // 2000mA threshold
      tft.setCursor(10, yPos);
      tft.setTextColor(ST77XX_ORANGE);
      tft.print("OVER DISCHARGE CURRENT!");
    }
  }
  
  tft.setCursor(tft.width() - 60, 5);
  if (realData->dataValid) {
    unsigned long age = (millis() - realData->timestamp) / 1000;
    if (age < 10) {
      tft.setTextColor(ST77XX_GREEN);
      tft.print("LIVE");
    } else if (age < 30) {
      tft.setTextColor(ST77XX_YELLOW);
      tft.print("OLD");
    } else {
      tft.setTextColor(ST77XX_RED);
      tft.print("STALE");
    }
  } else {
    tft.setTextColor(ST77XX_RED);
    tft.print("NO DATA");
  }
}

void drawFaultLog() {
  tft.fillScreen(ST77XX_BLACK);
  
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_RED);
  tft.setCursor(5, 5);
  tft.print("FAULT LOG");
  
  int faultCount = getFaultCount();
  
  if (faultCount == 0) {
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(30, 60);
    tft.print("NO FAULTS");
    tft.setCursor(20, 80);
    tft.print("SYSTEM OK");
    
    tft.drawFastHLine(0, tft.height() - 20, tft.width(), ST77XX_WHITE);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(5, tft.height() - 15);
    tft.print("Return");
    tft.setCursor(tft.width() - 85, tft.height() - 15);
    tft.print("Hold OK=Clear");
    return;
  }
  
  int maxFaultsToShow = MAX_LINES_DISPLAY;
  
  bool showUp = scrollOffset > 0;
  bool showDown = scrollOffset < faultCount - maxFaultsToShow;
  drawScrollIndicators(showUp, showDown);
  
  tft.drawFastHLine(0, 15, tft.width(), ST77XX_WHITE);
  
  int startY = 20;
  int lineHeight = 15;
  
  for (int i = 0; i < maxFaultsToShow; i++) {
    int faultIndex = i + scrollOffset;
    if (faultIndex >= faultCount) break;
    
    FaultEntry fault = getFault(faultIndex);
    int yPos = startY + (i * lineHeight);
    
    uint16_t severityColor = getSeverityColor(fault.severity);
    tft.fillRect(5, yPos, 8, 8, severityColor);
    
    tft.setTextColor(ST77XX_BLACK);
    tft.setCursor(7, yPos + 1);
    char sevChar = (fault.severity == 1) ? 'I' : 
                   (fault.severity == 2) ? 'W' : 
                   (fault.severity == 3) ? 'E' : 'C';
    tft.print(sevChar);
    
    String faultMessage = createFaultMessage(fault);
    
    int16_t textX, textY;
    uint16_t textW, textH;
    tft.getTextBounds(faultMessage.c_str(), 0, 0, &textX, &textY, &textW, &textH);
    
    int availableWidth = tft.width() - 20;
    
    if (textW <= availableWidth) {
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(15, yPos + 1);
      tft.print(faultMessage);
    } else {
      String firstLine = "";
      String secondLine = "";
      
      String words[20];
      int wordCount = 0;
      String currentWord = "";
      
      for (unsigned int j = 0; j < faultMessage.length(); j++) {
        char c = faultMessage.charAt(j);
        if (c == ' ' || c == '\0') {
          if (currentWord.length() > 0) {
            words[wordCount++] = currentWord;
            currentWord = "";
          }
        } else {
          currentWord += c;
        }
      }
      if (currentWord.length() > 0) {
        words[wordCount++] = currentWord;
      }
      
      firstLine = "";
      for (int j = 0; j < wordCount; j++) {
        String testLine = firstLine;
        if (testLine.length() > 0) testLine += " ";
        testLine += words[j];
        
        tft.getTextBounds(testLine.c_str(), 0, 0, &textX, &textY, &textW, &textH);
        
        if (textW > availableWidth) {
          secondLine = words[j];
          for (int k = j + 1; k < wordCount; k++) {
            secondLine += " " + words[k];
          }
          break;
        } else {
          firstLine = testLine;
        }
      }
      
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(15, yPos + 1);
      tft.print(firstLine);
      
      if (i + 1 < maxFaultsToShow) {
        i++;
        yPos = startY + (i * lineHeight);
        
        String displaySecondLine = secondLine;
        tft.getTextBounds(displaySecondLine.c_str(), 0, 0, &textX, &textY, &textW, &textH);
        
        if (textW > availableWidth) {
          displaySecondLine = truncateText(displaySecondLine, availableWidth);
        }
        
        tft.setTextColor(ST77XX_CYAN);
        tft.setCursor(15, yPos + 1);
        tft.print(displaySecondLine);
      }
    }
  }
  
  tft.drawFastHLine(0, tft.height() - 20, tft.width(), ST77XX_WHITE);
  
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(5, tft.height() - 15);
  tft.print(faultCount);
  tft.print(" faults");
  
  tft.setCursor(tft.width() - 85, tft.height() - 15);
  tft.print("Hold OK=Clear");
}

String createFaultMessage(FaultEntry fault) {
  String message = String(fault.description);
  
  if (fault.pack != 0) {
    message += " in ";
    message += getPackName(fault.pack);
  }
  
  unsigned long age = (millis() - fault.timestamp) / 1000;
  message += " for ";
  
  if (age < 60) {
    message += "<1m";
  } else if (age < 3600) {
    message += String(age / 60);
    message += "m";
  } else {
    message += String(age / 3600);
    message += "h";
  }
  
  return message;
}

String truncateText(String text, int maxWidth) {
  tft.setTextSize(1);
  
  int maxChars = 0;
  String testStr = "";
  
  for (int i = 0; i < text.length(); i++) {
    testStr = text.substring(0, i + 1) + "...";
    
    int16_t textX, textY;
    uint16_t textW, textH;
    tft.getTextBounds(testStr.c_str(), 0, 0, &textX, &textY, &textW, &textH);
    
    if (textW > maxWidth) {
      break;
    }
    maxChars = i + 1;
  }
  
  if (maxChars > 3) {
    return text.substring(0, maxChars - 3) + "...";
  } else {
    return "...";
  }
}