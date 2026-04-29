#include "display.h"
#include <SPI.h>

#define TFT_CS   15
#define TFT_RST  4
#define TFT_DC   2
#define TFT_LED  27

static bool displayOffFlag = false;

void displayInit() {
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);
  
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
}

void displayOff() {
  tft.fillScreen(ST77XX_BLACK);
  tft.writeCommand(ST77XX_SLPIN);
  
  delay(50);
  digitalWrite(TFT_LED, LOW);
  
  displayOffFlag = true;
}

void displayOn() {
  digitalWrite(TFT_LED, HIGH);
  delay(50);
  
  tft.writeCommand(ST77XX_SLPOUT);
  delay(150);
  
  tft.fillScreen(ST77XX_BLACK);
  
  displayOffFlag = false;
}

bool isDisplayOff() {
  return displayOffFlag;
}