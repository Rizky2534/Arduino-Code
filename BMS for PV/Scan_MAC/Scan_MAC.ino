#include <WiFi.h>

void setup() {
    Serial.begin(115200);
    delay(1000); // Allow some time for the ESP32 to initialize

    WiFi.mode(WIFI_STA); // Set WiFi to station mode
    delay(1000); // Wait a bit before fetching the MAC address

    Serial.print("ESP32 MAC Address: ");
    Serial.println(WiFi.macAddress());
}

void loop() {
    // No need to do anything in the loop
}
