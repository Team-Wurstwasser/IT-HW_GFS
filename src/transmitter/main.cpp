#include <Arduino.h>
#include "config.h"
#include "hardware/ev3_sensor.h"
#include "comm/esp_now_handler.h"

void setup() {
  sensorBegin(); // Sensor-Verbindung und Handshake starten
  
  initESPNow(); // ESP-NOW initialisieren und konfigurieren
}

void loop() {
  // 1. Aktuelle Farbe vom EV3-Sensor einlesen
  uint8_t color = sensorRead();
  
  // 2. Daten via ESP-NOW an den Empfänger senden
  sendColorData(color);
  
  delay(200);
}