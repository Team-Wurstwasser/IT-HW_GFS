#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "esp_now_handler.h"

// Ringpuffer zur Stabilisierung der Farberkennung
volatile uint8_t colorBuffer[20] = {0}; // Speichert die letzten 20 empfangenen Farbwerte
volatile uint8_t bufferIndex = 0;       // Aktuelle Schreibposition im Ringpuffer
volatile unsigned long lastSampleTime = 0; // Zeitstempel zur Begrenzung der Abtastrate

// ESP-NOW Datenstruktur
typedef struct struct_message {
  uint8_t colorID;
} struct_message;

struct_message receivedData;

// Empfangs-Callback (wird von ESP-NOW aufgerufen)
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  
  if (receivedData.colorID < 8) {
    unsigned long currentTime = millis();
    // Entprellung / Rauschfilter: Nur alle 100ms einen Wert akzeptieren
    if (currentTime - lastSampleTime >= 100) {
      colorBuffer[bufferIndex] = receivedData.colorID;
      bufferIndex = (bufferIndex + 1) % 20; // Puffer weiterbewegen
      lastSampleTime = currentTime;
    }
  }
}

void initESPNow() {
  WiFi.mode(WIFI_STA); // WiFi-Modus für ESP-NOW initialisieren
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
}

uint8_t getFilteredColor() {
  uint32_t colorCounter[8] = {0};
  
  // Häufigkeit jeder Farb-ID im Puffer zählen
  for (uint8_t i = 0; i < 20; i++) {
    colorCounter[colorBuffer[i]]++;
  }
  
  // Die Farbe mit den meisten Treffern ermitteln
  uint8_t mostFrequentColorID = 0;
  uint32_t maxCount = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (colorCounter[i] > maxCount) {
      maxCount = colorCounter[i];
      mostFrequentColorID = i;
    }
  }
  
  return mostFrequentColorID;
}