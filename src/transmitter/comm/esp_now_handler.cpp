#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "../config.h"
#include "esp_now_handler.h"

// Struktur für die ESP-NOW Datenübertragung
typedef struct struct_message {
  uint8_t colorID; // ID der erkannten Farbe
} struct_message;

esp_now_peer_info_t peerInfo; // Informationen über den Empfänger-Peer
static struct_message data;

void initESPNow() {
  // 1. WiFi starten
  WiFi.mode(WIFI_STA);

  // 2. ESP-NOW initialisieren
  esp_now_init();
  
  // 3. Empfänger-Informationen konfigurieren
  memcpy(peerInfo.peer_addr, receiverAddress, 6); // MAC-Adresse kopieren
  peerInfo.channel = 0;                           // WLAN-Kanal
  peerInfo.encrypt = false;                       // Übertragung unverschlüsselt
  
  // 4. Empfänger-Gerät hinzufügen
  esp_now_add_peer(&peerInfo);
}

void sendColorData(uint8_t colorID) {
  data.colorID = colorID;
  esp_now_send(receiverAddress, (uint8_t *) &data, sizeof(data));
}