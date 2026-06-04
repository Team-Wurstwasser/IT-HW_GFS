#include <esp_now.h>
#include <WiFi.h>
#include "Ev3ColorSensor.h"

Ev3ColorSensor sensor(8, 7);

// MAC-Adresse Empfänger
uint8_t receiverAddress[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}; 

typedef struct struct_message {
  uint8_t colorID;
} struct_message;

struct_message daten;
esp_now_peer_info_t peerInfo;

void setup() {
  Serial.begin(115200);
  sensor.begin();
  
  WiFi.mode(WIFI_STA);

  esp_now_init();
  
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  esp_now_add_peer(&peerInfo);
}

void loop() {
  uint8_t color = sensor.read();  
  daten.colorID = color;
  esp_now_send(receiverAddress, (uint8_t *) &daten, sizeof(daten));
  delay(200);
}