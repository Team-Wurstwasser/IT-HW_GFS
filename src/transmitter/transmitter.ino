#include <esp_now.h>
#include <WiFi.h>

// MAC-Adresse Empfänger
uint8_t receiverAddress[] = {0x28, 0x05, 0xa5, 0x4f, 0xcf, 0xe0}; 

typedef struct struct_message {
  uint8_t colorID;
} struct_message;

struct_message daten;
esp_now_peer_info_t peerInfo;

const uint8_t EV3_RX_PIN = 26;
const uint8_t EV3_TX_PIN = 27;
const uint8_t ACK_BYTE = 0x04;
const uint8_t NACK_BYTE = 0x02;
const uint8_t COLOR_MODE_BYTE = 2;

int serialValue[4] = {0, 0, 0, 0};
uint8_t waitTime = 5;
unsigned long startTime = 0; 

void sendMode() {
  Serial2.write(0x44);
  Serial2.write(0x11);
  Serial2.write(0xff ^ 0x44 ^ 0x11);

  for (int n = 0; n < 3; n++) {
    Serial2.write(0x43);
    Serial2.write(COLOR_MODE_BYTE & 0x7);
    Serial2.write(0xff ^ 0x43 ^ (COLOR_MODE_BYTE & 0x7));

    Serial2.write(NACK_BYTE);
  }
}

void sensorBegin() {
  Serial2.begin(2400, SERIAL_8N1, EV3_RX_PIN, EV3_TX_PIN);
  
  while (true) {
    int value = Serial2.read();
    if (value != -1) {
      serialValue[1] = serialValue[0];
      serialValue[0] = value;
    }

    if (serialValue[0] == 128 && serialValue[1] == 144) {
      while (Serial2.read() != 4);
      
      Serial2.write(ACK_BYTE);
      delay(60);
      
      Serial2.end();
      Serial2.begin(57600, SERIAL_8N1, EV3_RX_PIN, EV3_TX_PIN);
      break;
    }
  }
  sendMode();
}

int8_t sensorRead() {
  sendMode();
  startTime = millis();
  while (millis() - startTime <= waitTime);
  int8_t resp = 0;
  while (Serial2.available()) {
    serialValue[2] = Serial2.read();

    if (serialValue[2] != -1 && serialValue[3] == 0) {
      if (serialValue[2] <= 7) { 
        serialValue[3] = 1;
        resp = serialValue[2];
      }
    }
    else {
      serialValue[3] = 0;
    }
  }
  
  if (resp < 0 || resp > 7) {
    resp = 0;
  }
  return resp;
}

void setup() {
  sensorBegin();
  
  WiFi.mode(WIFI_STA);

  esp_now_init();
  
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  esp_now_add_peer(&peerInfo);
}

void loop() {
  uint8_t color = sensorRead();
  daten.colorID = color;
  esp_now_send(receiverAddress, (uint8_t *) &daten, sizeof(daten));
  delay(200);
}