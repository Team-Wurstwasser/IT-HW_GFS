#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// MAC-Adresse des Empfänger-ESP32
const uint8_t receiverAddress[] = {0x28, 0x05, 0xa5, 0x4f, 0xcf, 0xe0}; 

// Pin-Definitionen für die serielle Verbindung zum EV3 Sensor
const uint8_t EV3_RX_PIN = 26;
const uint8_t EV3_TX_PIN = 27;

// Definitionen für das EV3-UART-Protokoll
const uint8_t ACK_BYTE = 0x04;
const uint8_t NACK_BYTE = 0x02;
const uint8_t COLOR_MODE_BYTE = 2;

#endif