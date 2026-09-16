#include <Arduino.h>
#include "../config.h"
#include "ev3_sensor.h"

// Array zum Zwischenspeichern von gelesenen Serial-Bytes
int serialValue[4] = {0, 0, 0, 0};
uint8_t waitTime = 5;          // Wartezeit in Millisekunden für die serielle Antwort
unsigned long startTime = 0;   // Zeitstempel für Delays

void sendMode() {
  // 1. Modus-Typ ankündigen (0x44)
  Serial2.write(0x44);
  Serial2.write(0x11);
  Serial2.write(0xff ^ 0x44 ^ 0x11); // Checksumme

  // 2. Modus mehrmals senden und mit NACK bestätigen, um die Übernahme zu erzwingen
  for (int n = 0; n < 3; n++) {
    Serial2.write(0x43); // Information über Modus
    Serial2.write(COLOR_MODE_BYTE & 0x7);
    Serial2.write(0xff ^ 0x43 ^ (COLOR_MODE_BYTE & 0x7)); // Checksumme

    Serial2.write(NACK_BYTE);
  }
}

void sensorBegin() {
  // Start des Handshakes mit der Standard-EV3-Start-Baudrate (2400)
  Serial2.begin(2400, SERIAL_8N1, EV3_RX_PIN, EV3_TX_PIN);
  
  while (true) {
    int value = Serial2.read();
    if (value != -1) {
      // Schiebe die Werte im Array weiter, um die letzten zwei Bytes zu prüfen
      serialValue[1] = serialValue[0];
      serialValue[0] = value;
    }

    // Prüfen auf die EV3-Sync-Bytes
    if (serialValue[0] == 128 && serialValue[1] == 144) {
      // Warte auf das Ende der Sensor-Meldung 
      while (Serial2.read() != 4);
      
      // Sende Bestätigung an den Sensor
      Serial2.write(ACK_BYTE);
      delay(60); // Kurze Pause, damit der Sensor umschalten kann
      
      // Verbindung trennen und mit hoher Geschwindigkeit neu starten
      Serial2.end();
      Serial2.begin(57600, SERIAL_8N1, EV3_RX_PIN, EV3_TX_PIN);
      break;
    }
  }
  // Modus nach erfolgreichem Handshake setzen
  sendMode();
}

int8_t sensorRead() {
  sendMode(); // Sensor triggern
  
  // Non-blocking Delay: Kurze Zeit warten, bis Daten im Buffer ankommen
  startTime = millis();
  while (millis() - startTime <= waitTime);
  
  int8_t resp = 0;
  
  // Verfügbare Daten auslesen und filtern
  while (Serial2.available()) {
    serialValue[2] = Serial2.read();

    // Gültigen Wert abfangen
    if (serialValue[2] != -1 && serialValue[3] == 0) {
      if (serialValue[2] <= 7) { 
        serialValue[3] = 1;      // Flag setzen, dass ein gültiger Wert gefunden wurde
        resp = serialValue[2];   // Farb-ID speichern
      }
    }
    else {
      serialValue[3] = 0; // Flag zurücksetzen
    }
  }
  
  // Sicherheitsprüfung: Ungültige Werte auf 0 zurücksetzen
  if (resp < 0 || resp > 7) {
    resp = 0;
  }
  return resp;
}