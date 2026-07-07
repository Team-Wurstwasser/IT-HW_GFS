#include <esp_now.h>
#include <WiFi.h>

// MAC-Adresse des Empfänger-ESP32
uint8_t receiverAddress[] = {0x28, 0x05, 0xa5, 0x4f, 0xcf, 0xe0}; 

// Struktur für die ESP-NOW Datenübertragung
typedef struct struct_message {
  uint8_t colorID; // ID der erkannten Farbe
} struct_message;

struct_message data;          // Instanz der Nachrichten-Struktur
esp_now_peer_info_t peerInfo; // Informationen über den Empfänger-Peer

// Pin-Definitionen für die serielle Verbindung zum EV3
const uint8_t EV3_RX_PIN = 26;
const uint8_t EV3_TX_PIN = 27;

// Definitionen für das EV3-UART-Protokoll
const uint8_t ACK_BYTE = 0x04;
const uint8_t NACK_BYTE = 0x02;
const uint8_t COLOR_MODE_BYTE = 2;

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

void setup() {
  // 1. Sensor-Verbindung und Handshake starten
  sensorBegin();
  
  // 2. WiFi starten
  WiFi.mode(WIFI_STA);

  // 3. ESP-NOW initialisieren
  if (esp_now_init() != ESP_OK) {
    return;
  }
  
  // 4. Empfänger-Informationen konfigurieren
  memcpy(peerInfo.peer_addr, receiverAddress, 6); // MAC-Adresse kopieren
  peerInfo.channel = 0;                           // WLAN-Kanal
  peerInfo.encrypt = false;                       // Übertragung unverschlüsselt
  
  // 5. Empfänger-Gerät hinzufügen
  esp_now_add_peer(&peerInfo);
}

void loop() {
  // 1. Aktuelle Farbe vom EV3-Sensor einlesen
  uint8_t color = sensorRead();
  
  // 2. Wert in die Datenstruktur eintragen
  data.colorID = color;
  
  // 3. Daten via ESP-NOW an den Empfänger senden
  esp_now_send(receiverAddress, (uint8_t *) &data, sizeof(data));
  
  delay(200);
}