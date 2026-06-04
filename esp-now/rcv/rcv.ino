#include <esp_now.h>
#include <WiFi.h>

String tableColor[8] = {
  "No color",
  "Black",
  "Blue",
  "Green",
  "Yellow",
  "Red",
  "White",
  "Brown"
};

typedef struct struct_message {
  uint8_t colorID;
} struct_message;

struct_message eingehendeDaten;

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&eingehendeDaten, incomingData, sizeof(eingehendeDaten));
  
  Serial.print("Empfangene Farbe: ");
  Serial.println(tableColor[eingehendeDaten.colorID]);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
}