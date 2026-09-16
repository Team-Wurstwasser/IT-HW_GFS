#include <Arduino.h>
#include <SCServo.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>
#include "config.h"
#include "hardware/gripper.h"
#include "hardware/calibration.h"
#include "math/kinematics.h"
#include "comm/esp_now_handler.h"

void ctrlMove();
void moveToCoordinate(double x, double y, double z, double theta);
uint8_t pickUpAndReadColor();
void dropBlock(int id);
void initArmPosition();

SMS_STS st; // Instanz für die Steuerung der SMS/STS Feetech-Servos

float InfoBuffer[9]; 

// Servo-Variablen für Bewegungsprofile
int16_t STPos[4];                  // Zielpositionen der Servos
int STDirection[4] = {1, 1, 1, -1}; // Drehrichtungskorrektur für die einzelnen Gelenke
uint16_t STSpd[4];                 // Geschwindigkeiten für die Servos
uint8_t STAc[4] = {50, 50, 50, 50}; // Beschleunigungsrampen für sanfte Bewegungen

// Servo-IDs im Bussystem
uint8_t STID[4] = {1, 2, 3, 4};    // Gelenk-Servos 1 bis 4

void setup() {
  Serial.begin(115200);
  
  // Hardware-Serial 1 für das Feetech-Servo-Bussystem konfigurieren
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;

  // Roboterkalibrierung und Startpositionierung ausführen
  s2AutoConfig();
  initArmPosition();
}

void loop() {
  // Endlosschleife: Farbe scannen, aufheben, farbabhängig sortieren
  int colorID = pickUpAndReadColor();
  dropBlock(colorID);
}

void ctrlMove() {
  STPos[0] = (int)(angleGenOut(STDirection[0] * InfoBuffer[angle_1]) + 0.5);
  STPos[1] = (int)(angleGenOut(STDirection[1] * InfoBuffer[angle_2] + 15) * 3 + 0.5); // *3 steht für eine mechanische Übersetzung
  STPos[2] = (int)(angleGenOut(STDirection[2] * InfoBuffer[angle_3] + 180) + 0.5);
  STPos[3] = (int)(angleGenOut(STDirection[3] * InfoBuffer[angle_4] + 180) + 0.5);
  
  // Synchroner Schreibbefehl an alle 4 Hauptgelenke gleichzeitig für flüssige Bahnen
  st.SyncWritePosEx(STID, STID_count, STPos, STSpd, STAc);
}

void moveToCoordinate(double x, double y, double z, double theta){
  bool nanCtrlIK = ctrlIK(x, y, z, theta);
  
  if (!nanCtrlIK) {
    ctrlMove(); // Bewegung starten
    
    // Blockierende Überwachung, ob die Servos die Position erreicht haben
    bool allAtTarget = false;
    while (!allAtTarget) {
      bool signalLost = false;
      allAtTarget = true;
      
      for (uint8_t i = 0; i < STID_count; i++) {
        int16_t currentPos = st.ReadPos(STID[i]);
        if (currentPos == -1) {
          signalLost = true;
          allAtTarget = false;
          break; 
        }
        if (abs(currentPos - STPos[i]) > positionTolerance) {
          allAtTarget = false; // Mindestens ein Servo ist noch unterwegs
        }
      }
      
      if (signalLost) {
        Serial.println("WARNUNG: Signal verloren oder Servo offline! Befehl wird neu gesendet...");
        ctrlMove();
        delay(40);
      } else if (!allAtTarget) {
        delay(50);
      }
    }
  } else {
    Serial.printf("FEHLER: Zielkoordinaten nicht erreichbar! X:%.2f, Y:%.2f, Z:%.2f\n", x, y, z);
  }
}

uint8_t pickUpAndReadColor() {
  moveToCoordinate(PICKUP_HOVER.x, PICKUP_HOVER.y, PICKUP_HOVER.z, PICKUP_HOVER.theta);
  moveToCoordinate(PICKUP_SCAN.x, PICKUP_SCAN.y, PICKUP_SCAN.z, PICKUP_SCAN.theta);

  // 4 Sekunden warten, damit der Sender genug stabile Farbsamples via ESP-NOW schicken kann
  delay(4000);
  uint8_t detectedColorID = getFilteredColor();
  Serial.print("Erkannte Farbe: ");
  Serial.println(tableColor[detectedColorID]);

  // Stein greifen und anheben
  moveToCoordinate(PICKUP_GRAB.x, PICKUP_GRAB.y, PICKUP_GRAB.z, PICKUP_GRAB.theta);
  closeGripper();
  moveToCoordinate(PICKUP_LIFT.x, PICKUP_LIFT.y, PICKUP_LIFT.z, PICKUP_LIFT.theta);

  return detectedColorID;
}

void dropBlock(int id) {
  if (id < 0 || id > 7) id = 0;
  moveToCoordinate(dropLocations[id].x, dropLocations[id].y, dropLocations[id].z, dropLocations[id].theta);
  openGripper();
}

void initArmPosition() {
  float startX = LEN_B + LEN_D + LEN_E;
  float startY = LEN_H;
  float startZ = LEN_A + LEN_C - LEN_F;
  float startTheta = 180.0;

  // Erste Fahrt extra langsam zur Sicherheit
  STSpd[0] = 1000; STSpd[1] = 1000; STSpd[2] = 1000; STSpd[3] = 1000;
  moveToCoordinate(startX, startY, startZ, startTheta);

  // Arbeitsgeschwindigkeit für den Loop erhöhen
  STSpd[0] = 3000; STSpd[1] = 3000; STSpd[2] = 3400; STSpd[3] = 3400;
  openGripper(1500); // Greifer für den Start öffnen
}