#include <Arduino.h>
#include <SCServo.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
uint8_t getFilteredColor();
int16_t angleGenOut(float angleInput);
void setMode(uint8_t id, uint8_t mode);
void setMiddle(uint8_t id);
void servoStop(uint8_t id);
int16_t readServoLoad(uint8_t id);
void s2AutoConfig();
bool calculateXYPlane(double x, double y);
bool calculateGripper(double theta);
bool calculateJointChain(double chainTargetX, double chainTargetY);
bool ctrlIK(double x, double y, double z, double theta);
void ctrlMove();
void moveToCoordinate(double x, double y, double z, double theta);
void controlGripperRaw(int16_t targetPos, uint16_t speed);
void openGripper(uint16_t speed = 2000);
void closeGripper(uint16_t speed = 2000);
uint8_t pickUpAndReadColor();
void dropBlock(int id);
void initArmPosition();

// Pin-Belegung für die serielle Kommunikation mit den Servos
const uint8_t S_RXD = 18;
const uint8_t S_TXD = 19;

// Ringpuffer zur Stabilisierung der Farberkennung
volatile uint8_t colorBuffer[20] = {0}; // Speichert die letzten 20 empfangenen Farbwerten
volatile uint8_t bufferIndex = 0;       // Aktuelle Schreibposition im Ringpuffer
volatile unsigned long lastSampleTime = 0; // Zeitstempel zur Begrenzung der Abtastrate

SMS_STS st; // Instanz für die Steuerung der SMS/STS Feetech-Servos

// Geometrische Längenmaße des Roboterarms
const float LEN_A = 115.432, LEN_B = 41.0513, LEN_C = 168.8579, LEN_D = 127.9234, LEN_E = 108.5357, LEN_F = 7.7076, LEN_G = 90.0, LEN_H = -13.75;

// Indizes für das InfoBuffer-Array zur Speicherung berechneter Gelenkwinkel und Längen
const uint8_t angle_1 = 0, angle_2 = 1, angle_3 = 2, angle_4 = 3, angle_IKE = 4, angle_EoAT  = 5, len_a = 6, len_b = 7, len_totalXY = 8;
float InfoBuffer[9]; 

// Servo-Variablen für Bewegungsprofile
int16_t STPos[4];                  // Zielpositionen der Servos
int STDirection[4] = {1, 1, 1, -1}; // Drehrichtungskorrektur für die einzelnen Gelenke
uint16_t STSpd[4];                 // Geschwindigkeiten für die Servos
uint8_t STAc[4] = {50, 50, 50, 50}; // Beschleunigungsrampen für sanfte Bewegungen

// Servo-IDs im Bussystem
uint8_t STID[4] = {1, 2, 3, 4};    // Gelenk-Servos 1 bis 4
const uint8_t STID_count = 4;      // Anzahl der Hauptgelenke
const uint8_t gripperID = 5;       // Servo-ID für den Greifer

// Konfigurationswerte für die mechanische Blockade-Erkennung
const int torqueThreshold = 300;   // Last-Schwellenwert für die Erkennung des Anschlags
const int reverseRange = 4900;     // Distanz für das Freifahren nach Anschlag
const int positionTolerance = 25;  // Toleranz in Schritten, wann ein Servo sein Ziel erreicht hat

// Zuordnung der Farb-IDs zu Klartext-Namen
String tableColor[8] = {
  "No color", "Black", "Blue", "Green", "Yellow", "Red", "White", "Brown"
};

// Struktur für kartesische Koordinaten und den Handgelenk-Winkel 
struct Coordinate { float x, y, z, theta; };

// Fest definierte Bahnpunkte für den Arbeitsablauf
const Coordinate PICKUP_HOVER = {0.0, 330.0, 200.0, 130.0}; // Sicherheits-Überflugsposition
const Coordinate PICKUP_SCAN  = {0.0, 330.0,  85.0, 130.0}; // Position zum Scannen der Farbe
const Coordinate PICKUP_GRAB  = {0.0, 351.0,  70.0, 250.0}; // Absenken und Stein greifen
const Coordinate PICKUP_LIFT  = {0.0, 351.0, 200.0, 250.0}; // Stein nach oben wegholen

// Ziel-Abladeplätze sortiert nach Farb-ID (0 bis 7)
Coordinate dropLocations[8] = {
  { 250.0, -70.0, 95.0, 250.0 },  // 0: No color (Default-Platz)
  { 250.0, -70.0, 95.0, 250.0 },  // 1: Black
  { -70.0, -315.0, 95.0, 250.0 }, // 2: Blue
  {  25.0, -315.0, 95.0, 250.0 }, // 3: Green
  { -70.0, -250.0, 95.0, 250.0 }, // 4: Yellow
  {  25.0, -250.0, 95.0, 250.0 }, // 5: Red
  { 250.0,   25.0, 95.0, 250.0 }, // 6: White
  { 250.0, -70.0, 95.0, 250.0 }   // 7: Brown
};

// ESP-NOW Datenstruktur
typedef struct struct_message {
  uint8_t colorID;
} struct_message;

struct_message receivedData;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); // WiFi-Modus für ESP-NOW initialisieren
  
  // Hardware-Serial 1 für das Feetech-Servo-Bussystem konfigurieren
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;

  // ESP-NOW initialisieren und Empfangs-Callback registrieren
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);

  // Roboterkalibrierung und Startpositionierung ausführen
  s2AutoConfig();
  initArmPosition();
}

void loop() {
  // Endlosschleife: Farbe scannen, aufheben, farbabhängig sortieren
  int colorID = pickUpAndReadColor();
  dropBlock(colorID);
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  
  if (receivedData.colorID < 8) {
    unsigned long currentTime = millis();
    // Entprellung / Rauschfilter: Nur alle 100ms einen Wert akzeptieren
    if (currentTime - lastSampleTime >= 100) {
      colorBuffer[bufferIndex] = receivedData.colorID;
      bufferIndex = (bufferIndex + 1) % 20; // puffer weiterbewegen
      lastSampleTime = currentTime;
    }
  }
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

int16_t angleGenOut(float angleInput) {
    return (int16_t)(angleInput * 4096.0 / 360.0 + 0.5);
}

void setMode(uint8_t id, uint8_t mode) {
  if (mode == 0) {
    st.unLockEprom(id);
    st.writeWord(id, 11, 4095); // Maximalen Positionsbereich freigeben
    st.writeByte(id, SMS_STS_MODE, mode);
    st.LockEprom(id);
  } else if (mode == 3) {
    st.unLockEprom(id);
    st.writeByte(id, SMS_STS_MODE, mode);
    st.writeWord(id, 11, 0);    // Geschwindigkeitsmodus aktivieren
    st.LockEprom(id);
  }
}

void setMiddle(uint8_t id) {
  st.CalibrationOfs(id);
}

void servoStop(uint8_t id) {
  st.EnableTorque(id, 0); // Drehmoment aus
  delay(10);
  st.EnableTorque(id, 1); // Drehmoment an
}

int16_t readServoLoad(uint8_t id) {
  if (st.FeedBack(id) != -1) {
    return st.ReadLoad(-1); // Gibt die Last zurück
  }
  return 0;
}

void s2AutoConfig() {
  uint8_t configIDs[4] = {1, 3, 4, 5};
  int16_t configTargets[4] = {
    angleGenOut(180),
    angleGenOut(270),
    angleGenOut(90), 
    angleGenOut(180) 
  };

  // Servos in sichere Ausgangsposition fahren
  for (uint8_t i = 0; i < 4; i++) {
    st.WritePosEx(configIDs[i], configTargets[i], 100, 150);
  }
  
  // Warten, bis alle Servos ihre Zielposition erreicht haben
  bool allAtTarget = false;
  while (!allAtTarget) {
    bool signalLost = false;
    allAtTarget = true;
    
    for (uint8_t i = 0; i < 4; i++) {
      int16_t currentPos = st.ReadPos(configIDs[i]);
      if (currentPos == -1) { // Verbindung verloren
        signalLost = true;
        allAtTarget = false;
        break;
      }
      if (abs(currentPos - configTargets[i]) > positionTolerance) {
        allAtTarget = false;
      }
    }
    
    if (signalLost) {
      Serial.println("WARNUNG: Signal verloren oder Servo offline! Befehl wird neu gesendet...");
      for (uint8_t i = 0; i < 4; i++) {
        st.WritePosEx(configIDs[i], configTargets[i], 100, 150);
      }
      delay(40);
    } else if (!allAtTarget) {
      delay(50);
    }
  }
  
  // --- Homing für Achse 2 ---
  setMode(2, 3); // Achse 2 in Radmodus schalten
  st.WritePosEx(2, -20000, 500, 50); // Fahre langsam rückwärts gegen den Anschlag
  
  while (true) {
    int16_t currentLoad = readServoLoad(2);
    // Wenn mechanischer Widerstand überschritten wird, ist der Anschlag erreicht
    if (currentLoad > torqueThreshold) {
      servoStop(2); // Bewegung stoppen
      st.WritePosEx(2, reverseRange, 1500, 150); // Fahre definierten Abstand nach vorne frei
      delay(11000);
      setMiddle(2); // Diese freigefahrene Position ist der neue Nullpunkt
      setMode(2, 0); // Zurück in den präzisen Positionsmodus
      break;
    }
    delay(50);
  }
}

bool calculateXYPlane(double x, double y) {
  y = -y; // Koordinatensystem-Spiegelung falls mechanisch nötig
  double squaredDistance = x * x + y * y;
  double effectiveArmLength;
  double Angle;

  if (y == 0) {
    Angle = 90.0 + asin(LEN_H / x) * RAD_TO_DEG;
    effectiveArmLength = sqrt(squaredDistance);
  } else {
    double angleToAxis = atan(x / fabs(y)) * RAD_TO_DEG;
    double offsetAngleH = acos(LEN_H / sqrt(squaredDistance)) * RAD_TO_DEG;
    effectiveArmLength = sqrt(squaredDistance - LEN_H * LEN_H);
    
    if (y > 0) {
      Angle = offsetAngleH + angleToAxis - 90.0;
    } else {
      Angle = 90.0 - angleToAxis + offsetAngleH;
    }
  }

  if (!isnan(Angle)) {
    InfoBuffer[angle_1] = Angle + 90.0;                 // Berechneter Winkel für Basis-Servo 1
    InfoBuffer[len_totalXY] = effectiveArmLength - LEN_B; // Bereinigte Resthorizontale für die Gelenkkette
    return false; // Berechnung erfolgreich
  }
  return true; // Mathematisch nicht erreichbar
}

bool calculateGripper(double theta) {
  double JointAngle;
  double correctionLengthX;
  double correctionLengthY;
  double jointAngleRad = theta * DEG_TO_RAD;

  if(theta == 90){
    JointAngle = theta - LEN_G;
    correctionLengthX = LEN_E;
    correctionLengthY = LEN_F;
  }
  else if(theta < 90){
    JointAngle = 90 - theta;
    double AngleRad = JointAngle * DEG_TO_RAD;
    correctionLengthX = cos(jointAngleRad) * LEN_F + cos(AngleRad) * LEN_E;
    correctionLengthY = sin(jointAngleRad) * LEN_F - sin(AngleRad) * LEN_E;
    JointAngle    = -JointAngle;
  }
  else if(theta > 90){
    JointAngle = LEN_G - (180 - theta);
    double AngleRad = JointAngle * DEG_TO_RAD;
    correctionLengthX = -cos(M_PI - jointAngleRad) * LEN_F + cos(AngleRad) * LEN_E;
    correctionLengthY = sin(M_PI - jointAngleRad) * LEN_F + sin(AngleRad) * LEN_E;
  }

  if(!isnan(JointAngle) && !isnan(correctionLengthX) && !isnan(correctionLengthY)){
    InfoBuffer[angle_EoAT] = JointAngle;
    InfoBuffer[len_a] = correctionLengthX;
    InfoBuffer[len_b] = correctionLengthY;
    return false;
  }
  return true;
}

bool calculateJointChain(double chainTargetX, double chainTargetY){
  double shoulderJointAngle;
  double elbowJointAngle;
  double jointCorrectionAngle;

  if(chainTargetY == 0){
    double angleBaseToTarget = acos((LEN_C * LEN_C + chainTargetX * chainTargetX - LEN_D * LEN_D) / (2 * LEN_C * chainTargetX)) * RAD_TO_DEG;
    shoulderJointAngle = 90 - angleBaseToTarget;
    elbowJointAngle    = angleBaseToTarget + acos((chainTargetX * chainTargetX + LEN_D * LEN_D - LEN_C * LEN_C) / (2 * chainTargetX * LEN_D)) * RAD_TO_DEG;
  }
  else {
    double squaredDistance = chainTargetX * chainTargetX + chainTargetY * chainTargetY;
    double directDistance = sqrt(squaredDistance);
    double planeSlopeAngle = atan(chainTargetY / chainTargetX) * RAD_TO_DEG;
    
    // Kosinussatz zur Winkelberechnung im Dreieck
    double angleBaseToTarget = acos((LEN_C * LEN_C + squaredDistance - LEN_D * LEN_D) / (2 * LEN_C * directDistance)) * RAD_TO_DEG;
    shoulderJointAngle = 90 - planeSlopeAngle - angleBaseToTarget;
    elbowJointAngle = angleBaseToTarget + acos((LEN_D * LEN_D + squaredDistance - LEN_C * LEN_C) / (2 * directDistance * LEN_D)) * RAD_TO_DEG;
  }
  // Bestimmt den Korrekturwinkel für das nachfolgende Handgelenk
  // Damit die Orientierung des Greifers im Raum stabil bleibt
  jointCorrectionAngle = 90 - shoulderJointAngle - elbowJointAngle;

  if(!isnan(shoulderJointAngle) && !isnan(elbowJointAngle) && !isnan(jointCorrectionAngle)){
    InfoBuffer[angle_2] = shoulderJointAngle;
    InfoBuffer[angle_3] = elbowJointAngle;
    InfoBuffer[angle_IKE] = jointCorrectionAngle;
    return false;
  }
  return true;
}

bool ctrlIK(double x, double y, double z, double theta) {
  theta = theta / 2.0; // Skalierungsfaktor für die Handgelenkorientierung
  
  // 1. Basis-Drehung und Greifer-Verschiebung berechnen
  if (calculateXYPlane(x, y) || calculateGripper(theta)) {
    return true; // Fehler, außerhalb des Arbeitsraums
  }
  
  // 2. Hauptachsen (Schulter & Ellenbogen) berechnen, indem Greiferversatz abgezogen wird
  if(calculateJointChain((InfoBuffer[len_totalXY] - InfoBuffer[len_a]), (z - LEN_A + InfoBuffer[len_b]))){
    return true; 
  }
  
  // 3. Orientierungswinkel für das Handgelenk (Servo 4) zusammensetzen
  InfoBuffer[angle_4] = InfoBuffer[angle_IKE] + InfoBuffer[angle_EoAT];
  return false; // Berechnung fehlerfrei
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

void controlGripperRaw(int16_t targetPos, uint16_t speed) {
  st.WritePosEx(gripperID, targetPos, speed, 50);
}

void openGripper(uint16_t speed) {
  int16_t targetPos = 1300;
  controlGripperRaw(targetPos, speed);
  delay(20);
  
  int16_t lastPos = -1;
  uint32_t lastCheckTime = millis();
  bool atTarget = false;
  
  while (!atTarget) {
    delay(30);
    int16_t currentPos = st.ReadPos(gripperID);
    
    if (currentPos == -1) {
      Serial.println("WARNUNG: Greifer-Signal verloren oder Greifer-Servo offline! Befehl wird neu gesendet...");
      controlGripperRaw(targetPos, speed);
      continue;
    }
    if (abs(currentPos - targetPos) < positionTolerance) {
      atTarget = true;
      break; 
    }
    // Falls der Greifer mechanisch blockiert und sich 150ms nicht bewegt, stoppen
    if (millis() - lastCheckTime > 150) {
      if (lastPos != -1 && abs(currentPos - lastPos) < 8) { 
        st.WritePosEx(gripperID, currentPos, 1000, 50); // Fixiere aktuelle Position
        atTarget = true;
        break; 
      }
      lastPos = currentPos;
      lastCheckTime = millis();
    }
  }
}

void closeGripper(uint16_t speed) {
  int16_t targetPos = 2300;
  controlGripperRaw(targetPos, speed);
  delay(20);
  
  int16_t lastPos = -1;
  uint32_t lastCheckTime = millis();
  bool atTarget = false;
  
  while (!atTarget) {
    delay(30);
    int16_t currentPos = st.ReadPos(gripperID);
    
    if (currentPos == -1) {
      Serial.println("WARNUNG: Greifer-Signal verloren oder Greifer-Servo offline! Befehl wird neu gesendet...");
      controlGripperRaw(targetPos, speed);
      continue;
    }
    if (abs(currentPos - targetPos) < positionTolerance) {
      atTarget = true;
      break; 
    }
    // Greif-Erkennung: Wenn sich die Finger 150ms nicht mehr bewegen, greift er das Objekt fest
    if (millis() - lastCheckTime > 150) {
      if (lastPos != -1 && abs(currentPos - lastPos) < 8) { 
        st.WritePosEx(gripperID, currentPos, 1000, 50); // Kraft halten
        delay(150);
        atTarget = true;
        break; 
      }
      lastPos = currentPos;
      lastCheckTime = millis();
    }
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