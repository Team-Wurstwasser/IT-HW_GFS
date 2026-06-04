#include <SCServo.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>

const uint8_t S_RXD = 18;
const uint8_t S_TXD = 19;

// Speicher für die empfangenen Daten
volatile uint8_t wertSpeicher[20] = {0};
volatile uint8_t speicherIndex = 0;
volatile unsigned long letzteStichprobeZeit = 0;

SMS_STS st;

const float LEN_A = 115.432, LEN_B = 41.0513, LEN_C = 168.8579, LEN_D = 127.9234, LEN_E = 108.5357, LEN_F = 7.7076, LEN_G = 90.0, LEN_H = -13.75;
float InfoBuffer[10];
int16_t STPos[4];
int STDirection[4] = {1, 1, 1, -1};
uint16_t STSpd[4] = {3000, 3000, 3400, 3400};
uint8_t STAc[4] = {50, 50, 50, 50};
uint8_t STID[4] = {1, 2, 3, 4};
const uint8_t gripperID = 5;

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
  
  if (eingehendeDaten.colorID < 8) {
    unsigned long jetzt = millis();
    
    if (jetzt - letzteStichprobeZeit >= 100) {
      wertSpeicher[speicherIndex] = eingehendeDaten.colorID;
      speicherIndex = (speicherIndex + 1) % 20; 
      letzteStichprobeZeit = jetzt;
    }
  }
}

uint8_t holeGefilterteFarbe() {
  uint32_t farbZaehler[8] = {0};
  
  uint8_t lokalerSpeicher[20];
  for (uint8_t i = 0; i < 20; i++) {
    lokalerSpeicher[i] = wertSpeicher[i];
  }
  
  for (uint8_t i = 0; i < 20; i++) {
    farbZaehler[lokalerSpeicher[i]]++;
  }
  
  uint8_t haeufigsteFarbeID = 0;
  uint32_t maximaleAnzahl = 0;
  
  for (uint8_t i = 0; i < 8; i++) {
    if (farbZaehler[i] > maximaleAnzahl) {
      maximaleAnzahl = farbZaehler[i];
      haeufigsteFarbeID = i;
    }
  }
  
  return haeufigsteFarbeID;
}

void setTorque(bool enable) {
  uint8_t cmd = enable ? 1 : 0;
  for(uint8_t id = 1; id <= 5; ++id) {
    st.EnableTorque(id, cmd);
  }
}

float angleGenOut(float angleInput) { return angleInput * 4096.0 / 360.0; }

void wigglePlaneIK(double LA, double aIn, double bIn) {
  double LB;
  double L2C;
  double LC;
  double alpha;
  double beta;
  double lambda;
  double psi;
  bIn = -bIn;
  if(bIn > 0){
    L2C = aIn * aIn + bIn * bIn;
    LC = sqrt(L2C);
    lambda = atan(aIn/bIn) * 180 / M_PI;
    psi = acos(LA/LC) * 180 / M_PI;
    LB = sqrt(L2C - LA * LA);
    alpha = psi + lambda - 90;
  }
  else if(bIn == 0){
    alpha = 90+ asin(LA/aIn) * 180 / M_PI;
    L2C = aIn * aIn + bIn * bIn;
    LB = sqrt(L2C);
  }
  else if(bIn < 0){
    bIn = -bIn;
    L2C = aIn * aIn + bIn * bIn;
    LC = sqrt(L2C);
    lambda = atan(aIn/bIn) * 180 / M_PI;
    psi = acos(LA/LC) * 180 / M_PI;
    LB = sqrt(L2C - LA * LA);
    alpha = 90 - lambda + psi;
  }
  InfoBuffer[1] = alpha + 90;
  InfoBuffer[9] = LB - LEN_B;
}

void EoAT_IK(float angleInput) {
  float betaGenOut = (angleInput > 90) ? (LEN_G - (180 - angleInput)) : (angleInput < 90 ? -(90 - angleInput) : 0);
  InfoBuffer[6] = betaGenOut;
  InfoBuffer[7] = LEN_E;
  InfoBuffer[8] = LEN_F;
}

void simpleLinkageIK(double LA, double LB, double aIn, double bIn) {
  double L2C = aIn * aIn + bIn * bIn, LC = sqrt(L2C);
  double psi = acos((LA * LA + L2C - LB * LB) / (2 * LA * LC)) * 180 / M_PI;
  double omega = acos((LB * LB + L2C - LA * LA) / (2 * LC * LB)) * 180 / M_PI;
  InfoBuffer[2] = 90 - atan(bIn / aIn) * 180 / M_PI - psi;
  InfoBuffer[3] = psi + omega;
  InfoBuffer[5] = 90 - InfoBuffer[2] - InfoBuffer[3];
}

void ctrlIK(float x, float y, float z, float theta) {
  theta = theta / 2.0;
  wigglePlaneIK(LEN_H, x, y);
  EoAT_IK(theta);
  simpleLinkageIK(LEN_C, LEN_D, (InfoBuffer[9] - InfoBuffer[7]), (z - LEN_A + InfoBuffer[8]));
  InfoBuffer[4] = InfoBuffer[5] + InfoBuffer[6];
}

void ctrlMove() {
  STPos[0] = (int)(angleGenOut(STDirection[0] * InfoBuffer[1]) + 0.5);
  STPos[1] = (int)(angleGenOut(STDirection[1] * InfoBuffer[2] + 15) * 3 + 0.5);
  STPos[2] = (int)(angleGenOut(STDirection[2] * InfoBuffer[3] + 180) + 0.5);
  STPos[3] = (int)(angleGenOut(STDirection[3] * InfoBuffer[4] + 180) + 0.5);
  
  st.SyncWritePosEx(STID, 4, STPos, STSpd, STAc);
}

void moveToCoordinate(float x, float y, float z, float theta){
  ctrlIK(x, y, z, theta);
  ctrlMove();
}

void moveServo2(float x, float y, float z, float theta) {
  ctrlIK(x, y, z, theta); 
  int16_t zielPosServo2 = (int)(angleGenOut(STDirection[1] * InfoBuffer[2] + 15) * 3 + 0.5);
  st.WritePosEx(STID[1], zielPosServo2, STSpd[1], STAc[1]);
}

void controlGripperRaw(int16_t targetPos, uint16_t speed) {
  st.WritePosEx(gripperID, targetPos, speed, 50); 
}

void openGripper(uint16_t speed = 2000) {
  int16_t targetPos = 1300;
  
  controlGripperRaw(targetPos, speed);
  
  int16_t lastPos = st.ReadPos(gripperID);
  uint32_t startTime = millis();
  uint32_t lastCheckTime = millis();
  
  while (millis() - startTime < 3000) {
    delay(30);
    
    int16_t currentPos = st.ReadPos(gripperID);
    if (currentPos == -1) continue;

    if (abs(currentPos - targetPos) < 15) {
      break; 
    }
    
    if (millis() - lastCheckTime > 150) {
      if (abs(currentPos - lastPos) < 8) { 
        st.WritePosEx(gripperID, currentPos, 1000, 50);
        delay(50);
        st.EnableTorque(gripperID, 0); 
        delay(50);
        st.EnableTorque(gripperID, 1);
        delay(50);
        break; 
      }
      lastPos = currentPos;
      lastCheckTime = millis();
    }
  }
}

void closeGripper(uint16_t speed = 2000) {
  int16_t targetPos = 2300;
  
  controlGripperRaw(targetPos, speed);
  
  int16_t lastPos = st.ReadPos(gripperID);
  uint32_t startTime = millis();
  uint32_t lastCheckTime = millis();
  
  while (millis() - startTime < 3000) {
    delay(30);
    
    int16_t currentPos = st.ReadPos(gripperID);
    if (currentPos == -1) continue;

    if (abs(currentPos - targetPos) < 15) {
      break; 
    }
    
    if (millis() - lastCheckTime > 150) {
      if (abs(currentPos - lastPos) < 8) { 
        st.WritePosEx(gripperID, currentPos, 1000, 50);
        delay(150);
        break; 
      }
      lastPos = currentPos;
      lastCheckTime = millis();
    }
  }
}

uint8_t holeSteinUndLeseFarbe() {
  moveToCoordinate(20.0, 288.0, 100.0, 160.0);
  delay(1200);

  moveToCoordinate(20.0, 288.0, -15.0, 160.0);
  delay(4000); 

  uint8_t erkannteFarbeID = holeGefilterteFarbe();

  if (erkannteFarbeID < 8) {
    Serial.print("Farbe: ");
    Serial.println(tableColor[erkannteFarbeID]);
  }

  moveServo2(20.0, 288.0, 100.0, 160.0);
  delay(800);

  moveToCoordinate(20.0, 360.0, 65.0, 295.0);
  delay(800); 
  closeGripper();

  moveServo2(20.0, 350.0, 140.0, 295.0);
  delay(800);

  return erkannteFarbeID;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;

  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
}

void gebeSteinAb(int id) {
  int farbeID = id;

  switch (farbeID) {
    case 0: 
      moveToCoordinate(200.0, 360.0, 140.0, 295.0);
      break;
    case 1: 
      moveToCoordinate(200.0, 360.0, 140.0, 295.0);
      break;
    case 2: 
      moveToCoordinate(200.0, 360.0, 140.0, 295.0);
      break;
    case 3: 
      moveToCoordinate(200.0, 360.0, 140.0, 295.0);
      break;
    case 4: 
      moveToCoordinate(200.0, 360.0, 140.0, 295.0);
      break;
    case 5: 
      moveToCoordinate(200.0, 360.0, 140.0, 295.0);
      break;
    case 6: 
      moveToCoordinate(200.0, 360.0, 140.0, 295.0);
      break;
    case 7: 
      moveToCoordinate(200.0, 360.0, 140.0, 295.0);
      break;
  }

  delay(1200);
  openGripper();
}

void loop() {
  int farbID = holeSteinUndLeseFarbe();
  gebeSteinAb(farbID);
  delay(2000);
}