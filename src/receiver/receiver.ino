#include <SCServo.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>

const uint8_t S_RXD = 18;
const uint8_t S_TXD = 19;

volatile bool espNowGesperrt = false;

// Speicher für die empfangenen Daten
volatile uint8_t wertSpeicher[20];
volatile uint8_t speicherIndex = 0;
volatile unsigned long letzteStichprobeZeit = 0;

SMS_STS st;

const float LEN_A = 115.432, LEN_B = 41.0513, LEN_C = 168.8579, LEN_D = 127.9234, LEN_E = 108.5357, LEN_F = 7.7076, LEN_G = 90.0, LEN_H = -13.75;
const uint8_t angle_1 = 0, angle_2 = 1, angle_3 = 2, angle_4 = 3, angle_IKE = 4, angle_EoAT  = 5, len_a = 6, len_b = 7, len_totalXY = 8;

float InfoBuffer[9];
int16_t STPos[4];
int STDirection[4] = {1, 1, 1, -1};
uint16_t STSpd[4];
uint8_t STAc[4] = {50, 50, 50, 50};
uint8_t STID[4] = {1, 2, 3, 4};
const uint8_t STID_count = 4;
const uint8_t gripperID = 5;

const int torqueThreshold = 300;
const int reverseRange = 4900;

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
  if (espNowGesperrt) return; 

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
  uint32_t farbZaehler[8];
  
  espNowGesperrt = true; 
  for (uint8_t i = 0; i < 20; i++) {
    farbZaehler[wertSpeicher[i]]++;
  }
  espNowGesperrt = false; 
  
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

float angleGenOut(float angleInput) { return angleInput * 4096.0 / 360.0; }

void setMode(uint8_t id, uint8_t mode) {
  if (mode == 0) {
    st.unLockEprom(id);
    st.writeWord(id, 11, 4095);
    st.writeByte(id, SMS_STS_MODE, mode);
    st.LockEprom(id);
  } else if (mode == 3) {
    st.unLockEprom(id);
    st.writeByte(id, SMS_STS_MODE, mode);
    st.writeWord(id, 11, 0);
    st.LockEprom(id);
  }
}

void setMiddle(uint8_t id) {
  st.CalibrationOfs(id);
}

void servoStop(uint8_t id) {
  st.EnableTorque(id, 0);
  delay(10);
  st.EnableTorque(id, 1);
}

int16_t readServoLoad(uint8_t id) {
  if (st.FeedBack(id) != -1) {
    return st.ReadLoad(-1);
  }
  return 0;
}

void s2AutoConfig() {
  st.WritePosEx(1, (int)(angleGenOut(180) + 0.5), 100, 150);
  st.WritePosEx(3, (int)(angleGenOut(270) + 0.5), 100, 150);
  st.WritePosEx(4, (int)(angleGenOut(90) + 0.5), 100, 150);
  st.WritePosEx(5, (int)(angleGenOut(180) + 0.5), 100, 150);
  delay(8000);
  
  setMode(2, 3);
  
  st.WritePosEx(2, -20000, 500, 50);
  
  while (true) {
    int16_t currentLoad = readServoLoad(2);
    
    if (currentLoad > torqueThreshold) {
      servoStop(2);
      st.WritePosEx(2, reverseRange, 1500, 150);
      delay(11000);
      setMiddle(2);
      setMode(2, 0);
      break;
    }
    delay(50);
  }
}

bool wigglePlaneIK(double LA, double aIn, double bIn) {
  bIn = -bIn;
  
  double L2C = aIn * aIn + bIn * bIn;
  double LC = sqrt(L2C);
  double LB = 0;
  double alpha = NAN;

  if (bIn == 0) {
    alpha = 90.0 + asin(LA / aIn) * RAD_TO_DEG;
    LB = sqrt(L2C);
  } 
  else {
    double absB = fabs(bIn);
    double lambda = atan(aIn / absB) * RAD_TO_DEG;
    double psi = acos(LA / LC) * RAD_TO_DEG;
    LB = sqrt(L2C - LA * LA);

    if (bIn > 0) {
      alpha = psi + lambda - 90.0;
    } else {
      alpha = 90.0 - lambda + psi;
    }
  }

  if (!isnan(alpha)) {
    InfoBuffer[angle_1] = alpha + 90.0;
    InfoBuffer[len_totalXY] = LB - LEN_B;
    return false;
  }
  return true;
}

bool EoAT_IK(double angleInput){
  double betaGenOut;
  double betaRad;
  double angleRad;
  double aGenOut;
  double bGenOut;

  angleRad = angleInput * DEG_TO_RAD;

  if(angleInput == 90){
    betaGenOut = angleInput - LEN_G;
    betaRad    = betaGenOut * DEG_TO_RAD;
    aGenOut = LEN_E;
    bGenOut = LEN_F;
  }
  else if(angleInput < 90){
    betaGenOut = 90 - angleInput;
    betaRad    = betaGenOut * DEG_TO_RAD;
    aGenOut = cos(angleRad) * LEN_F + cos(betaRad) * LEN_E;
    bGenOut = sin(angleRad) * LEN_F - sin(betaRad) * LEN_E;
    betaGenOut = -betaGenOut;
  }
  else if(angleInput > 90){
    betaGenOut = LEN_G - (180 - angleInput);
    betaRad    = betaGenOut * DEG_TO_RAD;
    aGenOut = - cos(M_PI - angleRad) * LEN_F + cos(betaRad) * LEN_E;
    bGenOut = sin(M_PI - angleRad) * LEN_F + sin(betaRad) * LEN_E;
  }
  if(!isnan(betaGenOut) && !isnan(aGenOut) && !isnan(bGenOut)){
    InfoBuffer[angle_EoAT] = betaGenOut;
    InfoBuffer[len_a] = aGenOut;
    InfoBuffer[len_b] = bGenOut;
    return false;
  }
  else{
    return true;
  }
}

bool simpleLinkageIK(double LA, double LB, double aIn, double bIn){
  double psi;
  double alpha;
  double omega;
  double beta;
  double L2C;
  double LC;
  double lambda;
  double delta;
  if(bIn == 0){
    psi   = acos((LA * LA + aIn * aIn - LB * LB) / (2 * LA * aIn)) * RAD_TO_DEG;
    alpha = 90 - psi;
    omega = acos((aIn * aIn + LB * LB - LA * LA) / (2 * aIn * LB)) * RAD_TO_DEG;
    beta  = psi + omega;
  }
  else{
    L2C = aIn * aIn + bIn * bIn;
    LC  = sqrt(L2C);
    lambda = atan(bIn / aIn) * RAD_TO_DEG;
    psi    = acos((LA * LA + L2C - LB * LB) / (2 * LA * LC)) * RAD_TO_DEG;
    alpha = 90 - lambda - psi;
    omega = acos((LB * LB + L2C - LA * LA) / (2 * LC * LB)) * RAD_TO_DEG;
    beta  = psi + omega;
  }
  delta = 90 - alpha - beta;

  if(!isnan(alpha) && !isnan(beta) && !isnan(delta)){
    InfoBuffer[angle_2]   = alpha;
    InfoBuffer[angle_3]   = beta;
    InfoBuffer[angle_IKE] = delta;
    return false;
  }
  else{
    return true;
  }
}

bool ctrlIK(double x, double y, double z, double theta) {
  theta = theta / 2.0;

  if (wigglePlaneIK(LEN_H, x, y) || EoAT_IK(theta)) {
    return true;
  }
  
  if(simpleLinkageIK(LEN_C, LEN_D, (InfoBuffer[len_totalXY] - InfoBuffer[len_a]), (z - LEN_A + InfoBuffer[len_b]))){
    return true;
  }
  
  InfoBuffer[angle_4] = InfoBuffer[angle_IKE] + InfoBuffer[angle_EoAT];
  return false;
}

void ctrlMove() {
  STPos[0] = (int)(angleGenOut(STDirection[0] * InfoBuffer[angle_1]) + 0.5);
  STPos[1] = (int)(angleGenOut(STDirection[1] * InfoBuffer[angle_2] + 15) * 3 + 0.5);
  STPos[2] = (int)(angleGenOut(STDirection[2] * InfoBuffer[angle_3] + 180) + 0.5);
  STPos[3] = (int)(angleGenOut(STDirection[3] * InfoBuffer[angle_4] + 180) + 0.5);
  
  st.SyncWritePosEx(STID, STID_count, STPos, STSpd, STAc);
}

void moveToCoordinate(double x, double y, double z, double theta){
  bool nanCtrlIK = ctrlIK(x, y, z, theta);
  if (!nanCtrlIK)
  {
    ctrlMove();
    
    bool alleAmZiel = false;
    
    while (!alleAmZiel) {
      bool signalVerloren = false;
      alleAmZiel = true;
      
      for (uint8_t i = 0; i < STID_count; i++) {
        int16_t aktuellePos = st.ReadPos(STID[i]);
        
        if (aktuellePos == -1) {
          signalVerloren = true;
          alleAmZiel = false;
          break; 
        }
        
        if (abs(aktuellePos - STPos[i]) > 25) {
          alleAmZiel = false;
        }
      }
      
      if (signalVerloren) {
        Serial.println("WARNUNG: Signal verloren oder Servo offline! Befehl wird neu gesendet...");
        ctrlMove();
        delay(40);
      }
      else if (!alleAmZiel) {
        delay(50); 
      }
    }
  }
  else
  {
    Serial.printf("FEHLER: Zielkoordinaten nicht erreichbar! X:%.2f, Y:%.2f, Z:%.2f\n", x, y, z);
  }
}

void controlGripperRaw(int16_t targetPos, uint16_t speed) {
  st.WritePosEx(gripperID, targetPos, speed, 50); 
}

void openGripper(uint16_t speed = 2000) {
  int16_t targetPos = 1300;
  
  controlGripperRaw(targetPos, speed);
  delay(20);
  
  int16_t lastPos = -1;
  uint32_t lastCheckTime = millis();
  uint8_t errorCounter = 0;
  bool AmZiel = false;
  
  while (!AmZiel) {
    delay(30);
    
    int16_t currentPos = st.ReadPos(gripperID);
    
    if (currentPos == -1) {
      Serial.println("WARNUNG: Greifer-Signal verloren oder Greifer-Servo offline! Befehl wird neu gesendet...");
      controlGripperRaw(targetPos, speed);
      continue;
    }

    if (abs(currentPos - targetPos) < 15) {
      AmZiel = true;
      break; 
    }
    
    if (millis() - lastCheckTime > 150) {
      if (lastPos != -1 && abs(currentPos - lastPos) < 8) { 
        st.WritePosEx(gripperID, currentPos, 1000, 50);
        AmZiel = true;
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
  delay(20);
  
  int16_t lastPos = -1; 
  uint32_t lastCheckTime = millis();
  bool AmZiel = false;
  
  while (!AmZiel) {
    delay(30);
    
    int16_t currentPos = st.ReadPos(gripperID);
    
    if (currentPos == -1) {
      Serial.println("WARNUNG: Greifer-Signal verloren oder Greifer-Servo! Befehl wird neu gesendet...");
      controlGripperRaw(targetPos, speed);
      continue;
    }

    if (abs(currentPos - targetPos) < 15) {
      AmZiel = true;
      break; 
    }
    
    if (millis() - lastCheckTime > 150) {
      if (lastPos != -1 && abs(currentPos - lastPos) < 8) { 
        st.WritePosEx(gripperID, currentPos, 1000, 50);
        delay(150);
        AmZiel = true;
        break; 
      }
      lastPos = currentPos;
      lastCheckTime = millis();
    }
  }
}

uint8_t holeSteinUndLeseFarbe() {
  moveToCoordinate(0.0, 330.0, 200.0, 130.0);
  moveToCoordinate(0.0, 330.0, 80.0, 130.0);

  delay(4000); 

  uint8_t erkannteFarbeID = holeGefilterteFarbe();

  if (erkannteFarbeID < 8) {
    Serial.print("Farbe: ");
    Serial.println(tableColor[erkannteFarbeID]);
  }

  moveToCoordinate(0.0, 351.0, 65.0, 250.0);
  closeGripper();

  moveToCoordinate(0.0, 351.0, 200.0, 250.0);

  return erkannteFarbeID;
}

void gebeSteinAb(int id) {
  int farbeID = id;

  switch (farbeID) {
    case 2: 
      moveToCoordinate(-70.0, -315.0, 90.0, 250.0);
      break;
    case 3: 
      moveToCoordinate(25.0, -315.0, 90.0, 250.0);
      break;
    case 4: 
      moveToCoordinate(-70.0, -250.0, 90.0, 250.0);
      break;
    case 5: 
      moveToCoordinate(25.0, -250.0, 90.0, 250.0);
      break;
    case 6: 
      moveToCoordinate(250.0, 25.0, 90.0, 250.0);
      break;
    default:
      moveToCoordinate(250.0, -70.0, 90.0, 250.0);
      break;
  }

  openGripper();
}

void initArmPosition() {
  float startX = LEN_B + LEN_D + LEN_E;
  float startY = LEN_H;
  float startZ = LEN_A + LEN_C - LEN_F;
  float startTheta = 180.0;

  STSpd[0] = 1000; STSpd[1] = 1000; STSpd[2] = 1000; STSpd[3] = 1000;
  moveToCoordinate(startX, startY, startZ, startTheta);

  STSpd[0] = 3000; STSpd[1] = 3000; STSpd[2] = 3400; STSpd[3] = 3400;
  openGripper(1500);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;

  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);

  s2AutoConfig();
  initArmPosition();
}

void loop() {
  int farbID = holeSteinUndLeseFarbe();
  gebeSteinAb(farbID);
}