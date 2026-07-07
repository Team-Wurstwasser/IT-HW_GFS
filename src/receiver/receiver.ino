#include <SCServo.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>

const uint8_t S_RXD = 18;
const uint8_t S_TXD = 19;

// Speicher für die empfangenen Daten
volatile uint8_t colorBuffer[20] = {0};
volatile uint8_t bufferIndex = 0;
volatile unsigned long lastSampleTime = 0;

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
const int positionTolerance = 25;

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

struct Coordinate { float x, y, z, theta; };

const Coordinate PICKUP_HOVER = {0.0, 330.0, 200.0, 130.0}; // Über dem Stein schweben
const Coordinate PICKUP_SCAN  = {0.0, 330.0,  85.0, 130.0}; // Zum Scannen absenken
const Coordinate PICKUP_GRAB  = {0.0, 351.0,  70.0, 250.0}; // Position zum Greifen
const Coordinate PICKUP_LIFT  = {0.0, 351.0, 200.0, 250.0}; // Stein anheben

Coordinate dropLocations[8] = {
  { 250.0, -70.0, 95.0, 250.0 },  // 0: No color (Default)
  { 250.0, -70.0, 95.0, 250.0 },  // 1: Black (Default)
  { -70.0, -315.0, 95.0, 250.0 }, // 2: Blue
  {  25.0, -315.0, 95.0, 250.0 }, // 3: Green
  { -70.0, -250.0, 95.0, 250.0 }, // 4: Yellow
  {  25.0, -250.0, 95.0, 250.0 }, // 5: Red
  { 250.0,   25.0, 95.0, 250.0 }, // 6: White
  { 250.0, -70.0, 95.0, 250.0 }   // 7: Brown (Default)
};

typedef struct struct_message {
  uint8_t colorID;
} struct_message;

struct_message receivedData;

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  
  if (receivedData.colorID < 8) {
    unsigned long currentTime = millis();
    
    if (currentTime - lastSampleTime >= 100) {
      colorBuffer[bufferIndex] = receivedData.colorID;
      bufferIndex = (bufferIndex + 1) % 20; 
      lastSampleTime = currentTime;
    }
  }
}

uint8_t getFilteredColor() {
  uint32_t colorCounter[8] = {0};
  
  for (uint8_t i = 0; i < 20; i++) {
    colorCounter[colorBuffer[i]]++;
  }
  
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
  uint8_t configIDs[4] = {1, 3, 4, 5};
  int16_t configTargets[4] = {
    (int)(angleGenOut(180) + 0.5), // Ziel für ID 1
    (int)(angleGenOut(270) + 0.5), // Ziel für ID 3
    (int)(angleGenOut(90) + 0.5),  // Ziel für ID 4
    (int)(angleGenOut(180) + 0.5)  // Ziel für ID 5
  };

  for (uint8_t i = 0; i < 4; i++) {
    st.WritePosEx(configIDs[i], configTargets[i], 100, 150);
  }
  
  bool allAtTarget = false;
  
  while (!allAtTarget) {
    bool signalLost = false;
    allAtTarget = true;
    
    for (uint8_t i = 0; i < 4; i++) {
      int16_t currentPos = st.ReadPos(configIDs[i]);
      
      if (currentPos == -1) {
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
    }
    else if (!allAtTarget) {
      delay(50); 
    }
  }
  
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

bool calculateXYPlane(double x, double y) {
  y = -y;
  
  double squaredDistance = x * x + y * y;
  double effectiveArmLength;
  double Angle;

  if (y == 0) {
    Angle = 90.0 + asin(LEN_H / x) * RAD_TO_DEG;
    effectiveArmLength = sqrt(squaredDistance);
  } 
  else {
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
    InfoBuffer[angle_1] = Angle + 90.0;
    InfoBuffer[len_totalXY] = effectiveArmLength - LEN_B;
    return false;
  }
  return true;
}

bool calculateGripper(double theta) {
  double JointAngle;
  double correctionLengthX;
  double correctionLengthY;

  double jointAngleRad = theta * DEG_TO_RAD;

  if(theta == 90){
    JointAngle = theta - LEN_G;
    double AngleRad = JointAngle * DEG_TO_RAD;
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
  else{
    return true;
  }
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
  else{
    double squaredDistance = chainTargetX * chainTargetX + chainTargetY * chainTargetY;
    double directDistance = sqrt(squaredDistance);
    double planeSlopeAngle = atan(chainTargetY / chainTargetX) * RAD_TO_DEG;
    
    double angleBaseToTarget = acos((LEN_C * LEN_C + squaredDistance - LEN_D * LEN_D) / (2 * LEN_C * directDistance)) * RAD_TO_DEG;
    shoulderJointAngle = 90 - planeSlopeAngle - angleBaseToTarget;
    elbowJointAngle = angleBaseToTarget + acos((LEN_D * LEN_D + squaredDistance - LEN_C * LEN_C) / (2 * directDistance * LEN_D)) * RAD_TO_DEG;
  }
  jointCorrectionAngle = 90 - shoulderJointAngle - elbowJointAngle;

  if(!isnan(shoulderJointAngle) && !isnan(elbowJointAngle) && !isnan(jointCorrectionAngle)){
    InfoBuffer[angle_2] = shoulderJointAngle;
    InfoBuffer[angle_3] = elbowJointAngle;
    InfoBuffer[angle_IKE] = jointCorrectionAngle;
    return false;
  }
  else{
    return true;
  }
}

bool ctrlIK(double x, double y, double z, double theta) {
  theta = theta / 2.0;

  if (calculateXYPlane(x, y) || calculateGripper(theta)) {
    return true;
  }
  
  if(calculateJointChain((InfoBuffer[len_totalXY] - InfoBuffer[len_a]), (z - LEN_A + InfoBuffer[len_b]))){
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
          allAtTarget = false;
        }
      }
      
      if (signalLost) {
        Serial.println("WARNUNG: Signal verloren oder Servo offline! Befehl wird neu gesendet...");
        ctrlMove();
        delay(40);
      }
      else if (!allAtTarget) {
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
    
    if (millis() - lastCheckTime > 150) {
      if (lastPos != -1 && abs(currentPos - lastPos) < 8) { 
        st.WritePosEx(gripperID, currentPos, 1000, 50);
        atTarget = true;
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
    
    if (millis() - lastCheckTime > 150) {
      if (lastPos != -1 && abs(currentPos - lastPos) < 8) { 
        st.WritePosEx(gripperID, currentPos, 1000, 50);
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

  delay(4000);
  uint8_t detectedColorID = getFilteredColor();

  Serial.print("Color: ");
  Serial.println(tableColor[detectedColorID]);

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
  int colorID = pickUpAndReadColor();
  dropBlock(colorID);
}