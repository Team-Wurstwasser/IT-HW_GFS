#include <Arduino.h>
#include <SCServo.h>
#include "../config.h"
#include "gripper.h"

extern SMS_STS st;

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