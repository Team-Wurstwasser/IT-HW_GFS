#include <Arduino.h>
#include <SCServo.h>
#include "../config.h"
#include "calibration.h"

extern SMS_STS st;

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