#include <Arduino.h>
#include <math.h>
#include "../config.h"

extern float InfoBuffer[9];

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