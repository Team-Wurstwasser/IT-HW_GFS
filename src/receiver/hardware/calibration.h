#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include <SCServo.h>

void setMode(uint8_t id, uint8_t mode);
void setMiddle(uint8_t id);
void servoStop(uint8_t id);
int16_t readServoLoad(uint8_t id);
void s2AutoConfig();

#endif