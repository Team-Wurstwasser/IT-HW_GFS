#ifndef GRIPPER_H
#define GRIPPER_H

#include <Arduino.h>
#include <SCServo.h>

void controlGripperRaw(int16_t targetPos, uint16_t speed);
void openGripper(uint16_t speed = 2000);
void closeGripper(uint16_t speed = 2000);

#endif