#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>

bool calculateXYPlane(double x, double y);
bool calculateGripper(double theta);
bool calculateJointChain(double chainTargetX, double chainTargetY);
bool ctrlIK(double x, double y, double z, double theta);

#endif