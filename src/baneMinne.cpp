#include "baneMinne.hpp"

void baneMinne::storePoint(double leftMotorRPM, double rightMotorRPM, long leftEncCount, long rightEncCount) {
    pathPoint point;
    point.time = millis();
    point.leftEncoderCount = leftEncCount;
    point.rightEncoderCount = rightEncCount;
    point.leftMotorSpeed = leftMotorRPM;
    point.rightMotorSpeed = rightMotorRPM;

    path.push_back(point);
}