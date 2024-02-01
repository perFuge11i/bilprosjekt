#include "baneMinne.hpp"

void baneMinne::storePathPoint(long leftEncCount, long rightEncCount, double leftMotorRPM, double rightMotorRPM, double pidError) {
    if (pathIndex < hvormangepunkterkanvihaminnemessig) {
        PathPoint point;
        point.time = millis();
        point.leftEncoderCount = leftEncCount;
        point.rightEncoderCount = rightEncCount;
        point.leftMotorSpeed = leftMotorRPM;
        point.rightMotorSpeed = rightMotorRPM;
        point.pidError = pidError;

        path[pathIndex++] = point;
    }
}