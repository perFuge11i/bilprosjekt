#include "baneMinne.hpp"


void baneMinne::storePoint(long leftEncCount, long rightEncCount) {
    pathPoint point;
    point.time = millis();
    point.leftEncoderCount = leftEncCount;
    point.rightEncoderCount = rightEncCount;

    path.push_back(point);
}