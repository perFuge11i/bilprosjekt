#include "baneMinne.hpp"


void baneMinne::storePoint(unsigned long leftPulseCount, unsigned long rightPulseCount, unsigned long time) {

    simplePathPoint point;
    point.time = time;
    point.leftPulseCount = leftPulseCount;
    point.rightPulseCount = rightPulseCount;

    path.push_back(point);
}