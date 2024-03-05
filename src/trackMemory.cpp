#include "trackMemory.hpp"


void trackMemory::storeToCar(point& pathPoint) {
    carPath.push_back(pathPoint);
}

void trackMemory::storeToLine(point& pathPoint) {
    linePath.push_back(pathPoint);
}

void trackMemory::printStoredPoints() {
    Serial.print("Car: ");
    for (const auto& point : carPath) {
        Serial.print("Time: ");
        Serial.print(point.timeStamp);
        Serial.print(", x: ");
        Serial.print(point.x);
        Serial.print(", y: ");
        Serial.print(point.y);
    }
    Serial.print("Line: ");
    for (const auto& point : linePath) {
        Serial.print("Time: ");
        Serial.print(point.timeStamp);
        Serial.print(", x: ");
        Serial.print(point.x);
        Serial.print(", y: ");
        Serial.print(point.y);
    }
    Serial.println();
}

void trackMemory::reset() {
    carPath.clear();
    linePath.clear();
}