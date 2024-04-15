#include "printer.hpp"

void printer::setCarPosition(point &carPosition) {
    doc["carX"] = round(carPosition.x*100.0)/100.0;
    doc["carY"] = round(carPosition.y*100.0)/100.0;
}

void printer::setCarDirection(point &carDirection) {
    doc["carDirX"] = round(carDirection.x*100.0)/100.0;
    doc["carDirY"] = round(carDirection.y*100.0)/100.0;
}

void printer::setLinePosition(point &linePosition) {
    doc["lineX"] = round(linePosition.x*100.0)/100.0;
    doc["lineY"] = round(linePosition.y*100.0)/100.0;
}

void printer::setLineDirection(point &lineDirection) {
    doc["lineDirX"] = round(lineDirection.x*100.0)/100.0;
    doc["lineDirY"] = round(lineDirection.y*100.0)/100.0;
}

void printer::print() {
    serializeJson(doc, Serial);
    Serial.println();
    delay(10);
}