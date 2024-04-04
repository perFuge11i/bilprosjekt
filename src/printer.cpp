#include "printer.hpp"

void printer::setCarPosition(point &carPosition) {
    doc["carX"] = carPosition.x;
    doc["carY"] = carPosition.y;
}

void printer::setCarDirection(point &carDirection) {
    doc["dirX"] = carDirection.x;
    doc["dirY"] = carDirection.y;
}

void printer::setLinePosition(point &linePosition) {
    doc["lineX"] = linePosition.x;
    doc["lineY"] = linePosition.y;
}

void printer::print() {
    serializeJson(doc, Serial);
    Serial.println();
    delay(10);
}