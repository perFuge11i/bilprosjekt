#include "printer.hpp"

void printer::setCarPosition(point &carPosition) {
    plotData[0] = carPosition.x;
    plotData[1] = carPosition.y;
}

void printer::setCarDirection(point &carDirection) {
    plotData[2] = carDirection.x;
    plotData[3] = carDirection.y;
}

void printer::setLinePosition(point &linePosition) {
    plotData[4] = linePosition.x;
    plotData[5] = linePosition.y;
}

void printer::print() {
    for (uint8_t i = 0; i < 6; i++) {
        Serial.println(plotData[i]);
    }
    delay(100);

}