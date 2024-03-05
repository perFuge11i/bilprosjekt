#include "printer.hpp"

void printer::setCarPosition(point &carPosition) {
    plotData[0] = carPosition.x;
    plotData[1] = carPosition.y;
}

void printer::setCarTrajectory(point &carTrajectory) {
    plotData[2] = carTrajectory.x;
    plotData[3] = carTrajectory.y;
}

void printer::print() {
    for (uint8_t i = 0; i <= 6; i++)
        Serial.write((byte *)&plotData[i], sizeof(plotData[i]));
}