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
    Serial.write(plotData);
}