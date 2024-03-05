#include "car.hpp"

car::car(uint8_t baseSpeed, motorPins& leftMotorPins, motorPins& rightMotorPins, carDimesions& dimesions)
        : leftMotor(leftMotorPins),
          rightMotor(rightMotorPins),
          sensorValSM(baseSpeed),
          odometryModel(dimesions.width),
          carPositionVector(0, 0) {
    lastTime = millis();

    lastLeftPulseCount = leftMotor.getPulses();
    lastRightPulseCount = rightMotor.getPulses();

    double wheelCircumfrence = M_PI*dimesions.wheelDiameter;
    travelPrPulse = wheelCircumfrence/dimesions.pulsesInRotation;
}

void car::run() {
    readSensors();
    updateTime();

    sensorValSM.update(sensorState);

    leftMotor.setSpeed(sensorValSM.getLeftSpeed());
    rightMotor.setSpeed(sensorValSM.getRightSpeed());

    calculateTravel();
    odometryModel.calculate(leftTravel, rightTravel);
    updateCarPosition();

    saveToMemory();
    dataPrinter.setCarPosition(carPosition);
    dataPrinter.setCarTrajectory(odometryModel.getTrajectory().getStructure());
    dataPrinter.print();
}

void car::updateCarPosition() {
    carPositionVector.add(odometryModel.getDistanceTravelled());
    carPosition.x = carPositionVector.x;
    carPosition.y = carPositionVector.y;
}

void car::calculateTravel() {
    uint8_t dPulseLeft = leftMotor.getPulses() - lastLeftPulseCount;
    uint8_t dPulseRight = rightMotor.getPulses() - lastRightPulseCount;

    leftTravel = dPulseLeft*travelPrPulse;
    rightTravel = dPulseRight*travelPrPulse;

    lastLeftPulseCount = leftMotor.getPulses();
    lastRightPulseCount = rightMotor.getPulses();
}

void car::readSensors() {
    if (Serial.available() > 0) {
        sensorState = Serial.read();
    }
}

void car::updateTime() {
    currentTime = millis();
    dt = currentTime - lastTime;
    lastTime = currentTime;
}

void car::saveToMemory() {
    memory.storeToCar(carPosition);
}

encoder& car::getLeftEncoder() {
    return leftMotor.getEncoder();
}

encoder &car::getRightEncoder() {
    return rightMotor.getEncoder();
}

void car::resetMemory() {
    memory.reset();
}