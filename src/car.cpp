#include "car.hpp"

car::car(uint8_t baseSpeed, motorPins& leftMotorPins, motorPins& rightMotorPins, carDimesions& dimesions)
        : leftMotor(leftMotorPins),
          rightMotor(rightMotorPins),
          sensorValSM(baseSpeed),
          odometryModel(dimesions.width, dimesions.length),
          carPositionVector(0, 0), carDirectionVector(0, 0),
          linePositionvector(0,0){
    lastTime = millis();

    lastLeftPulseCount = leftMotor.getPulses();
    lastRightPulseCount = rightMotor.getPulses();

    double wheelCircumfrence = M_PI*dimesions.wheelDiameter;
    travelPrPulse = wheelCircumfrence/dimesions.pulsesInRotation;


    //TODO: bare test
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
}

void car::run() {
    readSensors();
    updateTime();

    //sensorValSM.update(sensorState);

    //leftMotor.setSpeed(sensorValSM.getLeftSpeed());
    //rightMotor.setSpeed(sensorValSM.getRightSpeed());

    calculateTravel();
    odometryModel.calculate(leftTravel, rightTravel);
    odometryModel.calculateLine(sensorOffset);
    updatePosition();

    /*Serial.print(leftMotor.getPulses());
    Serial.print(" | ");
    Serial.print(rightMotor.getPulses());
    Serial.print(" | ");
    Serial.print(odometryModel.getDistanceTravelled().x);
    Serial.print(" | ");
    Serial.print(odometryModel.getDistanceTravelled().y);
    Serial.println();*/

    //saveToMemory(); TODO: fix vector
    dataPrinter.setCarPosition(carPosition);
    dataPrinter.setCarDirection(carDirection);
    dataPrinter.setLinePosition(linePosition);
    dataPrinter.print();
}

void car::updatePosition() {
    carPositionVector.add(odometryModel.getDistanceTravelled());
    carPosition.x = carPositionVector.x;
    carPosition.y = carPositionVector.y;

    linePositionvector = odometryModel.getLineDistance();
    linePositionvector.add(carPositionVector);
    linePosition.x = linePositionvector.x;
    linePosition.y = linePositionvector.y;

    carDirectionVector = odometryModel.getTrajectory();
    carDirection.x = carDirectionVector.x;
    carDirection.y = carDirectionVector.y;
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
        sensorOffset = Serial.read();
    }
}

void car::updateTime() {
    currentTime = millis();
    dt = currentTime - lastTime;
    lastTime = currentTime;
}

void car::saveToMemory() {
    //memory.storeToCar(carPosition); TODO: fix vector
}

encoder& car::getLeftEncoder() {
    return leftMotor.getEncoder();
}

encoder &car::getRightEncoder() {
    return rightMotor.getEncoder();
}

void car::resetMemory() {
    //memory.reset(); TODO: fix vector
}