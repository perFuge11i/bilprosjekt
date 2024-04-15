#include "car.hpp"

car::car(uint8_t baseSpeed, motorPins& leftMotorPins, motorPins& rightMotorPins, carDimesions& dimesions, PIDparameters& kValues)
        : leftMotor(leftMotorPins),
          rightMotor(rightMotorPins),
          sensorValSM(baseSpeed),
          odometryModel(dimesions.width, dimesions.length),
          carPositionVector(0, 0), carDirectionVector(0, 0),
          linePositionvector(0,0), anglePID(kValues.kP, kValues.kI, kValues.kD),
          currentIndex(0) {
    lastTime = millis();

    lastLeftPulseCount = leftMotor.getPulses();
    lastRightPulseCount = rightMotor.getPulses();

    double wheelCircumfrence = M_PI*dimesions.wheelDiameter;
    travelPrPulse = wheelCircumfrence/dimesions.pulsesInRotation;
    baseSpd = baseSpeed;

    linePosition.x = 0;
    linePosition.y = 0;
    sensorOffset = 111;
    angleToLine = 0;

    //TODO: bare test
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);

}

void car::run() {
    readSensors();
    updateTime();

    calculateTravel();
    odometryModel.calculate(leftTravel, rightTravel);
    odometryModel.calculateLine(sensorOffset);
    updatePosition();

    if (sensorOffset == 111) {
        lineLost = true;
    } else {
        lineLost = false;
        carReferancePoint.x = carPosition.x + carDirection.x * 8;
        carReferancePoint.y = carPosition.y + carDirection.y * 8;

        carToLine.x = linePosition.x-carReferancePoint.x;
        carToLine.y = linePosition.y-carReferancePoint.y;

    angleToLine = atan2(carDirection.x*carToLine.y-carDirection.y*carToLine.x, carDirection.x*carToLine.x + carDirection.y*carToLine.y);

        if (angleToLine >= 0) {
            lastAngleDir = 1;
        } else {
            lastAngleDir = -1;
        }
    }

    if (lineLost) {
        if (lastAngleDir == 1) {
            angleToLine = (atan2(carDirection.x*carToLine.y-carDirection.y*carToLine.x, carDirection.x*carToLine.x + carDirection.y*carToLine.y));
            if (angleToLine < 0) {
                angleToLine = (2*M_PI + atan2(carDirection.x*carToLine.y-carDirection.y*carToLine.x, carDirection.x*carToLine.x + carDirection.y*carToLine.y));
            }
        } else if (lastAngleDir == -1) {
            angleToLine = (2*M_PI + atan2(carDirection.x*carToLine.y-carDirection.y*carToLine.x, carDirection.x*carToLine.x + carDirection.y*carToLine.y));
            if (angleToLine > 0) {
                angleToLine = (-2*M_PI + atan2(carDirection.x*carToLine.y-carDirection.y*carToLine.x, carDirection.x*carToLine.x + carDirection.y*carToLine.y));
            }
        }
    }


    setMotorSpeeds();

    //saveToMemory(); TODO: fix vector
    dataPrinter.setCarPosition(carPosition);
    dataPrinter.setCarDirection(carDirection);
    dataPrinter.setLinePosition(linePosition);

    //dataPrinter.print();
}

void car::updatePosition() {
    carPositionVector.add(odometryModel.getDistanceTravelled());
    carPosition.x = carPositionVector.x;
    carPosition.y = carPositionVector.y;

    linePositionvector = odometryModel.getLineDistance();
    if (linePositionvector.x != 111) {
        linePositionvector.add(carPositionVector);
        linePosition.x = linePositionvector.x;
        linePosition.y = linePositionvector.y;
    }

    updateLinePositions(linePositionvector);

    carDirectionVector = odometryModel.getTrajectory();
    carDirection.x = carDirectionVector.x;
    carDirection.y = carDirectionVector.y;
}

void updateLinePositions(point newPosition) {
    
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
        readings = Serial.read();
    }

    if (readings == 111) {
        sensorOffset = readings;
    } else if (readings == 40) {
        sensorOffset = readings;
    } else if (readings == -40) {
        sensorOffset = readings;
    } else {
        double dbReadings = double(readings);
        sensorOffset = dbReadings/10;
    }

}


void car::updateTime() {
    currentTime = millis();
    dt = currentTime - lastTime;
    lastTime = currentTime;
}

void car::setMotorSpeeds() {
    if (currentTime < startTime + timer) {
        return;
    }

    float targetAngle = 0;


    if (sensorOffset == 40) {

        leftMotor.setSpeed(baseSpd);
        rightMotor.setSpeed(0);
        startTime = currentTime;
        timer = 500;



    } else if (sensorOffset == -40) {

        rightMotor.setSpeed(baseSpd);
        leftMotor.setSpeed(0);
        startTime = currentTime;
        timer = 500;


    } else {

        float correction = anglePID.regulate(dt, targetAngle, angleToLine);

        float leftMotorSpeed = baseSpd + baseSpd*correction;
        float rightMotorSpeed = baseSpd - baseSpd*correction;

        leftMotorSpeed = constrain(leftMotorSpeed, 0, baseSpd);
        rightMotorSpeed = constrain(rightMotorSpeed, 0, baseSpd);


        if (correction < 0) {
            leftMotor.setSpeed(leftMotorSpeed);
            rightMotor.setSpeed(baseSpd);
        } else if (correction >= 0) {
            leftMotor.setSpeed(baseSpd);
            rightMotor.setSpeed(rightMotorSpeed);
        }
    }
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

