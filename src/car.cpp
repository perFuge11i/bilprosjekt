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
    lineDir.x = 0;
    lineDir.y = 1;
    carDirection.x = 0;
    carDirection.y = 0;


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

    lineRegression();
    Serial.print(linePositions[0][0]);
    Serial.print(" ");
    Serial.print(linePositions[0][1]);
    Serial.print(" , ");
    Serial.print(linePositions[1][0]);
    Serial.print(" ");
    Serial.print(linePositions[1][1]);
    Serial.print(" , ");
    Serial.print(linePositions[2][0]);
    Serial.print(" ");
    Serial.print(linePositions[2][1]);
    Serial.print(" , ");
    Serial.print(linePositions[3][0]);
    Serial.print(" ");
    Serial.print(linePositions[3][1]);
    Serial.print(" , ");
    Serial.print(linePositions[4][0]);
    Serial.print(" ");
    Serial.print(linePositions[4][1]);
    Serial.print(" , ");
    Serial.print(linePositions[5][0]);
    Serial.print(" ");
    Serial.print(linePositions[5][1]);
    Serial.print(" , ");
    Serial.print(lineDir.x);
    Serial.print(" ");
    Serial.print(lineDir.y);
    Serial.println();

    //setMotorSpeeds();

    //saveToMemory(); TODO: fix vector
    dataPrinter.setCarPosition(carPosition);
    dataPrinter.setCarDirection(carDirection);
    dataPrinter.setLinePosition(linePosition);
    dataPrinter.setLineDirection(lineDir);

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

    updateLinePositions(linePosition);

    carDirectionVector = odometryModel.getTrajectory();
    carDirection.x = carDirectionVector.x;
    carDirection.y = carDirectionVector.y;
}

void car::updateLinePositions(point newPosition) {
    if (currentIndex >= line_pos_size-1) {
        isFull = true;
        for (int8_t i = 0; i < line_pos_size; i++) {
            linePositions[i][0] = linePositions[i + 1][0];
            linePositions[i][1] = linePositions[i + 1][1];
        }
        linePositions[line_pos_size-1][0] = newPosition.x;
        linePositions[line_pos_size-1][1] = newPosition.y;
    } else {
        linePositions[currentIndex][0] = newPosition.x;
        linePositions[currentIndex][1] = newPosition.y;
        currentIndex++;
    }
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
        sensorOffset = dbReadings/100;
    }

}

void car::lineRegression() {
    if (!isFull) {
        return;
    }

    double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;

    for (int8_t i = 0; i < line_pos_size; i++) {
        sumX += linePositions[i][0];
        sumY += linePositions[i][1];
        sumXY += linePositions[i][0] * linePositions[i][1];
        sumXX += linePositions[i][0] * linePositions[i][0];
    }

    double meanX = sumX / line_pos_size;
    double meanY = sumY / line_pos_size;
    double numerator = 0;
    double denominator = 0;
    for (int8_t i = 0; i < line_pos_size; i++) {
        numerator += (linePositions[i][0] - meanX)*(linePositions[i][1] - meanY);
        denominator += (linePositions[i][0] - meanX)*(linePositions[i][0] - meanX));

    }
    slope = numerator/denominator;
    intercept = meanY - slope*meanX;


    lineDir.x = 1/(sqrt(1+slope*slope));
    lineDir.y = slope/(sqrt(1+slope*slope));
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

