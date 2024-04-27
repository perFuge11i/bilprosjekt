#include "car.hpp"

car::car(double baseSpeed, range& speedRange, motorPins& leftMotorPins, motorPins& rightMotorPins, carDimesions& dimensions, PIDparameters& kValues)
        : leftMotor(leftMotorPins, speedRange), rightMotor(rightMotorPins, speedRange),
          odometryModel(dimensions.width, dimensions.length), dataPrinter(),
          carPositionVector(0.0, 0.0), carDirectionVector(0.0, 1.0),
          linePositionvector(0.0,dimensions.length), refLength(7.5, 0.0), refVector(0, 7.5),
          currentIndex(0), anglePID(kValues.kP, kValues.kI, kValues.kD, kValues.windup) {
    baseSpd = baseSpeed;

    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);

    linePosition.x = 0.0;
    linePosition.y = dimensions.length;
    carDirection.x = 0.0;
    carDirection.y = 1.0;
    angleToLine = 0.0;
    lastReading = 55;
    reading = 0;

    started = false;
    stop = false;
    integrating = false;
    lineDetected = true;

    travelPrPulse = M_PI*dimensions.wheelDiameter/dimensions.pulsesInRotation;
    pulseTimeThreshold = 50;

    lastTime = mimimillis();
    leftLastPulseTime = mimimillis();
    rightLastPulseTime = mimimillis();
    lastPrintTime = mimimillis();
    startTime90deg = mimimillis();
    leftPulseTime = 51;
    rightPulseTime = 51;
    startTimer = 1000;
    timer90deg = 100;
    PIDtimer =100;
    printTimer = 10;

    counts = 0;
    ignore = false;
    lCnt = 0;
    rCnt = 0;
    cntStartTime = mimimillis();
    cntTimer = 0;
    lineDir.x = 0.0;
    lineDir.y = 1.0;
}


bool car::stopp() const {
    return stop;
}

void car::run() {
    readSensors();
    updateTime();

    if (!started) {
        if (currentTime < startTime + startTimer) { return; }
        else { started = true; }
    }
    if (!integrating) {
        if (currentTime >= startTime + startTimer + PIDtimer) {
            anglePID.activateI();
            integrating = true;
        }
    }

    checkEncoderUpdates();
    calculateTravel();

    odometryModel.calculate(leftTravel, rightTravel);
    if (lineDetected) {
        odometryModel.calculateLine(sensorOffset);
    }
    updatePosition();
    calculateAngle();

    setMotorSpeeds();

    geogebraPrint();
}

void car::checkEncoderUpdates() {
    timeSinceLeftUpdate = currentTime - leftLastPulseTime;
    timeSinceRightUpdate = currentTime - rightLastPulseTime;

    if (leftPulseTime < timeSinceLeftUpdate) {
        leftPulseTime = timeSinceLeftUpdate;
    } else {
        leftUpdated = false;
    }
    if (rightPulseTime < timeSinceRightUpdate) {
        rightPulseTime = timeSinceRightUpdate;
    } else {
        rightUpdated = false;
    }
}

void car::calculateTravel() {
    if (leftPulseTime > pulseTimeThreshold) {
        leftTravel = 0;
    } else {
        leftTravel = travelPrPulse/leftPulseTime*dt;
    }

    if (rightPulseTime > pulseTimeThreshold) {
        rightTravel = 0;
    } else {
        rightTravel = travelPrPulse/rightPulseTime*dt;
    }
}

void car::updateLeftRPM() {
    leftPulseTime = mimimillis() - leftLastPulseTime;
    leftLastPulseTime = mimimillis();
    leftUpdated = true;
}
void car::updateRightRPM() {
    rightPulseTime = mimimillis() - rightLastPulseTime;
    rightLastPulseTime = mimimillis();
    rightUpdated = true;
}

void car::setMotorSpeeds() {
    if (reading == -37) {
        leftMotorSpeed = baseSpd;
        rightMotorSpeed = -baseSpd;
        startTime90deg = mimimillis();
    } else if (reading == 37) {
        leftMotorSpeed = -baseSpd;
        rightMotorSpeed = baseSpd;
        startTime90deg = mimimillis();
    } else if (currentTime > startTime90deg + timer90deg) {
        correction = anglePID.regulate(dt, 0, angleToLine);
        //Serial.println(angleToLine);
        if (correction < 0) {
            leftMotorSpeed = constrain(baseSpd + correction, -0.2, 1);
            rightMotorSpeed = baseSpd;
        } else {
            leftMotorSpeed = baseSpd;
            rightMotorSpeed = constrain(baseSpd - correction, -0.2, 1);

        }
    }

    leftMotor.setSpeed(leftMotorSpeed);
    rightMotor.setSpeed(rightMotorSpeed);
}

void car::calculateAngle() {
    refVector = refLength.rotate(directionAngle);
    carReferancePoint.x = carPosition.x + refVector.x;
    carReferancePoint.y = carPosition.y + refVector.y;

    carToLine.x = linePosition.x-carReferancePoint.x;
    carToLine.y = linePosition.y-carReferancePoint.y;

    angleToLine = atan2(carDirection.x*carToLine.y-carDirection.y*carToLine.x, carDirection.x*carToLine.x + carDirection.y*carToLine.y);

    if (!lineDetected) {
        if (lastAngleDir == 1 && angleToLine < 0) {
            angleToLine += 2*M_PI;
        } else if (lastAngleDir == -1 && angleToLine > 0) {
            angleToLine -= 2*M_PI;
        }
    } else {
        if (angleToLine >= 0) {
            lastAngleDir = 1;
        } else {
            lastAngleDir = -1;
        }
    }
}

void car::updatePosition() {
    carPositionVector.add(odometryModel.getDistanceTravelled());
    carPosition.x = carPositionVector.x;
    carPosition.y = carPositionVector.y;

    if (lineDetected) {
        linePositionvector = odometryModel.getLineDistance();
        linePositionvector.add(carPositionVector);
        linePosition.x = linePositionvector.x;
        linePosition.y = linePositionvector.y;
    }

    directionAngle = odometryModel.getDirAngle();

    carDirectionVector = odometryModel.getTrajectory();
    carDirection.x = carDirectionVector.x;
    carDirection.y = carDirectionVector.y;
}

void car::readSensors() {
    lastReading = reading;
    if (Serial.available() > 0) {
        while (Serial.available() > 0) {
            reading = int8_t(Serial.read());
        }
        if (reading == 44) {
            lineDetected = false;
        } else {
            lineDetected = true;
            sensorOffset = double(reading)/10;
        }
    }
    //Serial.println(sensorOffset);
    //Serial.println(reading);
}

void car::geogebraPrint() {
    if (currentTime >= lastPrintTime + printTimer) {
        dataPrinter.setCarPosition(carPosition);
        dataPrinter.setCarAngle(directionAngle);
        dataPrinter.setLinePosition(linePosition);
        if (dataPrinter.isFull()) {
            leftMotor.stop();
            rightMotor.stop();
            stop = true;
            Serial.print(leftMotor.getPulses());
            Serial.print(" | ");
            Serial.print(rightMotor.getPulses());
            Serial.print(" | ");
            dataPrinter.printPointList();
        }
        lastPrintTime = currentTime;
    }
}
double car::mimimillis() {
    return double(micros())/1000;
}

void car::updateTime() {
    currentTime = mimimillis();
    dt = currentTime - lastTime;
    lastTime = currentTime;
}

encoder& car::getLeftEncoder() {
    return leftMotor.getEncoder();
}

encoder &car::getRightEncoder() {
    return rightMotor.getEncoder();
}

void car::travel1Left() {
    leftTravel += travelPrPulse;
    lCnt += 1;
}
void car::travel1Right() {
    rightTravel += travelPrPulse;
    rCnt += 1;
}

void car::counter() {
    if (abs(reading) == 37 && abs(lastReading) != 37) {
        if (currentTime > cntStartTime + cntTimer) {
            cntTimer = 300;
            cntStartTime = double(micros())/1000;

            Serial.print("dffdddf");
            if (counts == 0) {
                counts++;
                ignore = false;
            } else if (counts == 1) {
                counts++;
                ignore = true;
            } else if (counts == 2) {
                counts++;
                ignore = true;
            } else if (counts == 3) {
                counts++;
                ignore = false;
            } else if (counts == 4) {
                counts++;
                ignore = true;
            } else if (counts == 5) {
                counts++;
                ignore = true;
            } else if (counts == 6) {
                counts++;
                ignore = false;
            } else if (counts == 7) {
                ignore = false;
                counts = 0;
            }
        }
    }
}

void car::updateLinePositions(point newPosition) {
    if (currentIndex >= line_pos_size-1) {
        printerIsFull = true;
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

void car::lineRegression() {
    if (!printerIsFull) {
        return;
    }

    double sumX = 0, sumY = 0;

    for (int8_t i = 0; i < line_pos_size; i++) {
        sumX += linePositions[i][0];
        sumY += linePositions[i][1];
    }

    double meanX = sumX / line_pos_size;
    double meanY = sumY / line_pos_size;
    double numerator = 0;
    double denominator = 0;
    for (int8_t i = 0; i < line_pos_size; i++) {
        numerator += (linePositions[i][0] - meanX)*(linePositions[i][1] - meanY);
        denominator += (linePositions[i][0] - meanX)*(linePositions[i][0] - meanX);

    }
    slope = numerator/denominator;

    lineDir.x = 1/(sqrt(1+slope*slope));
    lineDir.y = slope/(sqrt(1+slope*slope));
}

// pio device monitor --port /dev/cu.usbserial-10 --baud 9600