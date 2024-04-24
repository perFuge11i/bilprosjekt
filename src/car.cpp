#include "car.hpp"

car::car(double baseSpeed, range& speedRange, motorPins& leftMotorPins, motorPins& rightMotorPins, carDimesions& dimensions, PIDparameters& kValues)
        : leftMotor(leftMotorPins, speedRange), rightMotor(rightMotorPins, speedRange),
          odometryModel(dimensions.width, dimensions.length),
          dataPrinter(), carPositionVector(0.0, 0.0),
          carDirectionVector(0.0, 1.0), linePositionvector(0.0,dimensions.length),
          currentIndex(0), anglePID(kValues.kP, kValues.kI, kValues.kD, kValues.windup) {
    lastTime = millis();

    lastLeftPulseCount = leftMotor.getPulses();
    lastRightPulseCount = rightMotor.getPulses();
    travelPrPulse = M_PI*dimensions.wheelDiameter/dimensions.pulsesInRotation;
    baseSpd = baseSpeed;
    minSpd = speedRange.min;
    maxSpd = speedRange.max;

    linePosition.x = 0.0;
    linePosition.y = dimensions.length;
    sensorOffset = 44;
    angleToLine = 0.0;
    lineDir.x = 0.0;
    lineDir.y = 1.0;
    carDirection.x = 0.0;
    carDirection.y = 0.0;

    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);

    timer = 1000;
    started = false;
    startTime = millis();
    lTime = millis();
    lastPulsesLeft = 0;
    lastPulsesRight = 0;
    printTimer = 20;
    PIDtimer =100;
    stop = false;
    lCnt = 0;
    rCnt = 0;
    integrating = false;
}

bool car::stopp() {
    return stop;
}

void car::run() {
    readSensors();
    updateTime();

    if (!started) {
        if (currentTime < startTime + timer) { return; }
        else { started = true; }
    }
    if (!integrating) {
        if (currentTime >= startTime + timer + PIDtimer) {
            anglePID.activateI();
            integrating = true;
        }
    }

    if (lCnt >= 5 || rCnt >= 5) {
        odometryModel.calculate(leftTravel, rightTravel);
        odometryModel.calculateLine(sensorOffset);
        updatePosition();
        calculateAngle();
        leftTravel = 0;
        rightTravel = 0;
        lCnt = 0;
        rCnt = 0;
    }

    setMotorSpeeds();


    /*
    if (currentTime >= lTime + printTimer) {
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
        lTime = currentTime;
    }*/

}

void car::readSensors() {
    if (Serial.available() > 0) {
        while (Serial.available() > 0) {
            readings = Serial.read();
        }
        if (readings == 44) {
            sensorOffset = readings;
        } else {
            double dbReadings = double(readings);
            sensorOffset = -dbReadings/10;
        }
    }
}


void car::setMotorSpeeds() {
    if (currentTime < startTime + timer) {
        return;
    } if (readings == -37) {
        leftMotor.setSpeed(baseSpd);
        rightMotor.setSpeed(0);
        startTime = currentTime;
        timer = 500;
    } else if (readings == 37) {
        rightMotor.setSpeed(baseSpd);
        leftMotor.setSpeed(0);
        startTime = currentTime;
        timer = 500;
    } else {
        double correction = anglePID.regulate(dt, 0, angleToLine);

        if (correction < 0) {
            leftMotorSpeed = constrain(baseSpd + correction, 0, 1);
            rightMotorSpeed = baseSpd;
        } else {
            leftMotorSpeed = baseSpd;
            rightMotorSpeed = constrain(baseSpd - correction, 0, 1);
        }

        leftMotor.setSpeed(leftMotorSpeed);
        rightMotor.setSpeed(rightMotorSpeed);
    }
}

void car::calculateAngle() {
    carReferancePoint.x = carPosition.x + carDirection.x*7.15;
    carReferancePoint.y = carPosition.y + carDirection.y*7.15;

    carToLine.x = linePosition.x-carReferancePoint.x;
    carToLine.y = linePosition.y-carReferancePoint.y;

    if (sensorOffset == 44) {
        lineLost = true;
    } else {
        lineLost = false;
    }

    angleToLine = atan2(carDirection.x*carToLine.y-carDirection.y*carToLine.x, carDirection.x*carToLine.x + carDirection.y*carToLine.y);

    if (lineLost) {
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

encoder& car::getLeftEncoder() {
    return leftMotor.getEncoder();
}

encoder &car::getRightEncoder() {
    return rightMotor.getEncoder();
}


void car::updatePosition() {
    carPositionVector.add(odometryModel.getDistanceTravelled());
    carPosition.x = carPositionVector.x;
    carPosition.y = carPositionVector.y;

    linePositionvector = odometryModel.getLineDistance();
    if (linePositionvector.x != 44) {
        linePositionvector.add(carPositionVector);
        linePosition.x = linePositionvector.x;
        linePosition.y = linePositionvector.y;
    }

    directionAngle = odometryModel.getDirAngle();

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

void car::lineRegression() {
    if (!isFull) {
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

void car::updateTime() {
    currentTime = double(micros())/1000;
    dt = currentTime - lastTime;
    lastTime = currentTime;
}

void car::travel1Left() {
    leftTravel += travelPrPulse;
    lCnt += 1;
}

void car::travel1Right() {
    rightTravel += travelPrPulse;
    rCnt += 1;
}
void car::speedTest() {
    updateTime();


    if (currentTime < startTime + timer || stop) {
        return;
    } else if (currentTime < startTime + 2*timer) {
        baseSpd = 30;
    } else if (currentTime < startTime + 3*timer) {
        baseSpd = 40;
    } else if (currentTime < startTime + 3*timer) {
        baseSpd = 50;
    } else if (currentTime < startTime + 4*timer) {
        baseSpd = 70;
    } else if (currentTime < startTime + 5*timer) {
        baseSpd = 90;
    } else if (currentTime < startTime + 6*timer) {
        baseSpd = 120;
    } else if (currentTime < startTime + 7*timer) {
        stop = true;
    }

    rightMotor.setSpeed(baseSpd);
    leftMotor.setSpeed(baseSpd);


    if (currentTime >= lTime + 100) {
        double deltaLeft = (leftMotor.getPulses() - lastPulsesLeft);
        double deltaRight = (rightMotor.getPulses() - lastPulsesRight);
        Serial.print("(");
        Serial.print(baseSpd);
        Serial.print(", ");
        Serial.print(deltaLeft/deltaRight);
        Serial.print("), ");
        lTime = currentTime;
        lastPulsesLeft = leftMotor.getPulses();
        lastPulsesRight = rightMotor.getPulses();
    }

}
// pio device monitor --port /dev/cu.usbserial-10 --baud 9600