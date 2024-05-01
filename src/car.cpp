#include "car.hpp"

car::car(double baseSpeed, range& speedRange, motorPins& leftMotorPins, motorPins& rightMotorPins, carDimesions& dimensions, PIDparameters& angleKvalues, PIDparameters& speedKvalues)
        : leftMotor(leftMotorPins, speedRange), rightMotor(rightMotorPins, speedRange),
          odometryModel(dimensions.width, dimensions.length), dataPrinter(),
          anglePID(angleKvalues.kP, angleKvalues.kI, angleKvalues.kD, angleKvalues.windup),
          speedPID(speedKvalues.kP, speedKvalues.kI, speedKvalues.kD, speedKvalues.windup),
          carPositionVector(0.0, 0.0), carDirectionVector(0.0, 1.0), linePositionvector(0.0,dimensions.length),
          refLength(7.5, 0.0), refVector(0, 7.5),
          currentIndex(0){
    baseSpd = baseSpeed;
    regulatedSpd = baseSpd;


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
    bufferStartTime = mimimillis();
    leftPulseTime = 51;
    rightPulseTime = 51;
    startTimer = 1000;
    timer90deg = 200;
    waitTimer90deg = 100;
    PIDtimer = 100;
    printTimer = 50;
    bufferTimer = 100;
    counts = 0;
    ignore = false;
    lCnt = 0;
    rCnt = 0;
    cntStartTime = mimimillis();
    cntTimer = 300;
    lineDir.x = 0.0;
    lineDir.y = 1.0;
    leftWaitStartTime = 0;
    rightWaitStartTime = 0;
    nineTR = false;
    nineTL = false;
    curveThreshold = 0.12;
    curveAngle = 0;

    bufferIsFull = false;
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

    if (lineDetected && (abs(reading) != 37)) {
        if ((currentTime >= bufferStartTime + bufferTimer)) {
            updateLinePositions(linePosition);
            curveAlgorithm();
            bufferStartTime = mimimillis();
        }
    }

    calculateAngle();
    setMotorSpeeds();

    geogebraPrint();
}

void car::setMotorSpeeds() {

    if (reading == (-37)) {
        leftWaitStartTime = mimimillis();
        nineTL = true;
        nineTR = false;
    } else if (reading == 37) {
        leftWaitStartTime = mimimillis();
        nineTR = true;
        nineTL = false;
    }

    if (lineDetected && (currentTime >= leftWaitStartTime + waitTimer90deg)) {
        nineTR = false;
        nineTL = false;
    }

    if (!lineDetected && nineTR) {
        leftMotorSpeed = 0;
        rightMotorSpeed = -baseSpd;
        if (lineDetected) {
            nineTR = false;
            nineTL = false;
        }
    } else if (!lineDetected && nineTL) {
        leftMotorSpeed = -baseSpd;
        rightMotorSpeed = 0;
        if (lineDetected) {
            nineTR = false;
            nineTL = false;
        }
    } else if (currentTime > startTime90deg + timer90deg) {
        correction = anglePID.regulate(dt, 0, angleToLine);
        baseCorrection = speedPID.regulate(dt, 0, abs(curveError));

        regulatedSpd = baseSpd*(baseCorrection+1);

        if (correction < 0) {
            leftMotorSpeed = constrain(regulatedSpd + correction, -0.3, 1);
            rightMotorSpeed = regulatedSpd;
        } else {
            leftMotorSpeed = regulatedSpd;
            rightMotorSpeed = constrain(regulatedSpd - correction, -0.3, 1);
        }
        /*if (checkCurve()) {
            baseSpd = 0.6;
            if (correction < 0) {
                leftMotorSpeed = constrain(baseSpd + correction, -0.3, 1);
                rightMotorSpeed = baseSpd;
            } else {
                baseSpd = 0.6;
                leftMotorSpeed = baseSpd;
                rightMotorSpeed = constrain(baseSpd - correction, -0.3, 1);
            }
        } else { }*/
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
}

void car::geogebraPrint() {
    if (currentTime >= lastPrintTime + printTimer) {
        dataPrinter.writeCarPosition(carPosition);
        dataPrinter.writeCarFront(curveAngle);
        dataPrinter.writeLinePosition(linePosition);
        dataPrinter.writeCurveError(curveError);

        if (dataPrinter.isFull()) {
            leftMotor.stop();
            rightMotor.stop();
            dataPrinter.closeDocuments();
            stop = true;
        }

        lastPrintTime = currentTime;
    }
}

void car::updateLinePositions(point newPosition) {
    if (currentIndex >= line_pos_size-1) {
        bufferIsFull = true;
        for (int8_t i = 0; i < line_pos_size-1; i++) {
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

void car::curveAlgorithm() {
    if (!bufferIsFull) {
        return;
    }
    curveError = 0;

    vektorAB[0] = linePositions[line_pos_size-1][0]-linePositions[0][0];
    vektorAB[1] = linePositions[line_pos_size-1][1]-linePositions[0][1];

    for (int i = 1; i < line_pos_size-1; i++) {
        vektorAP[0] = linePositions[i][0]-linePositions[0][0];
        vektorAP[1] = linePositions[i][1]-linePositions[0][1];

        curveError += ((vektorAB[0]*vektorAP[1]-vektorAP[0]*vektorAB[1])/
                       (sqrt(vektorAB[0]*vektorAB[0]+vektorAB[1]*vektorAB[1])));
    }
}

void car::lineRegression() {
    if (!bufferIsFull) {
        return;
    }

    lastLineDir = lineDir;
    lastCurveAngle = curveAngle;

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

    if (denominator != 0) {
        slope = numerator/denominator;
        lineDir.x = 1/(sqrt(1+slope*slope));
        lineDir.y = slope/(sqrt(1+slope*slope));
        curveAngle = atan2(lineDir.x*lastLineDir.y-lineDir.y*lastLineDir.x, lineDir.x*lastLineDir.x + lineDir.y*lastLineDir.y);
    }
}

bool car::checkCurve() {
    return abs(curveAngle) > curveThreshold;
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



// pio device monitor --port /dev/cu.usbserial-10 --baud 9600 */