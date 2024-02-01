#include "simpleCar.hpp"

simpleCar::simpleCar(double baseRPM_, double Kp, double Ki, double Kd, double setPoint) : leftMotor(),
                                                                                          rightMotor(),
                                                                                          simplePID(&lineReading, &correction, &setPoint, Kp, Ki, Kd, DIRECT) {
    setPoint = 0;
    simplePID.SetMode(AUTOMATIC);
    baseRPM = baseRPM_;
}

void simpleCar::update() {
    //lineReading = 0;//Les fra Arduino 2
    lineReading = readLineSensors();

    if (lineReading != -1) {
        simplePID.Compute();
        RPMcorrection = calculateRPMcorrection(correction);
    }


    leftMotor.setRPM(baseRPM - RPMcorrection);
    rightMotor.setRPM(baseRPM + RPMcorrection);

    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastTime;
    lastTime = currentTime;

    saveToMemory();
}

double simpleCar::calculateRPMcorrection(double correction) {
    //Constrain correction
    double maxCorrection = 1/2*baseRPM;
    return constrain(correction, -maxCorrection, maxCorrection);
}

double simpleCar::readLineSensors() {
    int values[15];
    int position = 0;
    int total = 0;
    int threshold = 500; //eksempel
    for (int i = 0; i < 15; i++) {
        int value = analogRead(i);
        if (value > threshold) {
            values[i] = value;
            position += value * (i + 1);
            total += value;
        } else {
            values[i] = 0;
        }
    }
    if (total == 0) return -1; // Ingen linje

    double scaledPosition = ((double)position / total) / 15 - 1;
    return scaledPosition;
}


void simpleCar::saveToMemory() const {
    long leftEncCount = leftEncoder.getCount();
    long rightEncCount = rightEncoder.getCount();
    double leftMotorRPM = leftMotor.getRPM();
    double rightMotorRPM = rightMotor.getRPM();

    memory.storePoint(leftMotorRPM, rightMotorRPM, leftEncCount, rightEncCount); //sender verdiene til baneminne.hpp - lagrer videre i cpp
}
