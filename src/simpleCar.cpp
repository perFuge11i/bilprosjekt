#include "simpleCar.hpp"

simpleCar::simpleCar(double baseRPM_,
                     PIDparameters& kValues, PIDparameters& motorKvalues,
                     motorPins& leftMotorPins, motorPins& rightMotorPins) :
                     leftMotor(leftMotorPins, motorKvalues),
                     rightMotor(rightMotorPins, motorKvalues),
                     simplePID(&lineReading, &correction, 0, kValues.kP, kValues.kI, kValues.kD, DIRECT) {
    simplePID.SetMode(AUTOMATIC);
    baseRPM = baseRPM_;
}

void simpleCar::update() {
    lineReading = 0;//Les fra Arduino 2

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


void simpleCar::saveToMemory() const {
    long leftEncCount = leftMotor.getCount();
    long rightEncCount = rightMotor.getCount();
    double leftMotorRPM = leftMotor.getRPM();
    double rightMotorRPM = rightMotor.getRPM();

    memory.storePoint(leftMotorRPM, rightMotorRPM, leftEncCount, rightEncCount); //sender verdiene til baneminne.hpp - lagrer videre i cpp
}
