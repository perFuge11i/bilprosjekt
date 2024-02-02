#include "simpleCar.hpp"

simpleCar::simpleCar(double baseSpeed_, PIDparameters& kValues, motorPins& leftMotorPins, motorPins& rightMotorPins) :
                     leftMotor(leftMotorPins),
                     rightMotor(leftMotorPins),
                     simplePID(&lineReading, &correction, 0, kValues.kP, kValues.kI, kValues.kP, DIRECT) {
    simplePID.SetMode(AUTOMATIC);
    baseSpeed = baseSpeed_;
}

void simpleCar::update() {
    //Setter linje avlesning
    lineReading = 0;// TODO: Les fra Arduino 2

    //Hvis linjen detekteres
    if (lineReading != -1) {
        //Kjør pid og sett motorforhold
        simplePID.Compute();
        speedCorrection = calculateSpeedCorrection(correction);
    }

    //Set motor til respektive hastiheter
    leftMotor.setSpeed(baseSpeed - speedCorrection);
    rightMotor.setSpeed(baseSpeed + speedCorrection);

    saveToMemory();
    // TODO: Legg til runde 2
}

void simpleCar::saveToMemory() const {
    unsigned long leftPulseCount = leftMotor.getPulses();
    unsigned long rightPulseCount = rightMotor.getPulses();
    unsigned long currentTime = millis() - startTime;

    memory.storePoint(leftPulseCount, rightPulseCount, currentTime);
}

double simpleCar::calculateSpeedCorrection(double correction) {
    //Constrain correction
    double maxCorrection = 1/2*baseSpeed;
    return constrain(correction, -maxCorrection, maxCorrection);
}