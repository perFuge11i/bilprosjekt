#include "motor.hpp"

motor::motor(motorPins& pins, PIDparameters& kValues) :
        pidController(&rpm, &output, &setPoint, kValues.kP, kValues.kI, kValues.kD, DIRECT) {

    consKp(cKp), consKi(cKi), consKd(cKd),
    pinMode(encoderPin, INPUT);
    pidController.SetMode(AUTOMATIC);
}g

void motor::updatePID() {
    double error = rpm;
    pidController.Compute();
}


void motor::calculateRPM() {
    unsigned long currentTime = millis();
    unsigned long timeDifference = currentTime - lastMeasurement;

    if (timeDifference >= 1000) {
        rpm = (pulseCount / 12.0) * (60000.0 / timeDifference);
        pulseCount = 0;
        lastMeasurement = currentTime;
    }
}

void motor::update() {
    long encoderCount = encoder.getCount();
    calculateRPM();
    updatePID();
    // TODO: Output to motor driver
}

double motor::getRPM() const {
    return rpm;
}

void motor::setRPM(double targetRPM) {
    setPoint = targetRPM;
}
