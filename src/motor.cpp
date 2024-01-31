#include "motor.hpp"

motor::motor(int encoderPin, int pwmPin, double cKp, double cKi, double cKd) :
        encoderPin(pin), pulseCount(0), lastMeasurement(0),
        rpm(0), output(0), setPoint(0),
        pinMode(this->pwmPin, OUTPUT)
        consKp(cKp), consKi(cKi), consKd(cKd),
        pidController(&rpm, &output, &setPoint, consKp, consKi, consKd, DIRECT) {
    pinMode(encoderPin, INPUT);
    pidController.SetMode(AUTOMATIC);
}

void motor::updatePID() {
    double error = abs(setPoint - rpm);
    if (error < 10) {
        pidController.SetTunings(consKp, consKi, consKd);
    } else {
        pidController.SetTunings(aggKp, aggKi, aggKd);
    }
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

void motor::setSetPoint(double targetRPM) {
    setPoint = targetRPM;
}
