#include "motor.hpp"

motor::motor(motorPins& pins, range& speedRange) : motorEncoder(pins.encoderPin) {
    PMWpin = pins.PMWpin;
    dirPin1 = pins.dirPin1;
    dirPin2 = pins.dirPin2;

    minSpd = speedRange.min;
    maxSpd = speedRange.max;

    pinMode(PMWpin, INPUT);
    pinMode(dirPin1, OUTPUT);
    pinMode(dirPin2, OUTPUT);

    frwrd();
}

void motor::stop() {
    analogWrite(PMWpin, 0);
}

void motor::frwrd() {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
}

void motor::bckwrd() {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
}

void motor::setSpeed(double speed) {

    speedSignal = int((speed*(maxSpd-minSpd)+minSpd));

    if (speed < 0) {
        bckwrd();
        speedSignal = abs(speedSignal);
    } else {
        frwrd();
    }

    if (speedSignal <= 21) {
        speedSignal = 0;
    }

    analogWrite(PMWpin, speedSignal);
}

double motor::getPulses() const {
    return motorEncoder.getCount();
}

encoder& motor::getEncoder() {
    return motorEncoder;
}


