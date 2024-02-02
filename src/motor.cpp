#include "motor.hpp"

motor::motor(motorPins& pins) : motorEncoder(pins.encoderPin) {

    pinMode(pins.PMWpin, INPUT);
}

void motor::setSpeed(double speed) {
    // TODO: skriv til motorpins
}

void motor::pulse(unsigned int pulseCount) {
    // TODO: Lag funksjon
}

double motor::getPulses() const {
    return motorEncoder.getCount();
}

encoder& motor::getEncoder() {
    return motorEncoder;
}
