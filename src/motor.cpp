#include "motor.hpp"

motor::motor(motorPins& pins) : motorEncoder(pins.encoderPin) {
    PMWpin = pins.PMWpin;
    pinMode(PMWpin, INPUT);
}

void motor::setSpeed(uint8_t speed) {
    analogWrite(PMWpin, speed);
}

double motor::getPulses() const {
    return motorEncoder.getCount();
}

encoder& motor::getEncoder() {
    return motorEncoder;
}


