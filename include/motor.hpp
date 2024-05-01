#ifndef BILPROSJEKT_MOTOR_HPP
#define BILPROSJEKT_MOTOR_HPP

#include <Arduino.h>

#include "encoder.hpp"
#include "structures.hpp"

class motor {
private:
    encoder motorEncoder;
    uint8_t PWMpin;
    uint8_t speedSignal;
    uint8_t minSpd;
    uint8_t maxSpd;
    uint8_t dirPin1;
    uint8_t dirPin2;

public:
    motor(motorPins& pins, range& speedRange);
    void setSpeed(double speed);
    double getPulses() const;
    encoder &getEncoder();
    void frwrd();
    void bckwrd();
    void stop();
};

#endif //BILPROSJEKT_MOTOR_HPP

