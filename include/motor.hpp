#ifndef BILPROSJEKT_MOTOR_HPP
#define BILPROSJEKT_MOTOR_HPP

#include <Arduino.h>
#include "encoder.hpp"
#include "structures.hpp"

class motor {
private:
    encoder motorEncoder;
    volatile long pulseCount;
    uint8_t PMWpin;
public:
    motor(motorPins& pins);
    void setSpeed(uint8_t speed);
    double getPulses() const;
    encoder &getEncoder();
};

#endif //BILPROSJEKT_MOTOR_HPP

