#ifndef BILPROSJEKT_MOTOR_HPP
#define BILPROSJEKT_MOTOR_HPP

#include <Arduino.h>
#include "encoder.hpp"
#include "structures.hpp"

class motor {
private:
    encoder encoder;
    volatile long pulseCount;
public:
    motor(motorPins& pins);
    void setSpeed(double speed);
    void pulse(unsigned int pulseCount, );
    double getPulses() const;
};

#endif //BILPROSJEKT_MOTOR_HPP

