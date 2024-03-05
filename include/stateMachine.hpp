#ifndef BILPROSJEKT_STATEMACHINE_HPP
#define BILPROSJEKT_STATEMACHINE_HPP

#include "Arduino.h"

class stateMachine {
private:
    uint8_t baseSpeed;

    uint8_t leftMotorSpeed;
    uint8_t rightMotorSpeed;

    void adjustMotorSpeed(uint8_t leftAdjustment, uint8_t rightAdjustment);
public:
    stateMachine(uint8_t baseSpeed_);
    void update(uint8_t state);
    void setBaseSpeed(uint8_t baseSpeed_);
    uint8_t getLeftSpeed();
    uint8_t getRightSpeed();
};

#endif //BILPROSJEKT_STATEMACHINE_HPP
