#include "stateMachine.hpp"

stateMachine::stateMachine(uint8_t baseSpeed_) {
    baseSpeed = baseSpeed_;
}

void stateMachine::update(uint8_t state) {
    switch (state) {
        //høyre for linja
        case 1:
            adjustMotorSpeed(0, 5);
            break;
        case 2:
            adjustMotorSpeed(0, 10);
            break;
        case 3:
            adjustMotorSpeed(0, 25);
            break;
        case 4:
            adjustMotorSpeed(0, 40);
            break;
        case 5:
            adjustMotorSpeed(0, 55);
            break;
        case 6:
            adjustMotorSpeed(0, 65);
            break;
        case 7:
            adjustMotorSpeed(-5, 80);
            break;

        case 0: //midt på
            adjustMotorSpeed(0, 0);
            break;

            //venstre for linja
        case -1:
            adjustMotorSpeed(5, 0);
            break;
        case -2:
            adjustMotorSpeed(10, 0);
            break;
        case -3:
            adjustMotorSpeed(25, 0);
            break;
        case -4:
            adjustMotorSpeed(40, 0);
            break;
        case -5:
            adjustMotorSpeed(55, 0);
            break;
        case -6:
            adjustMotorSpeed(65, 0);
            break;
        case -7:
            adjustMotorSpeed(80, -5);
            break;

        case 15: //venstre 90 grader
            adjustMotorSpeed(120, -10);
            break;
        case 16: //høyre 90 grader
            adjustMotorSpeed(-10, 120);
            break;
    }
}

void stateMachine::adjustMotorSpeed(uint8_t leftAdjustment, uint8_t rightAdjustment) {
    leftMotorSpeed = baseSpeed - leftAdjustment;
    rightMotorSpeed = baseSpeed - rightAdjustment;
}

void stateMachine::setBaseSpeed(uint8_t baseSpeed_) {
    baseSpeed = baseSpeed_;
}

uint8_t stateMachine::getLeftSpeed() {
    return leftMotorSpeed;
}

uint8_t stateMachine::getRightSpeed() {
    return rightMotorSpeed;
}

