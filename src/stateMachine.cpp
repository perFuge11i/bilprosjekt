#include "stateMachine.hpp"

stateMachine::stateMachine(uint8_t baseSpeed_) {
    baseSpeed = baseSpeed_;
}

void stateMachine::update(uint8_t state) {
    switch (state) {
        //høyre for linja
        case 8:
            adjustMotorSpeed(0, 5);
            break;
        case 9:
            adjustMotorSpeed(0, 10);
            break;
        case 10:
            adjustMotorSpeed(0, 25);
            break;
        case 11:
            adjustMotorSpeed(0, 40);
            break;
        case 12:
            adjustMotorSpeed(0, 55);
            break;
        case 13:
            adjustMotorSpeed(0, 65);
            break;
        case 14:
            adjustMotorSpeed(-5, 80);
            break;

        case 7: //midt på
            adjustMotorSpeed(0, 0);
            break;

            //venstre for linja
        case 6:
            adjustMotorSpeed(5, 0);
            break;
        case 5:
            adjustMotorSpeed(10, 0);
            break;
        case 4:
            adjustMotorSpeed(25, 0);
            break;
        case 3:
            adjustMotorSpeed(40, 0);
            break;
        case 2:
            adjustMotorSpeed(55, 0);
            break;
        case 1:
            adjustMotorSpeed(65, 0);
            break;
        case 0:
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

int8_t stateMachine::getLeftSpeed() {
    return leftMotorSpeed;
}

int8_t stateMachine::getRightSpeed() {
    return rightMotorSpeed;
}

