#include "simpleCar.hpp"
#include "encoder.hpp"
#include "structures.hpp"

motorPins leftMotorPins;
motorPins rightMotorPins;
PIDparameters kValues;
const int switchPin1 = 4;
const int switchPin2 = 5;
const int switchPin3 = 6;

int mode = 0;

simpleCar myCar(100, kValues, leftMotorPins, rightMotorPins); // Eksempel parametere: baseRPM, Kp, Ki, Kd, gjør den global

void pulseLeft() {
    myCar.getLeftEncoder().updateCount();
};
void pulseRight() {
    myCar.getRightEncoder().updateCount();
};

void setup() {
    leftMotorPins.encoderPin = 2;
    rightMotorPins.encoderPin = 3;

    Serial.begin(9600);

    pinMode(switchPin1, INPUT_PULLUP); //INPUT_PULLUP: arduino funksjon som setter en default high state for open switches
    pinMode(switchPin2, INPUT_PULLUP);
    pinMode(switchPin3, INPUT_PULLUP);


    attachInterrupt(digitalPinToInterrupt(leftMotorPins.encoderPin), pulseLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(rightMotorPins.encoderPin), pulseRight, RISING);
}

void loop() {

    static int lap = 1;
    static bool lapInProgress = false;

    // tracke lap
    if (digitalRead(switchPin1) == LOW) {
        if (mode != 1) { //om mode byttes til 1 fra en anna modus
            mode = 1;
            lapInProgress = false
        }
    }

    else if (digitalRead(switchPin2) == LOW) {
        mode = 2;
        if (lap == 1 && lapInProgress) {
            lapInProgress = false;
            lap ++;
        }
    }

    else if (digitalRead(switchPin3) == LOW) {
        if (mode != 3 && lap = 2) {
            mode = 3;
        }

    }

    switch (mode) {
        case 1:
            if (!lapInProgress && lap == 1) {
                lapInProgress = true;
                myCar.update()

            }
            break;
        case 2:
            //noe her? standby mode eller hva
            break;
        case 3:
            if (!lapInProgress) {
                lapInProgress = true;
                myCar.beginFasterLap();
            } else {
                myCar.followSegment();
            }
            break;
        default:
            //lalallaalalalal noe her
            break;
    }
    if (lap > 2) {
        mode = 0
        return;
    }
}




