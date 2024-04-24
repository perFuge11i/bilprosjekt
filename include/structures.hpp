#ifndef BILPROSJEKT_STRUCTURES_HPP
#define BILPROSJEKT_STRUCTURES_HPP

#include "Arduino.h"

struct carDimesions {
    double length;//Midt mellom hjulene til midten av sensorarray
    double width;//Mellom innsiden av hjulene
    double wheelDiameter;//Diamter...
    double pulsesInRotation;//Finnutda
};

struct range {
    uint8_t min;
    uint8_t max;
};

struct point {
    double x;
    double y;
    unsigned long timeStamp;
};

struct PIDparameters {
    double kP;
    double kI;
    double kD;
    double windup;
};

struct motorPins {
    uint8_t encoderPin;
    uint8_t PMWpin;
    uint8_t dirPin1;
    uint8_t dirPin2;
};

#endif //BILPROSJEKT_STRUCTURES_HPP
