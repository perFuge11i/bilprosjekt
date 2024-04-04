#ifndef BILPROSJEKT_STRUCTURES_HPP
#define BILPROSJEKT_STRUCTURES_HPP

#include "Arduino.h"

struct carDimesions {
    double length;//Midt mellom hjulene til midten av sensorarray
    double width;//Mellom innsiden av hjulene
    double arrrayWidth;//Mellom ytterste sensorer
    double wheelDiameter;//Diamter...
    double pulsesInRotation;//Finnutda
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
};

struct motorPins {
    uint8_t encoderPin;
    uint8_t PMWpin;
};

#endif //BILPROSJEKT_STRUCTURES_HPP
