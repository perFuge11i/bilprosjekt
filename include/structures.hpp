#ifndef BILPROSJEKT_STRUCTURES_HPP
#define BILPROSJEKT_STRUCTURES_HPP

struct carDimesions {
    float length = 0;//Midt mellom hjulene til midten av sensorarray
    float width = 0;//Mellom innsiden av hjulene
    float arrrayWidth = 0;//Mellom ytterste sensorer
    float wheelDiameter = 0;//Diamter...
};

struct simplePathPoint {
    unsigned long leftPulseCount;
    unsigned long rightPulseCount;
    unsigned long time;
};

struct PIDparameters {
    double kP;
    double kI;
    double kD;
};

struct motorPins {
    unsigned int encoderPin;
    unsigned int PMWpin;
};

struct pathSegment {
    unsigned long targetLeftPulseCount;
    unsigned long targetRightPulseCount;
    unsigned long targetTime;
};
#endif //BILPROSJEKT_STRUCTURES_HPP
