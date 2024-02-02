#ifndef BILPROSJEKT_STRUCTURES_HPP
#define BILPROSJEKT_STRUCTURES_HPP

struct carDimesions {
    float length = 0;//Midt mellom hjulene til midten av sensorarray
    float width = 0;//Mellom innsiden av hjulene
    float arrrayWidth = 0;//Mellom ytterste sensorer
    float wheelDiameter = 0;//Diamter...
};

struct simplePathPoint {
    long leftEncoderCount;
    long rightEncoderCount;
    double leftMotorSpeed;
    double rightMotorSpeed;
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

#endif //BILPROSJEKT_STRUCTURES_HPP
