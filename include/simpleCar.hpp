#ifndef BILPROSJEKT_SIMPLECAR_HPP
#define BILPROSJEKT_SIMPLECAR_HPP

#include "vektor.hpp"
#include "odometriModell.hpp"
#include "linjeDetektor.hpp"
#include "hjul.hpp"
#include "Arduino.h"
#include "baneMinne.hpp"

class simpleCar {
private:
    float length = 0;//Midt mellom hjulene til midten av sensorarray
    float width = 0;//Mellom innsiden av hjulene
    float arrrayWidth = 0;//Mellom ytterste sensorer
    float wheelDiameter = 0;//Diamter...

    motor leftMotor;
    motor rightMotor;

    baneMinne memory;

    PID simplePID;
    double lineReading;
    double correction;
    double RPMcorrection;

    double baseRPM;
    unsigned long lastTime;

public:
    simpleCar(double baseRPM_, double Kp, double Ki, double Kd);
    void update();
    double calculateRPMcorrection(double linePosition);
    void saveToMemory() const;
};

#endif //BILPROSJEKT_SIMPLECAR_HPP
