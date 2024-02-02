#ifndef BILPROSJEKT_SIMPLECAR_HPP
#define BILPROSJEKT_SIMPLECAR_HPP

#include "vektor.hpp"
#include "odometriModell.hpp"
#include "linjeDetektor.hpp"
#include "hjul.hpp"
#include "Arduino.h"
#include "baneMinne.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include <PID_v1.h>

class simpleCar {
private:
    float length = 0; // Midt mellom hjulene til midten av sensorarray
    float width = 0; // Mellom innsiden av hjulene
    float arrayWidth = 0; // Mellom ytterste sensorer
    float wheelDiameter = 0; // Diameter...

    motor leftMotor;
    motor rightMotor;
    Encoder leftEncoder;
    Encoder rightEncoder;

    baneMinne memory;

    PID simplePID;
    double lineReading;
    double correction;
    double RPMcorrection;

    double baseRPM;
    unsigned long lastTime;

    void adjustMotorSpeeds(double desiredLeftSpeed, double desiredRightSpeed);
    void verifyPositionAndAdjust(long desiredLeftEncCount, long desiredRightEncCount);

public:
    simpleCar(double baseRPM_, double Kp, double Ki, double Kd);
    void update();
    double calculateRPMcorrection(double linePosition);
    void saveToMemory() const;
    void readLineSensors();
    void replicatePathFaster(double speedIncreaseFactor);
};

#endif // BILPROSJEKT_SIMPLECAR_HPP