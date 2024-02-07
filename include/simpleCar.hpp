#ifndef BILPROSJEKT_SIMPLECAR_HPP
#define BILPROSJEKT_SIMPLECAR_HPP

#include "Arduino.h"
#include "baneMinne.hpp"
#include "motor.hpp"
#include "structures.hpp"
#include <PID_v1.h>
#include "encoder.hpp"
#include "PID.hpp"

class simpleCar {
private:
    motor leftMotor;
    motor rightMotor;
    baneMinne memory;

    static const int numSensors = 19;
    int sensorPins[numSensors] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, A0, A1, A2, A3, A4, A5, A6}; // eksempel
    int sensorValues[numSensors];
    int sensorWeights[numSensors] = {-9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    PID simplePID;
    double lineReading;
    double correction;
    double speedCorrection;

    double baseSpeed;
    unsigned long startTime; // TODO: Bestem når starttime skal settes - når motorene starter?
    unsigned long segmentStartTime;
    unsigned int currentSegmentIndex = 0;
    bool newSegment = true;

    PID sensorPID;

    double calculateSpeedCorrection(double correction);
    void saveToMemory();
    void followSegment();

public:
    simpleCar(double baseSpeed_, PIDparameters& kValues, motorPins& leftMotorPins, motorPins& rightMotorPins);
    void update();
    void beginFasterLap();
    encoder &getLeftEncoder();
    encoder &getRightEncoder();
    void initSensorPins();
    void readSensors();
    float calculateSensorValue();
};

#endif //BILPROSJEKT_SIMPLECAR_HPP