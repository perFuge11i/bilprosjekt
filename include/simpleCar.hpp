#ifndef BILPROSJEKT_SIMPLECAR_HPP
#define BILPROSJEKT_SIMPLECAR_HPP

#include "Arduino.h"
#include "baneMinne.hpp"
#include "motor.hpp"
#include "structures.hpp"
#include <PID_v1.h>

class simpleCar {
private:
    motor leftMotor;
    motor rightMotor;

    baneMinne memory;

    PID simplePID;
    double lineReading;
    double correction;
    double speedCorrection;

    double baseSpeed;
    unsigned long startTime; // TODO: Bestem når starttime skal settes

    double calculateSpeedCorrection(double correction);
    void saveToMemory() const;

public:
    simpleCar(double baseSpeed_, PIDparameters& kValues, motorPins& leftMotorPins, motorPins& rightMotorPins);
    void update();
};

#endif //BILPROSJEKT_SIMPLECAR_HPP