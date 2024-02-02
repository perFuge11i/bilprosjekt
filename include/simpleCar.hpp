#ifndef BILPROSJEKT_SIMPLECAR_HPP
#define BILPROSJEKT_SIMPLECAR_HPP

#include "vektor.hpp"
#include "odometriModell.hpp"
#include "linjeDetektor.hpp"
#include "hjul.hpp"
#include "Arduino.h"
#include "baneMinne.hpp"
#include "encoder.hpp"

class simpleCar {
private:
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
public:
    simpleCar(double baseRPM_,
              PIDparameters& kValues, PIDparameters& motorKvalues,
              motorPins& leftMotorPins, motorPins& rightMotorPins);
    void update();
    double calculateRPMcorrection(double correction);
    void saveToMemory() const;
    void readLineSensors();
};

#endif //BILPROSJEKT_SIMPLECAR_HPP