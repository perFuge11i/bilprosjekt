#ifndef BILPROSJEKT_BIL_HPP
#define BILPROSJEKT_BIL_HPP

#include "vektor.hpp"
#include "odometriModell.hpp"
#include "linjeDetektor.hpp"
#include "hjul.hpp"
#include "Arduino.h"
#include "baneMinne.hpp"

class bil {
private:
    float length = 0;//Midt mellom hjulene til midten av sensorarray
    float width = 0;//Mellom innsiden av hjulene
    float arrrayWidth = 0;//Mellom ytterste sensorer
    float wheelDiameter = 0;//Diamter...

    hjul venstreHjul;
    hjul hoyreHjul;
    baneMinne pathMemory;

    odometriModell odometryModel;
    linjeDetektor lineDetector;

    PID simplePID;
    double lineReading;
    double speedCorrection;

    double baseSpeed;
    trackPoint track[MAXPOINS.hvormyeminneharvi];
    int trackIndex = 0;

    unsigned long lastTime;
    double directionAdjustment;

public:
    bil(double baseSpeed_, double Kp, double Ki, double Kd, double setPoint);
    void runSimple();
    void runAdvanced();
    void update();
    double readLineSensors();
    double calculateTargetRPM(double linePosition);
    void saveToMemory() const;
};

#endif //BILPROSJEKT_BIL_HPP

