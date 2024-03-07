#ifndef BILPROSJEKT_CAR_HPP
#define BILPROSJEKT_CAR_HPP

#include "Arduino.h"
#include "trackMemory.hpp"
#include "motor.hpp"
#include "structures.hpp"
#include "encoder.hpp"
#include "PID.hpp"
#include "stateMachine.hpp"
#include "odometry.hpp"
#include "printer.hpp"
#include "math.h"

class car {
private:
    motor leftMotor;
    motor rightMotor;
    //trackMemory memory; TODO: fix vector
    stateMachine sensorValSM;
    odometry odometryModel;
    printer dataPrinter;

    double localSensorPos;

    double lastTime;
    double dt;
    double currentTime;

    double travelPrPulse;
    long lastLeftPulseCount;
    long lastRightPulseCount;
    double leftTravel;
    double rightTravel;

    point carPosition;
    vektor carPositionVector;
    point carDirection;
    vektor carDirectionVector;

    void saveToMemory();
    void readSensors();
    void updateTime();
    void calculateTravel();
    void updateCarPosition();

public:
    car(uint8_t baseSpeed, motorPins& leftMotorPins, motorPins& rightMotorPins, carDimesions& dimesions);
    void run();
    encoder &getLeftEncoder();
    encoder &getRightEncoder();
    void resetMemory();
};

#endif //BILPROSJEKT_CAR_HPP