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
    odometry odometryModel;
    printer dataPrinter;

    int8_t readings;
    double sensorOffset;

    double lastTime;
    double dt;
    double currentTime;
    uint16_t timer;
    bool started;
    bool ignore;
    uint8_t printTimer;

    unsigned long lTime;
    double lastPulsesLeft;
    double lastPulsesRight;

    double travelPrPulse;
    double lastLeftPulseCount;
    double lastRightPulseCount;
    double leftTravel;
    double rightTravel;
    uint8_t lCnt;
    uint8_t rCnt;

    double angleToLine;
    point carToLine;

    double baseSpd;
    int8_t minSpd;
    int8_t maxSpd;

    double leftMotorSpeed;
    double rightMotorSpeed;

    point carPosition;
    vektor carPositionVector;
    point carDirection;
    vektor carDirectionVector;
    point linePosition;
    vektor linePositionvector;
    point carReferancePoint;
    point lineDir;

    static const int line_pos_size = 6;
    double linePositions[line_pos_size][2] = {{0,0},{0,0},{0,0},{0,0}};
    int currentIndex = 0;
    int counts = 0;
    lastReading = 0;
    bool isFull = false;
    double slope;

    bool lineLost;
    double lastAngleDir;
    double startTime;
    double PIDtimer;
    bool integrating;
    double directionAngle;

    bool stop = false;

    PID anglePID;

    void readSensors();
    void updateTime();
    void calculateTravel();
    void updatePosition();
    void setMotorSpeeds();
    void calculateAngle();
    void updateLinePositions(point newPosition);
    void lineRegression();

public:
    car(double baseSpeed, range& speedRange, motorPins& leftMotorPins, motorPins& rightMotorPins, carDimesions& dimensions, PIDparameters& kValues);
    void run();
    void speedTest();
    void counter();
    encoder &getLeftEncoder();
    encoder &getRightEncoder();
    void travel1Left();
    void travel1Right();
    bool stopp();
};

#endif //BILPROSJEKT_CAR_HPP

// pio device monitor --port /dev/cu.usbserial-10 --baud 9600