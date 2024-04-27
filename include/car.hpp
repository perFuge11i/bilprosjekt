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
    PID anglePID;

    double lastTime;
    double dt;
    double currentTime;
    bool started;
    double startTime;
    int startTimer;
    bool stop = false;

    int8_t reading;
    int8_t lastReading;
    double sensorOffset;
    uint8_t lCnt;
    uint8_t rCnt;
    bool lineDetected = false;

    double leftPulseTime;
    double leftLastPulseTime;
    double rightPulseTime;
    double travelPrPulse;
    double leftTravel;
    double rightTravel;
    double rightLastPulseTime;
    bool leftUpdated;
    bool rightUpdated;
    double pulseTimeThreshold;
    double timeSinceLeftUpdate;
    double timeSinceRightUpdate;

    double lastPrintTime;
    int printTimer;

    int timer90deg;
    double startTime90deg;
    double cntStartTime;
    bool ignore;

    double baseSpd;
    double PIDtimer;
    double correction;
    bool integrating;
    double leftMotorSpeed;
    double rightMotorSpeed;

    double angleToLine;
    double lastAngleDir;
    double directionAngle;
    point carToLine;
    point carPosition;
    vektor carPositionVector;
    point carDirection;
    vektor carDirectionVector;
    point linePosition;
    vektor linePositionvector;
    point carReferancePoint;
    point lineDir;
    vektor refLength;
    vektor refVector;

    static const int line_pos_size = 6;
    double linePositions[line_pos_size][2] = {{0,0},{0,0},{0,0},{0,0}};
    int currentIndex;
    int counts;
    double slope;
    int cntTimer;

    bool printerIsFull = false;

    void readSensors();
    void updateTime();
    void calculateTravel();
    void checkEncoderUpdates();
    void updatePosition();
    void setMotorSpeeds();
    void calculateAngle();
    void updateLinePositions(point newPosition);
    void lineRegression();
    static double mimimillis();
    void geogebraPrint();

public:
    car(double baseSpeed, range& speedRange, motorPins& leftMotorPins, motorPins& rightMotorPins, carDimesions& dimensions, PIDparameters& kValues);
    void run();
    void speedTest();
    void counter();
    encoder &getLeftEncoder();
    encoder &getRightEncoder();
    void travel1Left();
    void travel1Right();
    bool stopp() const;
    void updateLeftRPM();
    void updateRightRPM();
};

#endif //BILPROSJEKT_CAR_HPP

// pio device monitor --port /dev/cu.usbserial-10 --baud 9600