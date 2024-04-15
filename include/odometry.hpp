#ifndef BILPROSJEKT_ODOMETRY_HPP
#define BILPROSJEKT_ODOMETRY_HPP

#include "vektor.hpp"
#include "math.h"

class odometry {
private:
    double dWheel;
    double r;
    double angle;

    double dInner;
    double dOuter;

    double dLine;
    double length;

    vektor basisX;
    vektor basisY;

    //Hvor bilen skal i m/s
    vektor trajectory;
    //Hvor langt bilen kjørte i m;
    vektor distanceTraveled;

    vektor lineDistance;

    double offsetL;
    double offsetR;

    void calcualteAngle();
    void calculateR();

public:
    odometry(const double& carWidth, const double& carLength);
    void calculate(const double& leftWheelTravel, const double& rightWheelTravel);
    void calculateLine(const double sensorOffset);
    void lineRegression();
    void calculateInverse(const vektor& carPosition, const vektor& linePosition);
    vektor& getDistanceTravelled();
    vektor& getTrajectory();
    vektor& getLineDistance();
    double getLeftAdjustment();
    double getRightAdjustment();
};

#endif //BILPROSJEKT_ODOMETRY_HPP