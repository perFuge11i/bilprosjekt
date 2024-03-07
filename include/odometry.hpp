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

    vektor basisX;
    vektor basisY;

    //Hvor bilen skal i m/s
    vektor trajectory;
    //Hvor langt bilen kjørte i m;
    vektor distanceTraveled;

    vektor lineOfset;
    vektor baseToFront;
    vektor lineDistance;

    void calcualteAngle();
    void calculateR();

public:
    odometry(const double& carWidth, const double& carLength);
    void calculate(const double& leftWheelTravel, const double& rightWheelTravel);
    void calculateLine(const double& sensorOfset);
    vektor& getDistanceTravelled();
    vektor& getTrajectory();
    vektor& getLineDistance();
};

#endif //BILPROSJEKT_ODOMETRY_HPP