#ifndef BILPROSJEKT_ODOMETRY_HPP
#define BILPROSJEKT_ODOMETRY_HPP

#include "vektor.hpp"
#include "math.h"

class odometry {
private:
    double dWheel;
    double length;

    double dirAngle;

    double r;
    double pathAngle;
    double dInner;
    double dOuter;

    vektor trajectory;
    vektor distanceTraveled;
    vektor lineDistance;

    void calculateAngle();
    void calculateR();
public:
    odometry(const double& carWidth, const double& carLength);
    void calculate(const double& leftWheelTravel, const double& rightWheelTravel);
    void calculateLine(const double sensorOffset);
    vektor& getDistanceTravelled();
    vektor& getTrajectory();
    vektor& getLineDistance();
    double getDirAngle();
};

#endif //BILPROSJEKT_ODOMETRY_HPP