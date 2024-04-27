#ifndef BILPROSJEKT_ODOMETRY_HPP
#define BILPROSJEKT_ODOMETRY_HPP

#include "vektor.hpp"
#include "math.h"

class odometry {
private:
    double dWheel;
    double length;

    double dirAngle;

    double dInner;
    double dOuter;
    double r;
    double pathAngle;

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
    double getDirAngle() const;
};

#endif //BILPROSJEKT_ODOMETRY_HPP