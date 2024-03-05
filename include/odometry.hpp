#ifndef BILPROSJEKT_ODOMETRY_HPP
#define BILPROSJEKT_ODOMETRY_HPP

#include "vektor.hpp"

class odometry {
private:
    float dWheel;
    float r;
    float angle;

    float dInner;
    float dOuter;

    //Hvor bilen skal i m/s
    vektor trajectory;
    //Hvor langt bilen kjørte i m;
    vektor distanceTraveled;

    void calcualteAngle();
    void calculateR();

public:
    odometry(const float& carWidth);
    void calculate(const double& leftWheelTravel, const double& rightWheelTravel);
    vektor& getDistanceTravelled();
    vektor& getTrajectory();
};

#endif //BILPROSJEKT_ODOMETRY_HPP