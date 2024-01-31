#ifndef BILPROSJEKT_ODOMETRIMODELL_HPP
#define BILPROSJEKT_ODOMETRIMODELL_HPP

#include "vektor.hpp"

class odometriModell {
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
    odometriModell(const float& carWidth);
    void calculate(const float& leftWheelSpeed, const float& rightWheelSpeed, const double& dTime);
    vektor& getDistanceTravelled();
    vektor& getTrajectory();
};
#endif //BILPROSJEKT_ODOMETRIMODELL_HPP
