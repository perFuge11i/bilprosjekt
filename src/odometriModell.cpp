#include "odometriModell.hpp"
#include <math.h>
#include "vektor.hpp"

odometriModell::odometriModell(const float carWidth) : trajectory(0,0), distanceTraveled(0,0){
    //Avstanden fra senter til hjulet er halvparten av bilens bredde
    dWheel = carWidth/2;
}

void odometriModell::calcualteAngle() {
    angle = (dOuter - dInner)/2*dWheel;
}

void odometriModell::calculateR() {
    r = dInner/angle+dWheel;
}

void odometriModell::calculate(float leftWheelSpeed, float rightWheelSpeed, double dTime) {

    bool turningLeft = 0;
    //Finner ut hvilket hjul som er innerst/ytterst i buen
    if (leftWheelSpeed < rightWheelSpeed) {
        //Kjører til venstre
        dInner = leftWheelSpeed*dTime;
        dOuter = rightWheelSpeed*dTime;
        turningLeft = 1;

    } else if (leftWheelSpeed > rightWheelSpeed) {
        //Kjører til hoyre
        dInner = rightWheelSpeed*dTime;
        dOuter = leftWheelSpeed*dTime;
        turningLeft = 0;

    } else {
        //Kjører rett frem

        //Finner ut hvor langt bilen kjørte i dette steget
        distanceTraveled.setValues(trajectory.x, trajectory.y);
        distanceTraveled.scale(leftWheelSpeed*dTime);
        return;

    }
    //Finner ut hvor langt
    calcualteAngle();
    calculateR();

    //Setter basis for lokal koordinatsystem (fra hvor bilen peker)
    vektor basisX(trajectory.y,-trajectory.x);
    vektor basisY(trajectory.x,trajectory.y);

    if (turningLeft == true) {
        //Finner ut hvor langt bilen kjørte i dette steget (lokalt)
        distanceTraveled.setValues(r*(cos(angle)-1),
                                   r*sin(angle));
        //Finner enhetsvektor for retningen til bilen (lokalt)
        trajectory.setValues(-sin(angle),
                             cos(angle));

    } else {
        //Finner ut hvor langt bilen kjørte i dette steget (lokalt)
        distanceTraveled.setValues(r*(-cos(angle)+1),
                                   r*sin(angle));
        //Finner enhetsvektor for retningen til bilen (lokalt)
        trajectory.setValues(sin(angle),
                             cos(angle));

    }

    //Gjør om fra lokalt koordinatsystem (forrige trajectory)
    distanceTraveled.transform(basisX,basisY);
    trajectory.transform(basisX,basisY);
}

vektor odometriModell::getDistanceTravelled() {
    return distanceTraveled;
}

vektor odometriModell::getTrajectory() {
    return trajectory;
}


