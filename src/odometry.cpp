#include "odometry.hpp"
#include <math.h>
#include "vektor.hpp"

odometry::odometry(const float& carWidth) : trajectory(0, 0), distanceTraveled(0, 0){
    //Avstanden fra senter til hjulet er halvparten av bilens bredde
    dWheel = carWidth/2;
}

void odometry::calcualteAngle() {
    angle = (dOuter - dInner)/2*dWheel;
}

void odometry::calculateR() {
    r = dInner/angle+dWheel;
}

void odometry::calculate(const double& leftWheelTravel, const double& rightWheelTravel) {

    bool turningLeft;
    //Finner ut hvilket hjul som er innerst/ytterst i buen
    if (leftWheelTravel < rightWheelTravel) {
        //Kjører til venstre
        dInner = leftWheelTravel;
        dOuter = rightWheelTravel;
        turningLeft = true;

    } else if (leftWheelTravel > rightWheelTravel) {
        //Kjører til hoyre
        dInner = rightWheelTravel;
        dOuter = leftWheelTravel;
        turningLeft = false;

    } else {
        //Kjører rett frem

        //Finner ut hvor langt bilen kjørte i dette steget
        distanceTraveled.setValues(trajectory.x, trajectory.y);
        distanceTraveled.scale(leftWheelTravel);
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

vektor& odometry::getDistanceTravelled() {
    return distanceTraveled;
}

vektor& odometry::getTrajectory() {
    return trajectory;
}


