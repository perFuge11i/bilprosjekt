#include "odometry.hpp"

odometry::odometry(const double& carWidth, const double& carLength) :
        trajectory(0, 1), distanceTraveled(0, 0), lineOffset(0,0),
        baseToFront(0,carLength), lineDistance(0,0),
        basisX(0,0), basisY(0,0) {
    //Avstanden fra senter til hjulet er halvparten av bilens bredde
    dWheel = carWidth/2;
}

void odometry::calcualteAngle() {
    angle = (dOuter - dInner)/(2*dWheel);
}

void odometry::calculateR() {
    r = ((dInner+dOuter)*dWheel)/(dOuter-dInner);
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
    basisX.setValues(trajectory.y,-trajectory.x);
    basisY.setValues(trajectory.x,trajectory.y);

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

void odometry::calculateLine(const double &sensorOffset) {
    lineOffset.x = sensorOffset;

    lineDistance.add(baseToFront);
    lineDistance.add(lineOffset);
    lineDistance.transform(basisX, basisY);
}

vektor& odometry::getDistanceTravelled() {
    return distanceTraveled;
}

vektor& odometry::getTrajectory() {
    return trajectory;
}

vektor& odometry::getLineDistance() {
    return lineDistance;
}

