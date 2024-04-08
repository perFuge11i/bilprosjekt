#include "odometry.hpp"

odometry::odometry(const double& carWidth, const double& carLength) :
        trajectory(0, 1), distanceTraveled(0, 0),
        lineDistance(0,0), basisX(0,0), basisY(0,0) {
    //Avstanden fra senter til hjulet er halvparten av bilens bredde
    dWheel = carWidth/2;
    length = carLength;
    offsetR = 1;
    offsetL = 1;
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

void odometry::calculateLine(const double sensorOffset) {
    if (sensorOffset == 111) {
        lineDistance.setValues(111,111);
    } else {
        lineDistance.setValues(sensorOffset,length);
        lineDistance.transform(basisX, basisY);
    }
}

void odometry::calculateInverse(const vektor &carPosition, const vektor &linePosition) {
    vektor carLine(linePosition.x-carPosition.x, linePosition.y-carPosition.y);

    double venstrehoyre = trajectory.x*carLine.x-trajectory.y*carLine.y;

    int dir;
    if (venstrehoyre > 0) {
        dir = 1;
    } else if (venstrehoyre < 0) {
        dir = -1;
    } else {
        offsetL = 1;
        offsetR = 1;
        return;
    }

    double r = (carPosition.x*carPosition.x+carPosition.y*carPosition.y+linePosition.x*linePosition.x+linePosition.y*linePosition.y-2*(carPosition.x*linePosition.x+carPosition.y*linePosition.y))/(2*(trajectory.y*(-linePosition.x+carPosition.x)+trajectory.x*(linePosition.y-carPosition.y)));

    offsetL = 1 + dWheel/r;
    offsetR = 1 - dWheel/r;

    return;
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

double odometry::getLeftAdjustment() {
    return offsetL;
}

double odometry::getRightAdjustment() {
    return offsetR;
}