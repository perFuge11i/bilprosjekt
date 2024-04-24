#include "odometry.hpp"

odometry::odometry(const double& carWidth, const double& carLength) :
        trajectory(0.0, 1.0), distanceTraveled(0.0, 0.0), lineDistance(0.0,carLength) {
    dWheel = carWidth/2.0;
    length = carLength;

    dirAngle = M_PI_2;
}

void odometry::calculateAngle() {
    pathAngle = (dOuter - dInner)/(2.0*dWheel);
}

void odometry::calculateR() {
    r = ((dInner+dOuter)*dWheel)/(dOuter-dInner);
}

void odometry::calculate(const double& leftWheelTravel, const double& rightWheelTravel) {
    bool turningLeft;

    if (leftWheelTravel < rightWheelTravel) {
        dInner = leftWheelTravel;
        dOuter = rightWheelTravel;
        turningLeft = true;
    } else if (leftWheelTravel > rightWheelTravel) {
        dInner = rightWheelTravel;
        dOuter = leftWheelTravel;
        turningLeft = false;
    } else {
        distanceTraveled.setValues(trajectory.x, trajectory.y);
        distanceTraveled.scale(leftWheelTravel);
        return;
    }

    calculateAngle();
    calculateR();

    if (turningLeft) {
        distanceTraveled.setValues(r*sin(pathAngle),
                                   r*(1.0-cos(pathAngle)));
        distanceTraveled = distanceTraveled.rotate(dirAngle);
        dirAngle += pathAngle;
    } else {
        distanceTraveled.setValues(r*sin(pathAngle),
                                   -r*(1.0-cos(pathAngle)));
        distanceTraveled = distanceTraveled.rotate(dirAngle);
        dirAngle -= pathAngle;
    }
    trajectory.setValues(cos(dirAngle), sin(dirAngle));
}

void odometry::calculateLine(const double sensorOffset) {
    if (sensorOffset == 44) {
         lineDistance.setValues(44,44);
    } else {
        lineDistance.setValues(length,-sensorOffset);
        lineDistance = lineDistance.rotate(dirAngle);
    }
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

double odometry::getDirAngle() {
    return dirAngle;
}