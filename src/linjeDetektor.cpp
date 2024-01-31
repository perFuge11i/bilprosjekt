#include "linjeDetektor.hpp"

linjeDetektor::linjeDetektor(const float& carLength_, const float& arrayWidth_) : position(0,0){
    carLength = carLength_;
    arrayWidth = arrayWidth_;
}

void linjeDetektor::calculate(const vektor& carPosition, const vektor& carTrajectory) {
    float sensorValue = 0;//(Read value from arduino 2)
    float dLine = arrayWidth/2*sensorValue;

    //Starter fra bilens posisjon
    position = carPosition;

    //Lager vektor i retning mot sensor-array
    vektor carVector = carTrajectory;
    //Skalerer til lengden av bilen
    carVector.scale(carLength);

    //Lager vektor i retning mot sensoravlesning
    vektor arrayVector(carTrajectory.y, -carTrajectory.x);
    //Skalerer til lengden av sensor-avlesningen
    arrayVector.scale(dLine);

    //Flytter posisjon til linjen
    position.add(carVector);
    position.add(arrayVector);
}

vektor& linjeDetektor::getPosition() {
    return position;
}