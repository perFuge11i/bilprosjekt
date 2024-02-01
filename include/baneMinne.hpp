#include "motor.hpp"
#include "encoder.hpp"

class baneMinne {

private:
    static const int hvormangepunkterkanvihaminnemessig = 100;
    PathPoint path[hvormangepunkterkanvihaminnemessig];
    int pathIndex = 0;

public:

    void storePathPoint(long leftEncCount, long rightEncCount, double leftMotorRPM, double rightMotorRPM, double pidError);

    //For å lagre punkter : Encoder counts, hastighet på hjulene, og error for å se hvor godt bila følger linja
    struct PathPoint {
        unsigned long time;
        long leftEncoderCount;
        long rightEncoderCount;
        double leftMotorSpeed;
        double rightMotorSpeed;
        double pidError;
        // sensordata?
    };


};
