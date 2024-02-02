#include "motor.hpp"
#include "encoder.hpp"
#include <vector>

class baneMinne {

private:
    struct pathPoint {
        unsigned long time;
        long leftEncoderCount;
        long rightEncoderCount;
        double leftMotorSpeed;
        double rightMotorSpeed;
    };
    std::vector<pathPoint> path;

public:

    void storePoint(double leftMotorRPM, double rightMotorRPM, long leftEncCount, long rightEncCount);

    //For å lagre punkter : Encoder counts, hastighet på hjulene, og error for å se hvor godt bila følger linja
};
