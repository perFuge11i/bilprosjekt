#ifndef BILPROSJEKT_HJUL_HPP
#define BILPROSJEKT_HJUL_HPP

#include <Motor.hpp>

class hjul {
private:
    Motor& motor;
    float wheelDiameter;

public:
    hjul(Motor& motorInstance, float diameter) : motor(motorInstance), wheelDiameter(diameter) {}

    float getSpeed() { // Sett motor rpm basert på lineær hastighet
        // speed = RPM * circumference / 60
        float wheelCircumference = PI * wheelDiameter;
        return motor.getRPM() * wheelCircumference / 60.0;
    }

    void setSpeed(float speed) {
        float wheelCircumference = PI * wheelDiameter;
        float targetRPM = speed * 60.0 / wheelCircumference;
        motor.setSetPoint(targetRPM);
    }
};

#endif //BILPROSJEKT_HJUL_HPP