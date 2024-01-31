#ifndef BILPROSJEKT_HJUL_HPP
#define BILPROSJEKT_HJUL_HPP

#include <motor.hpp>

class hjul {
private:
    motor motor;
    float wheelDiameter;

public:
    hjul(float diameter) : motor(0,0,0,0), wheelDiameter(diameter) {}

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