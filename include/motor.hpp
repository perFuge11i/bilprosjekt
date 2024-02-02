#ifndef BILPROSJEKT_MOTOR_HPP
#define BILPROSJEKT_MOTOR_HPP

#include <Arduino.h>
#include <PID_v1.h>
#include "structures.hpp"

class motor {
private:
    int encoderPin;
    int pwmPin;
    volatile long pulseCount;
    unsigned long lastMeasurement;
    double rpm;
    double output;
    double setPoint;

    PID pidController;

public:
    motor(motorPins& pins, PIDparameters& kValues);
    void updatePID();
    void UpdatePulseCount();
    void calculateRPM();
    void update();
    void setRPM(double targetRPM);
    double getRPM() const;
};

#endif //BILPROSJEKT_MOTOR_HPP

