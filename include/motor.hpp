#ifndef BILPROSJEKT_MOTOR_HPP
#define BILPROSJEKT_MOTOR_HPP

#include <Arduino.h>
#include <PID_v1.h>

class motor {
private:
    int encoderPin;
    volatile long pulseCount;
    unsigned long lastMeasurement;
    double rpm;
    double output;
    double setPoint;
    double aggKp, aggKi, aggKd;
    double consKp, consKi, consKd;
    PID pidController;

public:
    motor(int pin, double cKp, double cKi, double cKd);
    void updatePID();
    void UpdatePulseCount();
    void calculateRPM();
    void update();
    double getRPM() const;
    void setSetPoint(double targetRPM);
};

#endif //BILPROSJEKT_MOTOR_HPP

