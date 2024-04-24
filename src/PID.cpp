
#include "PID.hpp"

PID::PID(const double _kp, const double _ki, const double _kd, const double _windup) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
    windup = _windup;
    lastIntegral = 0;
    lastError = 0;
    integrate = false;
    I = 0;
}

double PID::regulate(double dt, double target, double current) {

    double error = target - current; //0.2

    P = error * kp; // 0.2 * kp

    D = 0;
    if (dt > 0) {
        D = (error-lastError)/dt * kd;
    }

    if (integrate) {
        I = lastIntegral + error * dt * ki;
        if (windup != 0) {
            if (I > windup) {
                I = windup;
            }
            if (I < -windup) {
                I = -windup;
            }
        }
        if ((I > 0 && error < 0) || (I < 0 && error > 0)) {
            I = 0;
        }
    }

    lastError = error;
    lastIntegral = I;



    return P + I + D;
}

void PID::activateI() {
    integrate = true;
}