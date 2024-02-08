
#include "PID.hpp"

PID::PID(const float _kp, const float _ki, const float _kd, const float _windup) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
    windup = _windup;
    lastIntegral = 0;
    lastError = 0;
}

float PID::regulate(float dt, float target, float current) {

    float error = target - current;

    float P = error*kp;
    float I = lastIntegral + error*dt*ki;

    float D = 0;
    if (dt > 0) {
        D = (error-lastError)/dt * kd;
    }

    if (windup != 0) {
        if (I > windup) {
            I = windup;
        }

        if (I < -windup) {
            I = -windup;
        }
    }

    lastIntegral = I;
    lastError = error;

    return P + I + D;
}