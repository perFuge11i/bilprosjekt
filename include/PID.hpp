//
// Created by Per Fugelli on 05/02/2024.
//

#ifndef BILPROSJEKT_PID_HPP
#define BILPROSJEKT_PID_HPP


class PID {
public:
    float kp;
    float ki;
    float kd;
    float windup;

    float lastIntegral;
    float lastError;

    PID(float _kp, float _ki, float _kd, float _windup = 0);
    float regulate(float dt, float target, float current);

};
#endif //BILPROSJEKT_PID_HPP
