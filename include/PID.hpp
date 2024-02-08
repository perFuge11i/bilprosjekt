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

    PID(const float _kp, const float _ki, const float _kd, const float _windup = 0);
    float regulate(float dt, float target, float current);

};
#endif //BILPROSJEKT_PID_HPP
