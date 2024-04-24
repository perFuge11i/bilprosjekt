#ifndef BILPROSJEKT_PID_HPP
#define BILPROSJEKT_PID_HPP


class PID {
public:
    double P;
    double I;
    double D;

    double kp;
    double ki;
    double kd;
    double windup;

    double lastIntegral;
    double lastError;
    bool integrate;


    PID(const double _kp, const double _ki, const double _kd, const double _windup);
    double regulate(double dt, double target, double current);
    void activateI();

};
#endif //BILPROSJEKT_PID_HPP
