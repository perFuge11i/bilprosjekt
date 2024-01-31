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
    Motor(int pin, double cKp, double cKi, double cKd) :
            encoderPin(pin), pulseCount(0), lastMeasurement(0),
            rpm(0), output(0), setPoint(0),
            consKp(cKp), consKi(cKi), consKd(cKd),
            pidController(&rpm, &output, &setPoint, consKp, consKi, consKd, DIRECT) {
        pinMode(encoderPin, INPUT);
        pidController.SetMode(AUTOMATIC);
    }

    void updatePID() { //Oppdatere pid basert på error
        double error = abs(setPoint - rpm);
        if (error < 10) { //bytte RPM etterhvert
            pidController.SetTunings(consKp, consKi, consKd);
        } else {
            pidController.SetTunings(aggKp, aggKi, aggKd);
        }
        pidController.Compute();
    }

    void UpdatePulseCount() {
        pulseCount++;
    } //les opp mere på dette

    void calculateRPM() { // Finne RPM basert på pulse count
        unsigned long currentTime = millis();
        unsigned long timeDifference = currentTime - lastMeasurement;

        if (timeDifference >= 1000) { //Oppdater hver sek
            rpm = (pulseCount / 12.0) * (60000.0 / timeDifference); //finne rpm
            pulseCount = 0;
            lastMeasurement = currentTime;
        }
    }

    void update() {
        calculateRPM();
        updatePID();
        // TODO: Output til motor driver
    }

    double getRPM() const { //Getter
        return rpm;
    }

    void setSetPoint(double targetRPM) { //Setter
        setPoint = targetRPM;
    }
};


#endif //BILPROSJEKT_MOTOR_HPP
