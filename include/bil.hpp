#ifndef BILPROSJEKT_BIL_HPP
#define BILPROSJEKT_BIL_HPP

#include "vektor.hpp"
#include "odometriModell.hpp"
#include "hjul.hpp"
#include "Arduino.h"

class bil {
private:
    Motor& leftMotor;
    Motor& rightMotor;
    odometriModell odometryModel;

    double linePosition;
    double directionOutput;
    PID directionPID;
    unsigned long lastMeasurement;
    double directionAdjustment;

    /*hjul venstreHjul;
    hjul hoyreHjul;*/


public:

    bil(Motor& left, Motor& right, odometriModell& odometryModel, double Kp, double Ki, double Kd, double setPoint)
            : leftMotor(left), rightMotor(right), odometryModel(odometryModel),
              linePosition(0), directionOutput(0),
              directionPID(&linePosition, &directionOutput, &setPoint, Kp, Ki, Kd, DIRECT) {
        directionPID.SetMode(AUTOMATIC);
    }

    void update() {
        // Read the line sensors to get the current position of the line
        double lineSensorReading = readLineSensors();

        // Om linje er funnet :
        if (lineSensorReading != -1) {

            // Bruk sensor avlesning for retnings PID
            linePosition = lineSensorReading;
            directionPID.Compute();

            // Finne rette faktoren
            double adjustmentFactor = directionOutput * 0.1; // Scaling factor for PIDen, bytt som nødv.

            // Finne nye target rpms
            double leftMotorTargetRPM = calculateTargetRPM(adjustmentFactor);
            double rightMotorTargetRPM = calculateTargetRPM(-adjustmentFactor);

            // Sett nye target RPMs
            leftMotor.setSetPoint(leftMotorTargetRPM);
            rightMotor.setSetPoint(rightMotorTargetRPM);
        } else {
            //Om ingen linje - stopp
            leftMotor.setSetPoint(0);
            rightMotor.setSetPoint(0);
        }
        //intern klokke for å finne deltatid
        unsigned long currentTime = millis();
        unsigned long deltaTime = currentTime - lastMeasurement;

        // Oppdatere odemetri
        double leftWheelSpeed = leftMotor.getRPM();
        double rightWheelSpeed = rightMotor.getRPM();
        odometryModel.calculate(leftWheelSpeed, rightWheelSpeed, deltaTime);

        lastMeasurement = currentTime;

    }


    double readLineSensors() {
        int values[15]; // Lager en array for å beholde de avleste verdiene
        int position = 0;
        int total = 0;
        for (int i = 0; i < 15; i++) { // Gir hver sensor en vekt
            int value = analogRead(i);
            values[i] = value;
            position += value * (i + 1);
            total += value;
        }
        if (total == 0) return -1; // Om ingen linje er funnet
        return (double)position / total;
    }

    double calculateTargetRPM(double linePosition) {

        double baseRPM = 100.0; // Base motor hastightet
        double maxRPMOffset = 50.0; // Max offset

        // Oppdatere RPM basert på retningskontroll?
        double targetRPM = baseRPM + directionAdjustment * maxRPMOffset; //må lage funksjonen

        // Constrain RPM
        targetRPM = constrain(targetRPM, baseRPM - maxRPMOffset, baseRPM + maxRPMOffset);

        return targetRPM;
    }




};

#endif //BILPROSJEKT_BIL_HPP

