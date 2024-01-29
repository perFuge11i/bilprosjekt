#include <Arduino.h>

Motor leftMotor(leftMotorPin, leftMotorKp, leftMotorKi, leftMotorKd);
Motor rightMotor(rightMotorPin, rightMotorKp, rightMotorKi, rightMotorKd);

void setup() {
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN1, OUTPUT);
    pinMode(MOTOR_DIR_PIN2, OUTPUT);
}

void loop() {

    double linePosition = readLineSensors();

    // Finne target motor rpm for hver motor
    double leftMotorTargetRPM = calculateTargetRPM(linePosition, "left");
    double rightMotorTargetRPM = calculateTargetRPM(linePosition, "right");

    //Sette RPM for hver motor
    leftMotor.setSetPoint(leftMotorSpeedSetpoint); //splitte i 2?
    rightMotor.setSetPoint(rightMotorSpeedSetpoint);

    // Oppdatere motorene - Oppdatere PID, finne RPM, output til motor driver osvosv
    leftMotor.update();
    rightMotor.update();

}