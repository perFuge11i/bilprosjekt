#include <Arduino.h>
#include <bil.hpp>
#include <Motor.hpp>

int leftMotorPin = 1;
int rightMotorPin = 2;

double leftMotorKp, leftMotorKi, leftMotorKd;
double rightMotorKp, rightMotorKi, rightMotorKd;
double directionKp, directionKi, directionKd;


/*Motor leftMotor(leftMotorPin, leftMotorKp, leftMotorKi, leftMotorKd);
Motor rightMotor(rightMotorPin, rightMotorKp, rightMotorKi, rightMotorKd);

odometriModell odometryModel();

bil Car(leftMotor, rightMotor, odometryModel, directionKp, directionKi, directionKd);
*/
void setup() {
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    Serial.print("hA");
    /*
    Car.update();

    leftMotor.update();
    rightMotor.update();*/
} //pio device monitor -p /dev/tty.usbmodem101 -b 9600
