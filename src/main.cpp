#include "car.hpp"
#include "encoder.hpp"
#include "structures.hpp"

const uint8_t baseSpeed = 100;


motorPins leftMotorPins {2, 3};
motorPins rightMotorPins {2, 3};

carDimesions dimesions {0, 0, 0, 0, 0};

car theCar(baseSpeed, leftMotorPins, rightMotorPins, dimesions);

void pulseLeft() {
    theCar.getLeftEncoder().updateCount();
};
void pulseRight() {
    theCar.getRightEncoder().updateCount();
};

void setup() {
    Serial.begin(9600);

    attachInterrupt(digitalPinToInterrupt(leftMotorPins.encoderPin), pulseLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(rightMotorPins.encoderPin), pulseRight, RISING);
}

void loop() {
    //theCar.run();
    Serial.println(1);
}





