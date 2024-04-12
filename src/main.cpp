#include "car.hpp"
#include "structures.hpp"

const uint8_t baseSpeed = 120;

PIDparameters kValues {4,0,4}; //3, 0, 3.1

motorPins leftMotorPins {3, 10};
motorPins rightMotorPins {2, 9};

carDimesions dimesions {12, 16, 0, 2, 29};

car theCar(baseSpeed, leftMotorPins, rightMotorPins, dimesions, kValues);



void pulseLeft() {
    theCar.getLeftEncoder().updateCount();
}
void pulseRight() {
    theCar.getRightEncoder().updateCount();
}

void setup() {
    Serial.begin(9600);

    attachInterrupt(digitalPinToInterrupt(leftMotorPins.encoderPin), pulseLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(rightMotorPins.encoderPin), pulseRight, RISING);
}

void loop() {
    theCar.run();
}


// pio device monitor --port /dev/cu.usbserial-10 --baud 9600


