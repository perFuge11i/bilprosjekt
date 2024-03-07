#include "car.hpp"
#include "structures.hpp"

const uint8_t baseSpeed = 100;


motorPins leftMotorPins {3, 10};
motorPins rightMotorPins {2, 9};

carDimesions dimesions {0, 16, 0, 2, 154};

car theCar(baseSpeed, leftMotorPins, rightMotorPins, dimesions);

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


