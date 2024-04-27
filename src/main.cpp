#include "car.hpp"
#include "structures.hpp"

const double baseSpeed = 1;

range speedRange {20, 120};
//{0.97,0,18.7,0}
PIDparameters kValues {0.97,0,0,0};

motorPins leftMotorPins {3, 6, 8, 7};
motorPins rightMotorPins {2, 9, 10, 11};

carDimesions dimensions {12.2, 14.3, 2.0, 60.0};

car theCar(baseSpeed, speedRange, leftMotorPins, rightMotorPins, dimensions, kValues);


void pulseLeft() {
    theCar.getLeftEncoder().updateCount();
    theCar.updateLeftRPM();
}
void pulseRight() {
    theCar.getRightEncoder().updateCount();
    theCar.updateRightRPM();
}

void setup() {
    Serial.begin(115200);
    attachInterrupt(digitalPinToInterrupt(leftMotorPins.encoderPin), pulseLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(rightMotorPins.encoderPin), pulseRight, RISING);
}

void loop() {
    Serial.println(theCar.getRightEncoder().getCount());
    /*
    if (!theCar.stopp()) {
        theCar.run();
    }*/
}

// pio device monitor --port /dev/cu.usbserial-10 --baud 115200


