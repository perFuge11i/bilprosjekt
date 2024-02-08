#include "simpleCar.hpp"
#include "encoder.hpp"
#include "structures.hpp"

const int buttonPin = 7;
int mode = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 200;

PIDparameters kValues {1, 1, 1};
motorPins leftMotorPins {2, 3};
motorPins rightMotorPins {2, 3};
const int baseSpeed = 100;

simpleCar myCar(baseSpeed, kValues, leftMotorPins, rightMotorPins);

void pulseLeft() {
    myCar.getLeftEncoder().updateCount();
};
void pulseRight() {
    myCar.getRightEncoder().updateCount();
};

void setup() {


    Serial.begin(9600);

    pinMode(buttonPin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(leftMotorPins.encoderPin), pulseLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(rightMotorPins.encoderPin), pulseRight, RISING);
}

void loop() {

    bool fasterLapStart = false;

    int reading = digitalRead(buttonPin);
    static int lastButtonState = HIGH;
    static unsigned long lastDebounceTime = 0;

    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading == LOW) {
            mode = (mode + 1) % 4;
            while (digitalRead(buttonPin) == LOW);
        }
    }

    lastButtonState = reading;

    switch (mode) {
        case 0:
            myCar.resetMemory();
            break;
        case 1:
            myCar.update();


            break;
        case 2:
            fasterLapStart = false;
            break;
        case 3:
            if (!fasterLapStart) {
                myCar.beginFasterLap();
                fasterLapStart = true;
            }

            myCar.followSegment();
            break;
    }
}





