#include "simpleCar.hpp"
#include "encoder.hpp"
#include "structures.hpp"

const int buttonPin = 7;
int mode = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

int mode = 0;


void pulseLeft() {
    myCar.getLeftEncoder().updateCount();
};
void pulseRight() {
    myCar.getRightEncoder().updateCount();
};

void setup() {

    leftMotorPins.encoderPin = 2;
    rightMotorPins.encoderPin = 3;

    Serial.begin(9600);

    pinMode(buttonPin, INPUT_PULLUP);


    attachInterrupt(digitalPinToInterrupt(leftMotorPins.encoderPin), pulseLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(rightMotorPins.encoderPin), pulseRight, RISING);
}

void loop() {

    static int lap = 1;

    int reading = digitalRead(buttonPin);
    static int lastButtonState = HIGH;
    static unsigned long lastDebounceTime = 0;

    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading == LOW) {
            mode = (mode + 1) % 4;
            while(digitalRead(buttonPin) == LOW);
        }
    }

    lastButtonState = reading;

    switch (mode) {
        case 1:
            if (!lapInProgress && lap == 1) {
                myCar.update()

            }
            break;
        case 2:
            //todo: Standby + kanskje loade segments, gjør klar for runde 2 kanskje?
            break;
        case 3:
            myCar.beginFasterLap();
            myCar.followSegment();

            break;
        default:
            //Todo: er dette mode 0? Hvis ja, clear all lagrede data
            break;
    }
    if (lap > 2) {
        mode = 0
        return;
    }

}




