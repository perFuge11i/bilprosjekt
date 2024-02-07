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

    myCar.update();



    static int lap = 1;
    static bool lapInProgress = false;

    int reading = digitalRead(buttonPin);
    static int lastButtonState = HIGH; // assuming the button is in pull-up mode and HIGH when not pressed
    static unsigned long lastDebounceTime = 0;

    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        // if the button state has changed:
        if (reading == LOW) { // assuming the button is pressed when LOW
            mode = (mode + 1) % 4; // Cycle through modes 0 to 3
            while(digitalRead(buttonPin) == LOW); // Wait for button release to avoid multiple increments
        }
    }

    lastButtonState = reading;

    switch (mode) {
        case 1:
            if (!lapInProgress && lap == 1) {
                lapInProgress = true;
                myCar.update()

            }
            break;
        case 2:
            //noe her? standby mode eller hva
            lapInProgress = false;
            break;
        case 3:
            myCar.beginFasterLap();
            myCar.followSegment();

            break;
        default:
            //lalallaalalalal noe her reset data ens
            break;
    }
    if (lap > 2) {
        mode = 0
        return;
    }
*/

}




