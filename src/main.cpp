#include <Arduino.h>
#include <bil.hpp>
#include <Motor.hpp>

int leftMotorPin = 1;
int rightMotorPin = 2;
const int leftEncoderPin = 3;
const int rightEncoderPin = 4;

motor leftMotor(leftMotorPin, 2, 5, 1);
motor rightMotor(rightMotorPin, 2, 5, 1);

Encoder leftEncoder(leftEncoderPin);
Encoder rightEncoder(rightEncoderPin);

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

    /* Enkel forklaring på det under : IRS / Interupts er en funksjon Arduino har som lar visse viktige funksjoner
     bli håndtert umiddelbart uansett hva mikrokontrolleren holder på med, viktig for pulse counts i encoders.
     Digital pin to interrupt - gjør den digitale arduino pin til en interrupt - "håndter meg først! basically
     Resten kaller til funksjonen ISR wrapper, som oppdaterer pulse counten
     RISING - encoder funksjon som forteller nanoen å trigre interrupten når en pin går fra LOW til HIGH */
    attachInterrupt(digitalPinToInterrupt(leftEncoderPin), []() { Encoder::ISRWrapper(&leftEncoder); }, RISING);
    attachInterrupt(digitalPinToInterrupt(rightEncoderPin), []() { Encoder::ISRWrapper(&rightEncoder); }, RISING);
}

void loop() {
    Serial.print("hA");
    /*
    Car.update();
    car.recordPath();

    leftMotor.update();
    rightMotor.update();*/
} //pio device monitor -p /dev/tty.usbmodem101 -b 9600
