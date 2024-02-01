#include "bil.hpp"

bil::bil(double baseSpeed_, double Kp, double Ki, double Kd, double setPoint) : venstreHjul(wheelDiameter), hoyreHjul(wheelDiameter),
                                                            odometryModel(width), lineDetector(length,arrrayWidth),
                                                            simplePID(&lineReading, &speedCorrection, &setPoint, Kp, Ki, Kd, DIRECT) {
    simplePID.SetMode(AUTOMATIC);
    baseSpeed = baseSpeed_;
}

void bil::runSimple() {
    lineReading = 0;//Les fra Arduino 2

    if (lineReading != -1) {
        simplePID.Compute();
    }

    venstreHjul.setSpeed(baseSpeed - speedCorrection);
    hoyreHjul.setSpeed(baseSpeed + speedCorrection);

    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastTime;

    //LEGG TIL (Lagre data til minne)

    lastTime = currentTime;
}

void bil::runAdvanced() {

}


//Placeholder, skal flyttes til runSimple og runAdvanced
void bil::update() {
    // Read the line sensors to get the current position of the line
    double lineSensorReading = readLineSensors();

    // Om linje er funnet :
    if (lineSensorReading != -1) {

        // Bruk sensor avlesning for retnings PID
        linePosition = lineSensorReading;
        directionPID.Compute();

        // Finne rette faktoren
        double adjustmentFactor = directionOutput * 0.1; // Scaling factor for PIDen, bytt som nødv.

        // Finne nye target rpms
        double leftMotorTargetRPM = calculateTargetRPM(adjustmentFactor);
        double rightMotorTargetRPM = calculateTargetRPM(-adjustmentFactor);

        // Sett nye target RPMs
        leftMotor.setSetPoint(leftMotorTargetRPM);
        rightMotor.setSetPoint(rightMotorTargetRPM);
    } else {
        //Om ingen linje - stopp
        leftMotor.setSetPoint(0);
        rightMotor.setSetPoint(0);
    }
    //intern klokke for å finne deltatid
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastMeasurement;

    // Oppdatere odemetri
    double leftWheelSpeed = leftMotor.getRPM();
    double rightWheelSpeed = rightMotor.getRPM();
    odometryModel.calculate(leftWheelSpeed, rightWheelSpeed, deltaTime);

    lastMeasurement = currentTime;
}


//Skal fikses av andre arduino
double bil::readLineSensors() {
    int values[15]; // Lager en array for å beholde de avleste verdiene
    int position = 0;
    int total = 0;
    for (int i = 0; i < 15; i++) { // Gir hver sensor en vekt
        int value = analogRead(i);
        values[i] = value;
        position += value * (i + 1);
        total += value;
    }
    if (total == 0) return -1; // Om ingen linje er funnet
    return (double)position / total;
}

double bil::calculateTargetRPM(double linePosition) {
    double baseRPM = 100.0; // Base motor hastightet
    double maxRPMOffset = 50.0; // Max offset

    // Oppdatere RPM basert på retningskontroll?
    double targetRPM = baseRPM + directionAdjustment * maxRPMOffset; //må lage funksjonen

    // Constrain RPM
    targetRPM = constrain(targetRPM, baseRPM - maxRPMOffset, baseRPM + maxRPMOffset);

    return targetRPM;
}

void bil::recordPath() { //kan kanskje slette
    // Kalle denne i gjevne intervaller
    storePathPoint();
}


void bil::storePathPoint() {

    long leftEncCount = leftEncoder.getCount();
    long rightEncCount = rightEncoder.getCount();
    double leftMotorRPM = leftMotor.getRPM();
    double rightMotorRPM = rightMotor.getRPM();
    double pidError = simplePID.GetError(); //må lage denne

    pathMemory.storePathPoint(leftEncCount, rightEncCount, leftMotorRPM, rightMotorRPM, pidError); //sender verdiene til baneminne.hpp - lagrer videre i cpp
}
