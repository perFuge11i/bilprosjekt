#include "simpleCar.hpp"

const float kp_sensor = 1.0f, ki_sensor = 0.01f, kd_sensor = 0.001f, windup_sensor = 20.0f; //usikker på windup

simpleCar::simpleCar(double baseSpeed_, PIDparameters& kValues, motorPins& leftMotorPins, motorPins& rightMotorPins)
        : leftMotor(leftMotorPins),
          rightMotor(rightMotorPins),
          sensorPID(kp_sensor, ki_sensor, kd_sensor, windup_sensor),
          baseSpeed(baseSpeed_) {
}

void simpleCar::initSensorPins() { //placeholder for now
    for (int i = 0; i < numSensors; ++i) {
        pinMode(sensorPins[i], INPUT);
    }
}

void simpleCar::readSensors() {
    for (int i = 0; i < numSensors; ++i) {
        sensorValues[i] = digitalRead(sensorPins[i]);
    }
}

float simpleCar::calculateSensorValue() {
    float weightedSum = 0;
    int sensorsDetected = 0;

    for (int i = 0; i < 19; ++i) {
        weightedSum += sensorValues[i] * sensorWeights[i];
        if (sensorValues[i] == 1) {
            sensorsDetected++;
        }
    }

    if (sensorsDetected > 0) {
        return weightedSum / sensorsDetected;
    } else {
        return 0;
    }
}

void simpleCar::update() {

    readSensors();
    float weightedValue = calculateSensorValue();

    static unsigned long lastUpdateTime = 0;
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0; // Delta tid i sekund.
    lastUpdateTime = currentTime;

    float sensorInput = weightedValue;
    float sensorSetpoint = 0;
    float sensorOutput = sensorPID.regulate(dt, sensorSetpoint, sensorInput);

    float adjustment = sensorOutput; //burde kanskje scales
    float leftMotorSpeed = baseSpeed - adjustment; //todo må gjøre adjustment til PWM signal først, dette kan i teorien funke også bare ikke like accurate
    float rightMotorSpeed = baseSpeed + adjustment;
    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    leftMotor.setSpeed(leftMotorSpeed); //todo
    rightMotor.setSpeed(rightMotorSpeed);

    saveToMemory();
    memory.printStoredPoints();

}

void simpleCar::adjustMotorSpeeds(float encoderOutput) {

    int baseSpeed = 100;
    int leftMotorSpeed = baseSpeed - encoderOutput;
    int rightMotorSpeed = baseSpeed + encoderOutput;

    leftMotor.setSpeed(leftMotorSpeed);
    rightMotor.setSpeed(rightMotorSpeed);
}

void simpleCar::saveToMemory() {
    unsigned long leftPulseCount = leftMotor.getPulses();
    unsigned long rightPulseCount = rightMotor.getPulses();
    unsigned long currentTime = millis() - startTime;

    memory.storePoint(leftPulseCount, rightPulseCount, currentTime);
}

encoder& simpleCar::getLeftEncoder() {
    return leftMotor.getEncoder();
}

encoder &simpleCar::getRightEncoder() {
    return rightMotor.getEncoder();
}

void simpleCar::beginFasterLap() {
    currentSegmentIndex = 0;
    memory.generateSegments();
    segmentStartTime = millis();
}

void simpleCar::followSegment() {
    if (currentSegmentIndex < memory.segments.size()) { // Skjekker om det er flere segmenter å følge

        auto segment = memory.getNextSegment(currentSegmentIndex); //henter neste segment
        unsigned long currentTime = millis() - segmentStartTime; // beregner tiden som har gått siden starten av segmentet

        unsigned long currentLeftPulseCount = leftMotor.getPulses();
        unsigned long currentRightPulseCount = rightMotor.getPulses();

        bool hasReachedTarget = (currentLeftPulseCount >= segment.targetLeftPulseCount) && //må ha nådd target encoder counts på begge motorene for å gå videre
                                (currentRightPulseCount >= segment.targetRightPulseCount);

        if (hasReachedTarget) {
            currentSegmentIndex++;
            newSegment = true;
            if (currentSegmentIndex < memory.segments.size()) {
                segmentStartTime = millis(); // Reset time for den nye segmentet
            }
        }

        //todo: implementer hastighet calcs, noe sånt: men må bruke current time og target time, pluss manglende distance

        long distanceToLeftTarget = segment.targetLeftPulseCount - currentLeftPulseCount;
        long distanceToRightTarget = segment.targetRightPulseCount - currentRightPulseCount;

        double leftSpeedAdjustment = neededSpeed(distanceToLeftTarget); //todo
        double rightSpeedAdjustment = neededSpeed(distanceToRightTarget);

        leftMotor.setSpeed(leftSpeedAdjustment);
        rightMotor.setSpeed(rightSpeedAdjustment);

    }

    double simpleCar::neededSpeed(double correction) { //gjør encoder counts per millisekund til PWM HELT SIKKERT FEIL Æ E SLITEN
        //må gjør encoder counts til RPM - tar distance som mangler med target speed for segmenter
        unsigned long currentTime = millis();
        unsigned long elapsedTime = currentTime - segmentStartTime;

        unsigned long timeLeft = (segment.targetTime > elapsedTime) ? (segment.targetTime - elapsedTime) : 0;
        double speedAdjustment = distanceToTarget / timeLeft; //counts per milli

        double pwmValue = map(speedAdjustment, -maxSpeed, maxSpeed, -255, 255);
        return constrain(pwmValue, -maxPWM, maxPWM);
    }
