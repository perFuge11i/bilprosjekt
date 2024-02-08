#include "simpleCar.hpp"

const float kp_sensor = 1.0f, ki_sensor = 0.01f, kd_sensor = 0.001f, windup_sensor = 20.0f; //usikker på windup

simpleCar::simpleCar(double baseSpeed_, PIDparameters& kValues, motorPins& leftMotorPins, motorPins& rightMotorPins)
        : leftMotor(leftMotorPins),
          rightMotor(rightMotorPins),
          sensorPID(kp_sensor, ki_sensor, kd_sensor, windup_sensor),
          baseSpeed(baseSpeed_) {
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
    if (currentSegmentIndex < memory.getNumberOfSegments()) { // Skjekker om det er flere segmenter å følge

        auto segment = memory.getNextSegment(currentSegmentIndex); //henter neste segment
        unsigned long currentTime =
                millis() - segmentStartTime; // beregner tiden som har gått siden starten av segmentet

        unsigned long currentLeftPulseCount = leftMotor.getPulses();
        unsigned long currentRightPulseCount = rightMotor.getPulses();

        bool hasReachedTarget = (currentLeftPulseCount >= segment.targetLeftPulseCount) &&
                                //må ha nådd target encoder counts på begge motorene for å gå videre
                                (currentRightPulseCount >= segment.targetRightPulseCount);

        if (hasReachedTarget) {
            currentSegmentIndex++;
            newSegment = true;
            if (currentSegmentIndex < memory.getNumberOfSegments()) {
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
}
void simpleCar::resetMemory() {
    memory.reset();
}