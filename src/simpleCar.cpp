#include "simpleCar.hpp"

const float kp_sensor = 1.0f, ki_sensor = 0.01f, kd_sensor = 0.001f, windup_sensor = 20.0f; //usikker på windup

simpleCar::simpleCar(double baseSpeed_, PIDparameters& kValues, motorPins& leftMotorPins, motorPins& rightMotorPins)
        : leftMotor(leftMotorPins),
          rightMotor(rightMotorPins),
          sensorPID(kp_sensor, ki_sensor, kd_sensor, windup_sensor),
          baseSpeed(baseSpeed_) {
}

void simpleCar::update() {

    static unsigned long lastUpdateTime = 0;
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0; // Delta tid i sekund,
    lastUpdateTime = currentTime;

    float sensorInput = readSensor(); //må lage
    float sensorSetpoint = 0;
    float sensorOutput = sensorPID.regulate(dt, sensorSetpoint, sensorInput); //todo: legg til motor outputs


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

double simpleCar::calculateSpeedCorrection(double correction) { //blir denne brukt
    //Constrain correction
    double maxCorrection = 1/2*baseSpeed;
    return constrain(correction, -maxCorrection, maxCorrection);
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

        bool hasReachedTarget = (currentLeftPulseCount >= segment.targetLeftPulseCount) &&
                                (currentRightPulseCount >= segment.targetRightPulseCount);

        if (hasReachedTarget) {
            currentSegmentIndex++;
            newSegment = true;
            if (currentSegmentIndex < memory.segments.size()) {
                segmentStartTime = millis(); // Reset time for den nye segmentet
            }
        }

        //todo: implementer hastighet calcs, noe sånt:

        long distanceToLeftTarget = segment.targetLeftPulseCount - currentLeftPulseCount;
        long distanceToRightTarget = segment.targetRightPulseCount - currentRightPulseCount;

        double leftSpeedAdjustment = calculateSpeedAdjustment(distanceToLeftTarget);
        double rightSpeedAdjustment = calculateSpeedAdjustment(distanceToRightTarget);

        leftMotor.setSpeed(baseSpeed + leftSpeedAdjustment); //todo: lage setSpeed som gjør encoder counts per millisekund til PWM
        rightMotor.setSpeed(baseSpeed + rightSpeedAdjustment);

    }
