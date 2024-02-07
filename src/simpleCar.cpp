#include "simpleCar.hpp"

const float kp_sensor = 1.0f, ki_sensor = 0.01f, kd_sensor = 0.001f, windup_sensor = 20.0f; //usikker på windup
const float kp_motor = 1.0f, ki_motor = 0.01f, kd_motor = 0.001f, windup_motor = 20.0f;

simpleCar::simpleCar(double baseSpeed_, PIDparameters& kValues, motorPins& leftMotorPins, motorPins& rightMotorPins)
        : leftMotor(leftMotorPins),
          rightMotor(rightMotorPins),
          sensorPID(kp_sensor, ki_sensor, kd_sensor, windup_sensor),
          motorPID(kp_motor, ki_motor, kd_motor, windup_motor),
          baseSpeed(baseSpeed_) {
}

void simpleCar::update() {

    static unsigned long lastUpdateTime = 0;
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0; // Delta tid i sekund
    lastUpdateTime = currentTime;

    float sensorInput = readSensor(); //må lage
    float sensorSetpoint = 0;
    float sensorOutput = sensorPID.regulate(dt, sensorSetpoint, sensorInput);
    float encoderSetpoint = sensorOutput; // no deviation from path, hold lik fart
    float encoderInput = encoder;
    float encoderOutput = encoderPID.regulate(dt, encoderSetpoint, encoderInput);

    adjustMotorSpeeds(encoderOutput);

    saveToMemory();
    memory.printStoredPoints();

}

void simpleCar::adjustMotorSpeeds(float encoderOutput) {

    int baseSpeed = 100;
    int leftMotorSpeed = baseSpeed - encoderOutput;
    int rightMotorSpeed = baseSpeed + encoderOutput;

    leftMotor.setSpeed(leftMotorSpeed); //må lage
    rightMotor.setSpeed(rightMotorSpeed);
}




void simpleCar::saveToMemory() {
    unsigned long leftPulseCount = leftMotor.getPulses();
    unsigned long rightPulseCount = rightMotor.getPulses();
    unsigned long currentTime = millis() - startTime;

    memory.storePoint(leftPulseCount, rightPulseCount, currentTime);
}

double simpleCar::calculateSpeedCorrection(double correction) {
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
        // Beregner avstanden til målpulsantallet for både venstre og høyre motor
        long distanceToLeftTarget = segment.targetLeftPulseCount - currentLeftPulseCount;
        long distanceToRightTarget = segment.targetRightPulseCount - currentRightPulseCount;

        double leftSpeedAdjustment = calculateSpeedAdjustment(distanceToLeftTarget);
        double rightSpeedAdjustment = calculateSpeedAdjustment(distanceToRightTarget);

        leftMotor.setSpeed(baseSpeed + leftSpeedAdjustment);
        rightMotor.setSpeed(baseSpeed + rightSpeedAdjustment);

    }
