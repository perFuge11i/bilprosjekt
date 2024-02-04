#include "simpleCar.hpp"

simpleCar::simpleCar(double baseSpeed_, PIDparameters& kValues, motorPins& leftMotorPins, motorPins& rightMotorPins) :
                     leftMotor(leftMotorPins),
                     rightMotor(rightMotorPins),
                     simplePID(&lineReading, &correction, 0, kValues.kP, kValues.kI, kValues.kP, DIRECT) {
    simplePID.SetMode(AUTOMATIC);
    baseSpeed = baseSpeed_;
}

void simpleCar::update() {
    //Setter linje avlesning
    lineReading = 0; // TODO: Les fra Arduino 2

    //Hvis linjen detekteres
    if (lineReading != -1) {
        //Kjør pid og sett motorforhold
        simplePID.Compute();
        speedCorrection = calculateSpeedCorrection(correction);
    }

    //Set motor til respektive hastiheter
    leftMotor.setSpeed(baseSpeed - speedCorrection);
    rightMotor.setSpeed(baseSpeed + speedCorrection);

    saveToMemory();

}

void simpleCar::saveToMemory() {
    unsigned long leftPulseCount = leftMotor.getPulses();
    unsigned long rightPulseCount = rightMotor.getPulses();
    unsigned long currentTime = millis() - startTime;
g
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
    if (currentSegmentIndex < memory.segments.size()) {

        auto segment = memory.getNextSegment(currentSegmentIndex);
        unsigned long currentTime = millis() - segmentStartTime;

        unsigned long currentLeftPulseCount = leftMotor.getPulses();
        unsigned long currentRightPulseCount = rightMotor.getPulses();

        bool hasReachedTarget = (currentLeftPulseCount >= segment.targetLeftPulseCount) &&
                                (currentRightPulseCount >= segment.targetRightPulseCount);

        if (hasReachedTarget) {
            currentSegmentIndex++;
            newSegment = true;
            if (currentSegmentIndex < memory.segments.size()) {
                segmentStartTime = millis(); // Reset time
            }
        }

        //todo: implementer hastighet calcs, noe sånt:

        long distanceToLeftTarget = segment.targetLeftPulseCount - currentLeftPulseCount;
        long distanceToRightTarget = segment.targetRightPulseCount - currentRightPulseCount;
        double leftSpeedAdjustment = calculateSpeedAdjustment(distanceToLeftTarget);
        double rightSpeedAdjustment = calculateSpeedAdjustment(distanceToRightTarget);

        leftMotor.setSpeed(baseSpeed + leftSpeedAdjustment);
        rightMotor.setSpeed(baseSpeed + rightSpeedAdjustment);

    }
