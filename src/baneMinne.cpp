#include "baneMinne.hpp"


void baneMinne::storePoint(unsigned long leftPulseCount, unsigned long rightPulseCount, unsigned long time) {

    simplePathPoint point;
    point.time = time;
    point.leftPulseCount = leftPulseCount;
    point.rightPulseCount = rightPulseCount;

    path.push_back(point);
}

void baneMinne::generateSegments() {

    segments.clear();
    if (path.size() < 2) return;

    for (size_t i = 1; i < path.size(); i++) {
        PathSegment segment;
        segment.targetLeftPulseCount = path[i].leftPulseCount;
        segment.targetRightPulseCount = path[i].rightPulseCount;
        segment.targetTime = path[i].time * 0.5 ; //noe sånt

        segments.push_back(segment);
    }
}

PathSegment baneMinne::getNextSegment(unsigned int currentIndex) {
    if (currentIndex < segments.size()) {
        return segments[currentIndex];
    }
    return PathSegment{0, 0, 0};
}