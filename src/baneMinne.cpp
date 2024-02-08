#include "baneMinne.hpp"


void baneMinne::storePoint(unsigned long leftPulseCount, unsigned long rightPulseCount, unsigned long time) {

    simplePathPoint point;
    point.time = time;
    point.leftPulseCount = leftPulseCount;
    point.rightPulseCount = rightPulseCount;

    path.push_back(point);

}

void baneMinne::generateSegments() {

    segments.clear(); // Tømmer listen med segmenter
    if (path.size() < 2) return; // Avslutter tidlig hvis det ikke er nok punkter til å lage segmenter

    for (size_t i = 1; i < path.size(); i++) {
        pathSegment segment;
        segment.targetLeftPulseCount = path[i].leftPulseCount;
        segment.targetRightPulseCount = path[i].rightPulseCount;
        segment.targetTime = path[i].time * 0.5 ; //Todo: denne blir kanskje feil, for lineær

        segments.push_back(segment);
    }
}

pathSegment baneMinne::getNextSegment(unsigned int currentIndex) {
    if (currentIndex < segments.size()) { // Sjekker om indeksen er innenfor grensene av segmentlisten
        return segments[currentIndex]; // Returnerer segmentet på den nåværende indeksen
    }
    return pathSegment{0, 0, 0};
}

void baneMinne::printStoredPoints() {
    for (const auto& point : path) {
        Serial.print("Time: ");
        Serial.print(point.time);
        Serial.print(", Left Pulse: ");
        Serial.print(point.leftPulseCount);
        Serial.print(", Right Pulse: ");
        Serial.println(point.rightPulseCount);
    }
}

int baneMinne::getNumberOfSegments() {
    return segments.size();
}

void baneMinne::reset() {
    path.clear();
    segments.clear();
}