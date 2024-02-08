#include <vector>
#include "structures.hpp"
#include "Arduino.h"

class baneMinne {

private:
    std::vector<simplePathPoint> path;
    std::vector<pathSegment> segments;

public:
    void storePoint(unsigned long leftPulseCount, unsigned long rightPulseCount, unsigned long time);
    void generateSegments();
    void printStoredPoints();
    int getNumberOfSegments();
    pathSegment getNextSegment(unsigned int currentIndex);
    void reset();
};


