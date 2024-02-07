#include <vector>
#include "structures.hpp"

class baneMinne {

private:
    std::vector<simplePathPoint> path;
    std::vector<PathSegment> segments;

public:
    void storePoint(unsigned long leftPulseCount, unsigned long rightPulseCount, unsigned long time);
    void generateSegments();
    void printStoredPoints();
    PathSegment getNextSegment(unsigned int currentIndex);
};


