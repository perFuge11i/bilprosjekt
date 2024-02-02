#include <vector>
#include "structures.hpp"

class baneMinne {

private:
    std::vector<simplePathPoint> path;

public:
    void storePoint(unsigned long leftPulseCount, unsigned long rightPulseCount, unsigned long time);
};

