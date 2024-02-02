#include <vector>

class baneMinne {

private:

    struct pathPoint {
        unsigned long time;
        long leftEncCount;
        long rightEncCount;
    };
    std::vector<pathPoint> path;

public:

    void storePoint(long leftEncCount, long rightEncCount);

};

