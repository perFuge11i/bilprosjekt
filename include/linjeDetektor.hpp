#ifndef BILPROSJEKT_LINJEDETEKTOR_HPP
#define BILPROSJEKT_LINJEDETEKTOR_HPP

#include "vektor.hpp"
#include <vector>

class linjeDetektor {
private:
    float carLength;
    float arrayWidth;

    vektor position;

public:
    linjeDetektor(const float& carLength_, const float& arrayWidth_);
    void calculate(const vektor& carPosition, const vektor& carTrajectory);
    vektor& getPosition();
};
#endif //BILPROSJEKT_LINJEDETEKTOR_HPP
