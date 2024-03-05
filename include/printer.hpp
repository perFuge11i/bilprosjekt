#ifndef BILPROSJEKT_PRINTER_HPP
#define BILPROSJEKT_PRINTER_HPP

#include "structures.hpp"
#include "Arduino.h"

class printer{
    double plotData[6];
public:
    void setCarPosition(point& carPosition);
    void setCarTrajectory(point& carTrajectory);
    void print();
};
#endif //BILPROSJEKT_PRINTER_HPP
