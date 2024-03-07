#ifndef BILPROSJEKT_PRINTER_HPP
#define BILPROSJEKT_PRINTER_HPP

#include "structures.hpp"
#include "Arduino.h"

class printer{
    double plotData[6];
public:
    void setCarPosition(point& carPosition);
    void setCarDirection(point& carDirection);
    void print();
};
#endif //BILPROSJEKT_PRINTER_HPP
