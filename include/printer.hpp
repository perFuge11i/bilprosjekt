#ifndef BILPROSJEKT_PRINTER_HPP
#define BILPROSJEKT_PRINTER_HPP

#include "structures.hpp"
#include "Arduino.h"
#include "ArduinoJson.h"

class printer{
    JsonDocument doc;
public:
    void setCarPosition(point& carPosition);
    void setCarDirection(point& carDirection);
    void setLinePosition(point& linePosition);
    void print();
};
#endif //BILPROSJEKT_PRINTER_HPP
