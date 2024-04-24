#ifndef BILPROSJEKT_PRINTER_HPP
#define BILPROSJEKT_PRINTER_HPP

#include "structures.hpp"
#include "Arduino.h"
#include "ArduinoJson.h"
#include "vektor.hpp"
const int numPoints = 50;

class printer{
private:
    JsonDocument doc;
    double cX;
    double cY;
    double lX;
    double lY;
    float carX[numPoints];
    float carY[numPoints];
    float lineX[numPoints];
    float lineY[numPoints];
    float angle[numPoints];
    int8_t cntC;
    int8_t cntL;
    int8_t cntD;
    bool full;
public:
    printer();
    void setCarPosition(point& carPosition);
    void setCarAngle(double angle);
    void setLinePosition(point& linePosition);
    void setLineDirection(point& lineDirection);
    void print();
    void printPointList();
    bool isFull() const;
};
#endif //BILPROSJEKT_PRINTER_HPP
