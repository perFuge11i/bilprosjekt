#ifndef BILPROSJEKT_PRINTER_HPP
#define BILPROSJEKT_PRINTER_HPP

#include "structures.hpp"
#include "Arduino.h"
#include "ArduinoJson.h"
#include "vektor.hpp"
#include "SPI.h"
#include "SD.h"

class printer{
private:
    File carBase;
    File carFront;
    File line;
    File curve;
    JsonDocument doc;
    unsigned long cntBase;
    unsigned long cntFront;
    unsigned long cntLine;
    unsigned long cntCurve;
    unsigned long maxCnt = 200;

    double cpx;
    double cpy;

    bool full;
public:
    printer();
    void writeCarPosition(point& carPosition);
    void writeLinePosition(point& linePosition);
    void writeCarFront(double angle);
    void writeCurveError(double curveError);
    void closeDocuments();
    bool isFull();
};
#endif //BILPROSJEKT_PRINTER_HPP
