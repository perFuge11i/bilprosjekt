#include "printer.hpp"

printer::printer() {
    carBase = SD.open("carBase.txt", FILE_WRITE);
    carFront = SD.open("carFront.txt", FILE_WRITE);
    line = SD.open("line.txt", FILE_WRITE);
    curve = SD.open("curveError.txt", FILE_WRITE);

    carBase.println("bilenBak = {");
    carFront.println("bilenFremme = {");
    line.println("linjen = {");
    curve.println("curvyness = {");

    cntBase = 0;
    cntFront = 0;
    cntLine = 0;
    cntCurve = 0;
    full = false;
}

void printer::writeCarPosition(point &carPosition) {
    cpx = carPosition.x;
    cpy = carPosition.y;
    carBase.print("(");
    carBase.print(carPosition.x);
    carBase.print(", ");
    carBase.print(carPosition.y);
    carBase.print(")");
    if (cntBase < maxCnt) {
        carBase.print(", ");
    } else {
        carBase.print("}");
        full = true;
    }

    cntBase++;
}

void printer::writeCarFront(double angle) {
    vektor lengde(12.2,0.0);
    lengde = lengde.rotate(angle);

    carFront.print("(");
    carFront.print(cpx + lengde.x);
    carFront.print(", ");
    carFront.print(cpy + lengde.x);
    carFront.print(")");
    if (cntFront < maxCnt) {
        carFront.print(", ");
    } else {
        carFront.print("}");
        full = true;
    }

    cntFront++;
}

void printer::writeLinePosition(point &linePosition) {
    line.print("(");
    line.print(linePosition.x);
    line.print(", ");
    line.print(linePosition.y);
    line.print(")");
    if (cntLine < maxCnt) {
        line.print(", ");
    } else {
        line.print("}");
        full = true;
    }

    cntLine++;
}

void printer::writeCurveError(double curveError) {
    curve.print("(");
    curve.print(cntCurve+1);
    curve.print(", ");
    curve.print(curveError);
    curve.print(")");
    if (cntCurve < maxCnt) {
        curve.print(", ");
    } else {
        curve.print("}");
        full = true;
    }

    cntCurve++;
}

void printer::closeDocuments() {
    carBase.close();
    carFront.close();
    line.close();
    curve.close();
}

bool printer::isFull() {
    return full;
}