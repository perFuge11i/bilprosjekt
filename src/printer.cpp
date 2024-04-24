#include "printer.hpp"

printer::printer() {
    doc["carX"] = 0;
    doc["carY"] = 0;
    doc["carDirX"] = 0;
    doc["carDirY"] = 0;
    doc["lineX"] = 0;
    doc["lineY"] = 0;
    doc["lineDirX"] = 0;
    doc["lineDirY"] = 0;
    cntC = 0;
    cntL = 0;
    cntD = 0;
    full = false;
}

void printer::setCarPosition(point &carPosition) {
    cX = carPosition.x;
    cY = carPosition.y;
    doc["carX"] = cX;
    doc["carY"] = cY;
    if (cntC < numPoints) {
        carX[cntC] = float(cX);
        carY[cntC] = float(cY);
        cntC += 1;
    } else {
        full = true;
    }
}

void printer::setCarAngle(double angle_) {
    doc["carDir"] = angle_;
    if (cntD < numPoints) {
        angle[cntD] = float(angle_);
        cntD += 1;
    } else {
        full = true;
    }
}

void printer::setLinePosition(point &linePosition) {
    lX = linePosition.x;
    lY = linePosition.y;
    doc["lineX"] = lX;
    doc["lineY"] = lY;
    if (cntL < numPoints) {
        lineX[cntL] = float(lX);
        lineY[cntL] = float(lY);
        cntL += 1;
    } else {
        full = true;
    }
}

bool printer::isFull() const {
    return full;
}

void printer::setLineDirection(point &lineDirection) {
    doc["lineDirX"] = lineDirection.x;
    doc["lineDirY"] = lineDirection.y;
}

void printer::print() {
    serializeJson(doc, Serial);
    Serial.println();
    delay(100);
}

void printer::printPointList() {
    Serial.println();
    Serial.print("Bilen = {");
    for (int8_t i = 0; i < cntC; i++) {
        Serial.print("(");
        Serial.print(carX[i]);
        Serial.print(", ");
        Serial.print(carY[i]);
        Serial.print(")");
        if (i+1 != cntC) {
            Serial.print(", ");
        }
    }
    Serial.print("}");
    Serial.println();
    Serial.print("Fremme = {");
    for (int8_t i = 0; i < cntD; i++) {
        vektor lengde(12.2,0.0);
        lengde = lengde.rotate(angle[i]);
        Serial.print("(");
        Serial.print(carX[i] + lengde.x);
        Serial.print(", ");
        Serial.print(carY[i] + lengde.y);
        Serial.print(")");
        if (i+1 != cntD) {
            Serial.print(", ");
        }
    }
    Serial.print("}");
    Serial.println();
    Serial.print("Linjen = {");
    for (int8_t i = 0; i < cntL; i++) {
        Serial.print("(");
        Serial.print(lineX[i]);
        Serial.print(", ");
        Serial.print(lineY[i]);
        Serial.print(")");
        if (i+1 != cntL) {
            Serial.print(", ");
        }
    }
    Serial.print("}");
    Serial.println();
}