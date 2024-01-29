#ifndef BILPROSJEKT_BIL_HPP
#define BILPROSJEKT_BIL_HPP

#include "vektor.hpp"
#include "vector"
#include "odometriModell.hpp"
#include "hjul.hpp"

class bil {
private:
    struct dimensjoner {
        const float lengde;
        const float bredde;
        const float centerOfRotation;
        const float hjulRadius;
    };
    struct koordinater {
        float x;
        float y;
    };

    koordinater posisjon;
    vektor vektor;
    std::vector<koordinater> path;

    odometriModell odometriModell;

    hjul venstreHjul;
    hjul hoyreHjul;
public:
    bil(const float lenge_, const float bredde_, const float hjulRadius_);

    double readLineSensors() {
        int position = 0;
        int total = 0;
        for (int = 0; i < 15; i++) { //15 sensore
            int value = analogRead(i);
            values[i] = value;
            position += value * (i + 1); //vektet av sensor posisjon
            total += value;
        } if (total == 0) return -1;

        return (double)position / total;

    }

    double calculateTargetRPM(double linePosition) {

        double baseRPM = 100.0;
        double maxRPMOffset = 50.0;
        if (linePosition == -1) {
            return baseRPM / 2;
        }
        double error = 3 - linePosition;
        double rpmAdjustment = (error / 15) * maxRPMOffset; //15 sensore

        double targetRPM = baseRPM - rpmAdjustment;
        targetRPM = constrain(targetRPM, baseRPM - maxRPMOffset, baseRPM + maxRPMOffset);

        return targetRPM;
    }



};

#endif //BILPROSJEKT_BIL_HPP

