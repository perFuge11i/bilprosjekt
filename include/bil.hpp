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


};

#endif //BILPROSJEKT_BIL_HPP
