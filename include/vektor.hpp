#ifndef BILPROSJEKT_VEKTOR_HPP
#define BILPROSJEKT_VEKTOR_HPP

#include "structures.hpp"
#include "math.h"

class vektor {
public:
    double x;
    double y;

    vektor(const double initX, const double initY);
    void setValues(const double x_, const double y_);
    double getLength() const;
    double dotProduct(vektor vektor2) const;
    void scale(const double scalar);
    void add(const vektor vektor2);
    vektor rotate(const double angle);
    void transform(const vektor xBasis, const vektor yBasis);
};

#endif //BILPROSJEKT_VEKTOR_HPP
