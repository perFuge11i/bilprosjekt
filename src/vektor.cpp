#include "vektor.hpp"
#include "math.h"

vektor::vektor(const double initX, const double initY) {
    x = initX;
    y = initY;
}

void vektor::setValues(const double x_, const double y_) {
    x = x_;
    y = y_;
}

double vektor::getLength() const {
    double length = sqrt(x*x + y*y);
    return length;
}

double vektor::dotProduct(vektor vektor2) const {
    double dotProduct = this->x*vektor2.x + this->y*vektor2.y;
    return dotProduct;
}

void vektor::scale(const double scalar) {
    x *= scalar;
    y *= scalar;
}
void vektor::add(const vektor vektor2) {
    x += vektor2.x;
    y += vektor2.y;
}
void vektor::transform(const vektor xBasis, const vektor yBasis) {
    double transformedX = this->x*xBasis.x + this->y*yBasis.x;
    double transformedY = this->x*xBasis.y + this->y*yBasis.y;
    this->setValues(transformedX, transformedY);
}