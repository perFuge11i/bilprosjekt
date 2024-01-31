#ifndef BILPROSJEKT_VEKTOR_HPP
#define BILPROSJEKT_VEKTOR_HPP

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
    void transform(const vektor xBasis, const vektor yBasis);
};

#endif //BILPROSJEKT_VEKTOR_HPP
