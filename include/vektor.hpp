#ifndef BILPROSJEKT_VEKTOR_HPP
#define BILPROSJEKT_VEKTOR_HPP

class vektor {
private:
    struct koordinater {
        float x;
        float y;
    };

    koordinater coords;
public:
    void setValues(const float x_, const float y_);
    float getValues() const;
    float getLength() const;
    float dotProduct(vektor vektor2) const;
};

#endif //BILPROSJEKT_VEKTOR_HPP
