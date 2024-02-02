#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

class encoder {
private:
    unsigned int pin;
    volatile long count;

public:
    encoder(unsigned int pin_);
    void updateCount();
    volatile long getCount() const;
};

#endif // ENCODER_HPP

