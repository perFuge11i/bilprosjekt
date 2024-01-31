#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

class Encoder {
private:
    int pin;
    volatile long count;

public:
    Encoder(int pin);
    void updateCount();
    long getCount() const;
    static void ISRWrapper(Encoder *encoder);
};

#endif // ENCODER_HPP

