#include "encoder.hpp"

encoder::encoder(unsigned int pin_) {
    pin = pin_;
    count = 0;
    pinMode(pin, INPUT);
}

void encoder::updateCount() {
    count++;
}

volatile long encoder::getCount() const {
    return count;
}

