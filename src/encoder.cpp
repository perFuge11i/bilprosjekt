#include "encoder.hpp"

Encoder::Encoder(int pin) : pin(pin), count(0) {
    pinMode(pin, INPUT);
}

void Encoder::updateCount() {
    count++;
}

long Encoder::getCount() const {
    return count;
}

void Encoder::ISRWrapper(Encoder *encoder) {
    if (encoder) {
        encoder->updateCount();
    }
}

