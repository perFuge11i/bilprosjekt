#include "simpleCar.hpp"

simpleCar myCar(100, 2.0, 5.0, 1.0); // Eksempel parametere: baseRPM, Kp, Ki, Kd, gjør den global

void setup() {
    Serial.begin(9600);

}

void loop() {
    // tracke lap
    static int lap = 1;
    static bool lapInProgress = false;

    // skjekke om lappen e in progress
    if (!lapInProgress) {
        lapInProgress = true;

        if (lap == 1) {
            Serial.println("Starter første runde.");
            // Kode til å starte første runde og recorde
            // Kjør rundt banen og mycar.update()?
            while (!isLapComplete()) {
                myCar.update(); // Oppdatere jevnlig
            }
            Serial.println("Første runde ferdig.");
        } else if (lap == 2) {
            Serial.println("Starter andre runde, raskere");
            // Kode til å starte andre runde raskere
            myCar.replicatePathFaster(1.5); // eksempel
            Serial.println("Andre runde fullført.");
            // Stopp loopen?
        }

        lap++;
        lapInProgress = false; // Reset flagget om runden e ferdig
    }


    // Må implemeter en metode til å bryte loopen eller få nanoen til å sove etter andre runden, for å hindre at det aldri stopper
    if (lap > 2) {
        // stopp bila ens
        return;
    }
}


// Må finne ut hvordan vi skjekker at en runde er ferdig
bool isLapComplete() {

    // Sensor, timer, manual input..
    return false;
}

