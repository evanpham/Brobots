#ifndef SONAR_H
#define SONAR_H

class Sonar {
    private:
        // Echo and Trig pins for SONAR
        int echo;
        int trig;

    public:
        Sonar(int echoPin, int trigPin);

        int readSonar();
};
#endif