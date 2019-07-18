#ifndef ARM_H
#define ARM_H
#include <Servo.h>
#include "Sonar.h"

class Arm {
    private:
        // Position readout sonars
        Sonar *upSonar;
        Sonar *outSonar;

        // Position control motors
        Servo baseServo;
        PinName up;
        PinName down;
        PinName out;
        PinName in;
        Servo clawServo;
    
    public:
        Arm(PinName base, PinName claw, PinName outPin, PinName inPin, PinName upPin, PinName downPin, Sonar *outS, Sonar *upS);

        void open();
        void close();
        void upDown(Sonar stop, int stopVal);
        void inOut(Sonar stop, int stopVal);
        void pivot(int angle);
        int getHeight();
        int getDistance();
};
#endif