// #include <Arduino.h>
// #include "Arm.h"
// #include <Servo.h>

// #define OPEN 100
// #define CLOSED 0

// Arm::Arm(PinName base, PinName claw, PinName outPin, PinName inPin, PinName upPin, PinName downPin, Sonar outS, Sonar upS) {
//     // Assign all motor and sonar pins and attach servos
//     baseServo.attach(base);
//     clawServo.attach(claw);
//     out = outPin;
//     in = inPin;
//     up = upPin;
//     down = downPin;
//     outSonar = outS;
//     upSonar = upS;
// }

// // Opens claw
// void Arm::open() {
//     clawServo.write(OPEN);
// }

// // Closes claw
// void Arm::close() {
//     clawServo.write(CLOSED);
// }

// // Moves claw up or down
// // stop is the sonar which will trigger the stop
// // stopVal is the distance in cm at which motion is stopped
// void Arm::upDown(Sonar stop, int stopVal) {
//     // If the claw is below setpoint, go up until at setpoint
//     // If the claw is above setpoint, go down until at the setpoint
//     if (stop.readSonar() < stopVal) {
//         while (stop.readSonar() < stopVal) {
//             pwm_start(up, 100000, 500, 400, 1);
//         }
//         pwm_stop(up);
//     } else {
//         while (stop.readSonar() > stopVal) {
//             pwm_start(down, 100000, 500, 400, 1);
//         }
//         pwm_stop(down);
//     }
// }


// // Moves claw in or out
// // stop is the sonar which will trigger the stop
// // stopVal is the distance in cm at which motion is stopped
// void Arm::inOut(Sonar stop, int stopVal) {
//     // If the claw is closer than setpoint, go out until at setpoint
//     // If the claw is further than setpoint, go in until at the setpoint
//     if (stop.readSonar() < stopVal) {
//         while (stop.readSonar() < stopVal) {
//             pwm_start(out, 100000, 500, 400, 1);
//         }
//         pwm_stop(out);
//     } else {
//         while (stop.readSonar() > stopVal) {
//             pwm_start(in, 100000, 500, 400, 1);
//         }
//         pwm_stop(in);
//     }
// }

// // Pivots arm to given angle
// void Arm::pivot(int angle) {
//     baseServo.write(angle);
// }

// // Returns height, in cm, of claw
// int Arm::getHeight() {
//     return upSonar.readSonar(); // May want to add some constant distance due to offset
// }

// // Returns distance, in cm, between arm and claw
// int Arm::getDistance() {
//     return outSonar.readSonar(); // May want to add some constant distance due to offset
// }