#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <FreeMono9pt7b.h>
#include <time.h>
#include "Sonar.h"
#include "Arm.h"

// #if (SSD1306_LCDHEIGHT != 64)
// #error("Height incorrect, please fix Adafruit_SSD1306.h!");
// #endif

#define OLED_RESET -1  // Not used
Adafruit_SSD1306 display(OLED_RESET);

#define pot PA5
#define button PA12
#define motorL PB_0
#define motorR PA_0
#define motorLrev PB_1
#define motorRrev PA_1
#define leftQRD PB10
#define leftestQRD PB11
#define rightQRD PA4
#define rightestQRD PA3
#define leftSplitQRD PA15
#define rightSplitQRD PB4
#define upEcho PB12
#define upTrig PB13
#define outEcho PB14
#define outTrig PB15
#define baseServo PB_4
#define clawServo PA_5
#define armOut PA_6
#define armIn PA_7
#define armUp PA_3
#define armDown PA_2
#define splitLED PA1

// PWM limits, right and left value, and correction value
int correction;
int minPWM = 0;
int maxPWM = 450;
int PWMleft = (maxPWM - minPWM)/2;
int PWMright = (maxPWM - minPWM)/2;
int splits = 0;

// PID gains, tuning pot, increment, and error variables
float Kp = 85.0;
float Kd = 135.0;
float inc = 1;
int potVal, oldPot;
int error, lastError, deltaError = 0;

// Timers used for derivative calculations
int lastSwitch = millis();
int tPrev, tCurrent;

// Bools for split handling
bool stayLeft = true;
bool splitting = false;
int lastSplit = millis();

// Bools for mode tracking/changing
bool tuneKp = false;
bool tuneKd = false;
bool freeSpins = false;
bool moving = false;

// Bools for QRD logic
bool left = false;
bool center = false;
bool right = false;

// Function prototypes
void updatePWMvalue(int max, int min, bool leftMotor);
void splitProcedure(void);
void noSplitProcedure(void);
void change_mode(void);
float calcDerivative(void);
void updateError(void);
int readSonar(void);
void getStone(Arm a);

// Arm and Sonar initialization
// out and up are sonars which measure distance out and up respectively
// All other inputs are pin names corresponding to specific control pins
// Ex. armUp is a pin which, when powered, moves the arm up
Sonar up(upEcho, upTrig);
Sonar out(outEcho, outTrig); 
//Arm arm(baseServo, clawServo, armOut, armIn, armUp, armDown, out, up);


void setup() {
  Serial.begin(9600);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  // init done

  // Startup display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setFont(&FreeMono9pt7b);
  display.setCursor(3,10);
  display.println("Edith");
  display.println("Booting");
  display.println("Up...");
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setCursor(20,40);
  display.println("ZOOM!");
  display.display();

  pinMode(button, INPUT_PULLUP);
  pinMode(leftQRD, INPUT_PULLUP);
  pinMode(rightQRD, INPUT_PULLUP);
  pinMode(leftestQRD, INPUT_PULLUP);
  pinMode(rightestQRD, INPUT_PULLUP);
  pinMode(rightSplitQRD, INPUT_PULLUP);
  pinMode(leftSplitQRD, INPUT_PULLUP);
  pinMode(motorLrev, INPUT_PULLDOWN);
  pinMode(motorRrev, INPUT_PULLDOWN);
  // attachInterrupt(digitalPinToInterrupt(button), change_mode, LOW);

}

void loop() {
  // Read mode change button
  if (!digitalRead(button)) {
    change_mode();
  }

  if (tuneKp) {
    // Tune Kp mode (Turn off motors)
    pwm_stop(motorL);
    pwm_stop(motorR);
    pwm_stop(motorLrev);
    pwm_stop(motorRrev);

    oldPot = potVal;
    potVal = analogRead(pot);
    if (potVal > oldPot) {
      Kp += inc;
    } else if (potVal < oldPot) {
      Kp -= inc;
    }
    display.clearDisplay();
    display.setCursor(4,45);
    display.print("Kp: ");
    display.print(Kp);
    display.display();

  } else if (tuneKd) {
    // Tune Kd mode (Turn motors off)
    pwm_stop(motorL);
    pwm_stop(motorR);

    oldPot = potVal;
    potVal = analogRead(pot);
    if (potVal > oldPot) {
      Kd += inc;
    } else if (potVal < oldPot) {
      Kd -= inc;
    }
    display.clearDisplay();
    display.setCursor(4,45);
    display.print("Kd: ");
    display.print(Kd);
    display.display();

  } else if (freeSpins) {
    // Free potentiometer spins (Turn motors off)
    pwm_stop(motorL);
    pwm_stop(motorR);

    display.clearDisplay();
    display.setCursor(4,40);
    display.print("FREE SPINS");
    display.display();

  } else {
    // Main PID sequence
    updateError();

    correction = Kp*error + Kd*calcDerivative(); // Correction tend to be + when right of tape

    // Update PWM values for left and right motor
    updatePWMvalue(maxPWM, minPWM, true);
    updatePWMvalue(maxPWM, minPWM, false);
   
    digitalWrite(motorRrev, 0);
    digitalWrite(motorLrev, 0);
    pwm_start(motorR, 100000, 500, PWMright, 1);
    pwm_start(motorL, 100000, 500, PWMleft, 1);
    // Serial.println(correction);
    // Serial.println(PWMleft);
    // Serial.println(PWMright);
    // delay(1000);
  }
}

void updateError(void) {
  // Main PID control sequence
    if (!digitalRead(leftestQRD) && digitalRead(leftQRD) && digitalRead(rightQRD) && !digitalRead(rightestQRD)) {
      // Centered
      left = false;
      right = false;
  
      error = 0;
      Serial.println("Centered");
      splitting = false;

    } else if (!digitalRead(leftestQRD) && !digitalRead(leftQRD) && digitalRead(rightQRD) && !digitalRead(rightestQRD)) {
      // Misaligned leftish
      left = true;
      right = false;

      error = -1;
      Serial.println("leftish");
      splitting = false;

    } else if (!digitalRead(leftestQRD) && digitalRead(leftQRD) && !digitalRead(rightQRD) && !digitalRead(rightestQRD)) {
      // Misaligned rightish
      right = true;
      left = false;

      error = 1;
      Serial.println("rightish");
      splitting = false;

    } else if (!digitalRead(leftestQRD) && !digitalRead(leftQRD) && digitalRead(rightQRD) && digitalRead(rightestQRD)) {
      // Misaligned left
      left = true;
      right = false;

      error = -3;
      Serial.println("left");
      splitting = false;

    } else if (digitalRead(leftestQRD) && digitalRead(leftQRD) && !digitalRead(rightQRD) && !digitalRead(rightestQRD)) {
      // Misaligned right
      right = true;
      left = false;

      error = 3;
      Serial.println("right");
      splitting = false;

    } else if (!digitalRead(leftestQRD) && !digitalRead(leftQRD) && !digitalRead(rightQRD) && digitalRead(rightestQRD)){
      // Misaligned quite left
      left = true;
      right = false;
  
      error = -5;
      Serial.println("quite left");
      splitting = false;

    } else if (digitalRead(leftestQRD) && !digitalRead(leftQRD) && !digitalRead(rightQRD) && !digitalRead(rightestQRD)) {
      // Misaligned quite right
      right = true;
      left = false;
  
      error = 5;
      Serial.println("quite right");
      splitting = false;

    } else if (left) {
      // Misaligned left
      error = -8;
      Serial.println("v left");
      splitting = false;

    } else if (right) {
      // Misaligned right
      error = 8;
      Serial.println("v right");
      splitting = false;
    
    }

    // // Split conditions
    if (digitalRead(leftSplitQRD) && stayLeft) {
      // If the left split QRD is triggered and we want to stay left, hardcut left
      splitProcedure();

    } else if (digitalRead(rightSplitQRD) && !stayLeft) {
      // If the right split QRD is triggered and we want to stay right, hardcut right
      splitProcedure();

    }

    // Serial.println(error);
    // Serial.println(left);
    // Serial.println("Right split");
    // Serial.println(digitalRead(rightSplitQRD));
    // Serial.println("Rightest");
    // Serial.println(digitalRead(rightestQRD));
    // Serial.println("Right");
    // Serial.println(digitalRead(rightQRD));
    // Serial.println("Left");
    // Serial.println(digitalRead(leftQRD));
    // Serial.println("Leftest");
    // Serial.println(digitalRead(leftestQRD));
    // Serial.println("Left split");
    // Serial.println(digitalRead(leftSplitQRD));
    // delay(1000);
}

float calcDerivative(void) {
  double derivative;
  if (error == lastError) {
    // Same state
    tCurrent = millis() - lastSwitch;
    derivative = 1000.0*deltaError/(tPrev + tCurrent);
  } else {
    // New state
    deltaError = error - lastError;
    tPrev = millis() - lastSwitch;
    lastSwitch = millis();
    derivative = 1000.0*deltaError/tPrev; // tCurrent is 0 at the moment of a state change
    lastError = error;
  }
  // Serial.println(derivative);
  return derivative;
}

// Shift mode by button press
// tuneKp mode changes Kp with potentiometer
// tuneKd mode changes Kd with potentiometer
// freeSpins mode changes QRD threshold with potentiometer
// Default mode (all others false) tries to follow tape
void change_mode(void) {
  pwm_stop(motorL);
  pwm_stop(motorR);
  pwm_stop(motorLrev);
  pwm_stop(motorRrev);

  // Mark moving boolean as false to trigger ramp up when motion is resumed
  moving = false;
  delay(100);

  // Continue if button still pressed after delay (helps combat noise)
  if (!digitalRead(button)) {
    // move to next mode
    if (!tuneKp && !tuneKd && !freeSpins) {
      tuneKp = true;
      tuneKd = false;
      freeSpins = false;
    } else if (tuneKp) {
      tuneKp = false;
      tuneKd = true;
      freeSpins = false;
    } else if (tuneKd) {
      tuneKd = false;
      tuneKp = false;
      freeSpins = true;
    } else if (freeSpins) {
      tuneKp = false;
      tuneKd = false;
      freeSpins = false;
      display.clearDisplay();
      display.setCursor(20,40);
      display.println("ZOOM!");
      display.display();
    }
  }
}

void updatePWMvalue(int max, int min, bool leftMotor) {
  // Change motor PWM values of either left or right wheel
  if (leftMotor) {
    if (PWMleft-correction > min && PWMleft-correction < max) {
      PWMleft -= correction;
    } else if (PWMleft-correction < min) {
      PWMleft = min;
    } else if (PWMleft-correction > max) {
      PWMleft = max;
    }
  } else {
    if (PWMright+correction > min && PWMright+correction < max) {
      PWMright += correction;
    } else if (PWMright+correction < min) {
      PWMright = min;
    } else if (PWMright+correction > max) {
      PWMright = max;
    }
  }

}

// Procedure which is run in every splitting state
void splitProcedure(void) {
  Serial.println("SPLITTING");
  Serial.println(splits);
  // Set state booleans accordingly
  left = !stayLeft;
  right = stayLeft;

  splitting = true;
  splits++;
  // If this is a new split, change splitting boolean, count split, and update lastSplit timestamp
  // if (millis() - lastSplit > 100) {
  //   splitting = true;
  //   splits++;
  //   lastSplit = millis();
  // }

  if (splits >= 3) {
    pwm_stop(motorL);
    pwm_stop(motorR);
    pwm_stop(motorRrev);
    pwm_stop(motorLrev);
    splits = 0;
    tuneKp = true;
  }
  
  // Turn in direction of desired split path until the QRD's sense tape
  if (stayLeft) {
    pwm_stop(motorL);
    pwm_stop(motorRrev);
    pwm_start(motorR, 100000, 500, 150, 1);
    pwm_start(motorLrev, 100000, 500, 250, 1);
    delay(70);

    while (!(digitalRead(leftestQRD) || digitalRead(leftQRD) || digitalRead(rightQRD) || digitalRead(rightestQRD))) {
      delay(10);
    }
    pwm_stop(motorLrev);
    pwm_start(motorL, 100000, 500, 200, 1);
    pwm_start(motorR, 100000, 500, 200, 1);
    error = 0;

  } else {
    pwm_start(motorL, 100000, 500, 500, 1);
    pwm_start(motorR, 100000, 500, 0, 1);
    delay(50);
  }
}

// Procedure which is run in every non splitting state
void noSplitProcedure(void) {
  // If coming from a split state, reset PWM averages
  if (splitting) {
    PWMleft = (maxPWM + minPWM)/2;
    PWMright = (maxPWM + minPWM)/2;
  }

  splitting = false;
}

// Function for aqcuiring a stone from a pillar
// a is the Arm object which controls all motion of the robot arm and claw
// Motion must include outward motion to pillar, upward motion to stone, claw closure,
// deposition maneuvering, and claw openning, and return to default position
void getStone(Arm a) {

}

void resetMotors() {

}
