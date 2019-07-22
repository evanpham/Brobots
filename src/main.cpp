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
#define motorL PA_0
#define motorR PB_0
#define leftQRD PB10
#define leftestQRD PB11
#define rightQRD PA4
#define rightestQRD PA3
#define leftSplitQRD PB3
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
int maxPWM = 500;
int PWMleft = (maxPWM - minPWM)/2;
int PWMright = (maxPWM - minPWM)/2;
int splits = 0;

// PID gains, tuning pot, increment, and error variables
float Kp = 85.0;
float Kd = 140.0;
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
void updatePWMvalue(int max, int min, bool left);
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

    // If not splitting, update left and right PWM value around same average value
    // If splitting, update left and right PWM values around different average values
    // which favor turning direction
    if (!splitting) {
      updatePWMvalue(maxPWM, minPWM, left = true);
      updatePWMvalue(maxPWM, minPWM, left = false);
    } else if (stayLeft) {
      updatePWMvalue(maxPWM - 400, minPWM, left = true); // avg left PWM < avg right PWM
      updatePWMvalue(maxPWM, minPWM + 400, left = false); // will turn bot left
    } else {
      updatePWMvalue(maxPWM, minPWM + 400, left = true); // avg right PWM < avg left PWM
      updatePWMvalue(maxPWM - 400, minPWM, left = false); // will turn bot right
    }
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
    if (!digitalRead(leftestQRD) && !digitalRead(leftQRD) && digitalRead(rightQRD) && !digitalRead(rightestQRD)) {
      // Misaligned leftish
      left = false;
      center = true;
      right = false;

      error = -1;
      Serial.println("leftish");
      noSplitProcedure();

    } else if (!digitalRead(leftestQRD) && digitalRead(leftQRD) && !digitalRead(rightQRD) && !digitalRead(rightestQRD)) {
      // Misaligned rightish
      right = false;
      center = true;
      left = false;

      error = 1;
      Serial.println("rightish");
      noSplitProcedure();

    } else if (!digitalRead(leftestQRD) && !digitalRead(leftQRD) && digitalRead(rightQRD) && digitalRead(rightestQRD)) {
      // Misaligned left
      left = true;
      center = false;
      right = false;

      error = -3;
      Serial.println("left");
      noSplitProcedure();

    } else if (digitalRead(leftestQRD) && digitalRead(leftQRD) && !digitalRead(rightQRD) && !digitalRead(rightestQRD)) {
      // Misaligned right
      left = false;
      center = false;
      right = true;

      error = 3;
      Serial.println("right");
      noSplitProcedure();

    } else if (!digitalRead(leftestQRD) && !digitalRead(leftQRD) && !digitalRead(rightQRD) && digitalRead(rightestQRD)){
      // Misaligned quite left
      left = true;
      center = false;
      right = false;
  
      error = -5;
      Serial.println("quite left");
      noSplitProcedure();

    } else if (digitalRead(leftestQRD) && !digitalRead(leftQRD) && !digitalRead(rightQRD) && !digitalRead(rightestQRD)) {
      // Misaligned quite right
      left = false;
      center = false;
      right = true;
      
  
      error = 5;
      Serial.println("quite right");
      noSplitProcedure();

    } else if (left) {
      // Misaligned very left
      error = -8;
      Serial.println("v left");
      noSplitProcedure();

    } else if (center) {
      // Centered
      error = 0;
      Serial.println("Centered");
      noSplitProcedure();

    } else if (right) {
      // Misaligned very right
      error = 8;
      Serial.println("v right");
      noSplitProcedure();
    
    }

    // Split conditions
    if (digitalRead(leftestQRD) && digitalRead(leftQRD) && digitalRead(rightQRD) && digitalRead(rightestQRD)) {
      // If staying left, right. If staying right, left
      error = stayLeft ? 11 : -11;
      splitProcedure();

    }
    if (digitalRead(leftestQRD) && digitalRead(leftQRD) && digitalRead(rightQRD) && !digitalRead(rightestQRD)) {
        //bool?true:false
      // If staying left, either right or quite right. If staying right, either centered or leftish
      error = stayLeft ? 9 : -7;
      splitProcedure();

    } else if (!digitalRead(leftestQRD) && digitalRead(leftQRD) && digitalRead(rightQRD) && digitalRead(rightestQRD)) {
      // If staying left, either centered or rightish. If staying right, either left or quite left
      error = stayLeft ? 7 : -9;
      splitProcedure();

    } else if (digitalRead(leftestQRD) && !digitalRead(leftQRD) && digitalRead(rightQRD) && !digitalRead(rightestQRD)) {
      // If staying left, quite right. If staying right, leftish
      error = stayLeft ? 10 : -7;
      splitProcedure();

    } else if (!digitalRead(leftestQRD) && digitalRead(leftQRD) && !digitalRead(rightQRD) && digitalRead(rightestQRD)) {
      // If staying left, rightish. If staying right, quite left
      error = stayLeft ? 7 : -10;
      splitProcedure();

    } else if (digitalRead(leftestQRD) && !digitalRead(leftQRD) && digitalRead(rightQRD) && digitalRead(rightestQRD)) {
      // If staying left, quite right. If staying right, left
      error = stayLeft ? 14 : -11;
      splitProcedure();

    } else if (digitalRead(leftestQRD) && digitalRead(leftQRD) && !digitalRead(rightQRD) && digitalRead(rightestQRD)) {
      // If staying left, right. If staying right, quite left
      error = stayLeft ? 11 : -14;
      splitProcedure();

    } else if (digitalRead(leftestQRD) && !digitalRead(leftQRD) && !digitalRead(rightQRD) && digitalRead(rightestQRD)) {
      // If staying left, quite right. If staying right, quite left
      error = stayLeft ? 15 : -15;
      splitProcedure();

    } else if (digitalRead(leftSplitQRD) && stayLeft) {
      // If the left split QRD is triggered and we want to stay left, hardcut left
      PWMleft = 0;
      PWMright = 500;
      splitProcedure();

    } else if (digitalRead(rightSplitQRD) && !stayLeft) {
      // If the right split QRD is triggered and we want to stay right, hardcut right
      PWMleft = 500;
      PWMright = 0;
      splitProcedure();

    }

    // Serial.println(error);
    // Serial.println(left);
    //Serial.println("Right split");
    // Serial.println(digitalRead(rightSplitQRD));

    // Serial.println("Rightest");
    // Serial.println(digitalRead(rightestQRD));

    // Serial.println("Right");
    // Serial.println(digitalRead(rightQRD));

    // Serial.println("Left");
    // Serial.println(digitalRead(leftQRD));

    // Serial.println("Leftest");
    // Serial.println(digitalRead(leftestQRD));

    //Serial.println("Left split");
    // Serial.println(digitalRead(leftSplitQRD));
    // delay(1000);
  
    if (splits >= 2) {
      pwm_stop(motorL);
      pwm_stop(motorR);
      tuneKp = true;
      splits = 0;
    }

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

void updatePWMvalue(int max, int min, bool left) {
  // Change motor PWM values of either left or right wheel
  if (left) {
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
  // Set state booleans accordingly
  left = !stayLeft;
  right = stayLeft;

  // If this is a new split, change splitting boolean, count split, and update lastSplit timestamp
  if (millis() - lastSplit > 100) {
    splitting = true;
    splits++;
    lastSplit = millis();
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
