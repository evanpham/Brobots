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
#define armPin PA10
#define LED PB13

// PWM limits, offRight and offLeft value, and correction value
int correction;
int minPWM = 0;
int maxPWM = 500;
int PWMleft = (maxPWM - minPWM)/2;
int PWMright = (maxPWM - minPWM)/2;
int revSpeed = 40;

// PID gains, tuning pot, increment,  alignment and error variables
float Kp = 90.0;
float Kd = 150.0;
float inc = 1;
int potVal, oldPot;
int error, lastError, deltaError = 0;
bool leftest, left, rightest, right, leftSplit, rightSplit;

// Timers used for derivative calculations
int lastSwitch = millis();
int tPrev, tCurrent;

// Variables for split handling and return marking/timing
bool stayLeft = true;
bool splitting = false;
int lastSplit = millis();
int splits = 0;
int splitGoal = 4;
bool returning = false;
int ETtime = 90000; // Time in milliseconds at which we need to start returning

// Bools for mode tracking/changing
bool tuneKp = false;
bool tuneKd = false;
bool freeSpins = false;

// Bools for QRD logic (remembers how the bot was most recently aligned)
bool offLeft = false;
bool offRight = false;

// Function prototypes
void updatePWMvalue(int max, int min);
void splitProcedure(void);
void noSplitProcedure(void);
void returnSplitProcedure(void);
void gauntletProcedure(void);
void change_mode(void);
float calcDerivative(void);
void updateQRDs(void);
void updateError(void);
void updateMotors(int leftVal, int rightVal);
void hardStop(void);
int readSonar(void);
void getStone(Arm a);
void pivot(void);

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
  pinMode(motorL, INPUT_PULLDOWN);
  pinMode(motorR, INPUT_PULLDOWN);
}

void loop() {
  // Read mode change button
  if (!digitalRead(button)) {
    change_mode();
  }

  if (tuneKp) {
    // Tune Kp mode (Turn off motors)
    pwm_start(motorR, 100000, 500, 0, 1);
    pwm_start(motorL, 100000, 500, 0, 1);
    pwm_start(motorRrev, 100000, 500, 0, 1);
    pwm_start(motorLrev, 100000, 500, 0, 1);

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
    pwm_start(motorR, 100000, 500, 0, 1);
    pwm_start(motorL, 100000, 500, 0, 1);
    pwm_start(motorRrev, 100000, 500, 0, 1);
    pwm_start(motorLrev, 100000, 500, 0, 1);

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
    pwm_start(motorR, 100000, 500, 0, 1);
    pwm_start(motorL, 100000, 500, 0, 1);
    pwm_start(motorRrev, 100000, 500, 0, 1);
    pwm_start(motorLrev, 100000, 500, 0, 1);

    display.clearDisplay();
    display.setCursor(4,40);
    display.print("FREE SPINS");
    display.display();

  } else if (splits >= splitGoal || returning || millis() >= ETtime) {
    // If first time running this go home code, pivot
    if (!returning) {
      pivot();
    }
    returning = true;

    // Follow tape back to gauntlet, counting splits along the way

  } else {
    // Main PID sequence
    updateError();

    correction = Kp*error + Kd*calcDerivative(); // Correction tend to be + when offRight of tape

    // Update PWM values
    updatePWMvalue(maxPWM, minPWM);
   
    // Updates motors with new PWM values
    updateMotors(PWMleft, PWMright);
    
    // Serial.println(correction);
    // Serial.println(PWMleft);
    // Serial.println(PWMright);
    // delay(1000);
  }
}

// Updates QRD readings
// Booleans are 1 when QRD is on tape, and 0 when off
void updateQRDs(void) {
  leftest = digitalRead(leftestQRD);
  left = digitalRead(leftQRD);
  right = digitalRead(rightQRD);
  rightest = digitalRead(rightestQRD);
  leftSplit = digitalRead(leftSplitQRD);
  rightSplit = digitalRead(rightSplitQRD);
}

// Updates error value according to state of QRDs
// Positive errors indicate offRight misalignment
// Negative errors indicate offLeft misalignment
void updateError(void) {
  // Main PID control sequence
  updateQRDs();

  if (!leftest && left && right && !rightest) {
    // Centered
    offLeft = false;
    offRight = false;

    error = 0;
    Serial.println("Centered");
    splitting = false;

  } else if (!leftest && !left && right && !rightest) {
    // Misaligned leftish
    offLeft = true;
    offRight = false;

    error = -1;
    Serial.println("leftish");
    splitting = false;

  } else if (!leftest && left && !right && !rightest) {
    // Misaligned rightish
    offRight = true;
    offLeft = false;

    error = 1;
    Serial.println("rightish");
    splitting = false;

  } else if (!leftest && !left && right && rightest) {
    // Misaligned offLeft
    offLeft = true;
    offRight = false;

    error = -3;
    Serial.println("left");
    splitting = false;

  } else if (leftest && left && !right && !rightest) {
    // Misaligned offRight
    offRight = true;
    offLeft = false;

    error = 3;
    Serial.println("right");
    splitting = false;

  } else if (!leftest && !left && !right && rightest){
    // Misaligned quite offLeft
    offLeft = true;
    offRight = false;
 
    error = -5;
    Serial.println("quite left");
    splitting = false;

  } else if (leftest && !left && !right && !rightest) {
    // Misaligned quite offRight
    offRight = true;
    offLeft = false;
  
    error = 5;
    Serial.println("quite right");
    splitting = false;

  } else if (offLeft) {
    // Misaligned offLeft
    error = -8;
    Serial.println("v left");
    splitting = false;

  } else if (offRight) {
    // Misaligned offRight
    error = 8;
    Serial.println("v right");
    splitting = false;
    
  }

  // Split conditions
  if (leftSplit && stayLeft) {
    // If left split QRD is triggered and we want to stay left, run split procedure
    splitProcedure();

  } else if (rightSplit && !stayLeft) {
    // If right split QRD is triggered and we want to stay right, run split procedure
    splitProcedure();

  } else if (rightSplit && stayLeft && returning) {
    // On return, split handling happens on the opposite side
    returnSplitProcedure();

  } else if (leftSplit && !stayLeft && returning) {
    // On return, split handling happens on the opposite side
    returnSplitProcedure();

  }

  // Serial.println("Right Split");
  // Serial.println(rightSplit);
  // Serial.println("Rightest");
  // Serial.println(rightest);
  // Serial.println("Right");
  // Serial.println(right);
  // Serial.println("Left");
  // Serial.println(left);
  // Serial.println("Leftest");
  // Serial.println(leftest);
  // Serial.println("Left Split");
  // Serial.println(leftSplit);
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
  // Continue if button still pressed after delay (helps combat noise)
  // Also continue if change was triggered by split
  if (!digitalRead(button) || splitting) {
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

// Updates the PWM values for both motors according to a correction value
// Limits PWM values to within max and min parameters
void updatePWMvalue(int max, int min) {
  // Change motor PWM values of offLeft and offRight wheel
  if (PWMleft-correction > min && PWMleft-correction < max) {
    PWMleft -= correction;
  } else if (PWMleft-correction < min) {
    PWMleft = min;
  } else if (PWMleft-correction > max) {
    PWMleft = max;
  }

  if (PWMright+correction > min && PWMright+correction < max) {
    PWMright += correction;
  } else if (PWMright+correction < min) {
    PWMright = min;
  } else if (PWMright+correction > max) {
    PWMright = max;
  }
}

void updateMotors(int leftVal, int rightVal) {
  if (leftVal != 0 && rightVal != 0) {
    pwm_start(motorRrev, 100000, 500, 0, 1);
    pwm_start(motorLrev, 100000, 500, 0, 1);
    pwm_start(motorR, 100000, 500, rightVal, 1);
    pwm_start(motorL, 100000, 500, leftVal, 1);
  } else if (leftVal == 0) {
    pwm_start(motorRrev, 100000, 500, 0, 1);
    pwm_start(motorL, 100000, 500, 0, 1);
    pwm_start(motorR, 100000, 500, rightVal, 1);
    pwm_start(motorLrev, 100000, 500, revSpeed, 1);
  } else if (rightVal == 0) {
    pwm_start(motorR, 100000, 500, 0, 1);
    pwm_start(motorLrev, 100000, 500, 0, 1);
    pwm_start(motorRrev, 100000, 500, revSpeed, 1);
    pwm_start(motorL, 100000, 500, leftVal, 1);
  }
}

// Procedure which is run in every splitting state
void splitProcedure(void) {
  // Set state booleans accordingly (if trying to stay left, misaligned right)
  offLeft = !stayLeft;
  offRight = stayLeft; //THESE MIGHT NOT BE NECESSARY ANYMORE

  // Ensure this is not the same as last split (prevents double counting a split)
  if (millis() - lastSplit > 350) {
    splitting = true;
    splits++;
    lastSplit = millis();
  }

  // Stop if at desired split number
  if (splits >= splitGoal) {
    hardStop();

    splits = 0;
    tuneKp = true;
    return;
  }
  
  // Turn in direction of desired split path until the QRD's sense tape
  // Dont turn after 2nd split. These are pillar markers
  if (stayLeft && splits < 3) {
    pwm_start(motorL, 100000, 500, 0, 1);
    pwm_start(motorRrev, 100000, 500, 0, 1);
    pwm_start(motorR, 100000, 500, 250, 1);
    pwm_start(motorLrev, 100000, 500, 250, 1);
    delay(250);
  } else if (splits < 3) {
    pwm_start(motorR, 100000, 500, 0, 1);
    pwm_start(motorLrev, 100000, 500, 0, 1);
    pwm_start(motorL, 100000, 500, 250, 1);
    pwm_start(motorRrev, 100000, 500, 250, 1);
    delay(250);
  }
  while (!(digitalRead(leftestQRD) || digitalRead(leftQRD) || digitalRead(rightQRD) || digitalRead(rightestQRD))) {
    delay(10); // Keep turning until back on tape
  }

  // Continue forward once tape is found
  pwm_stop(motorLrev);
  pwm_stop(motorRrev);
  pwm_start(motorL, 100000, 500, 100, 1);
  pwm_start(motorR, 100000, 500, 100, 1);
  error = 0;
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

// Procedure run at splits when on return path
void returnSplitProcedure(void) {
  // Ensure this is not the same as last split (prevents double counting a split)
  if (millis() - lastSplit > 350) {
    splitting = true;
    splits--;
    lastSplit = millis();
  }
  
  if (splits == 1) {
    // Second to last split before the gauntlet, need to turn
    if (stayLeft) {
      // If we were staying left on the outward path, need to stay right on return path
      pwm_start(motorR, 100000, 500, 0, 1);
      pwm_start(motorLrev, 100000, 500, 0, 1);
      pwm_start(motorL, 100000, 500, 250, 1);
      pwm_start(motorRrev, 100000, 500, 250, 1);
      delay(250); 
    } else {
      // If we were staying right on outward path, need to stay left on return path
      pwm_start(motorL, 100000, 500, 0, 1);
      pwm_start(motorRrev, 100000, 500, 0, 1);
      pwm_start(motorR, 100000, 500, 250, 1);
      pwm_start(motorLrev, 100000, 500, 250, 1);
      delay(250);
    }
    while (!(digitalRead(leftestQRD) || digitalRead(leftQRD) || digitalRead(rightQRD) || digitalRead(rightestQRD))) {
      delay(10); // Keep turning until back on tape
    }

    // Continue forward once tape is found
    pwm_stop(motorLrev);
    pwm_stop(motorRrev);
    pwm_start(motorL, 100000, 500, 100, 1);
    pwm_start(motorR, 100000, 500, 100, 1);
    error = 0;


  } else if (splits == 0) {
    // At the gauntlet
    gauntletProcedure();
  }
}

void gauntletProcedure(void) {
  // Hard stop (near ledge)
  hardStop();

  // If we stayed left initially,the gauntlet is on the left
  // Otherwise, its on the right
  if (stayLeft) {
    pwm_start(motorL, 100000, 500, 0, 1);
    pwm_start(motorRrev, 100000, 500, 0, 1);
    pwm_start(motorR, 100000, 500, 150, 1);
    pwm_start(motorLrev, 100000, 500, 150, 1);
    delay(250);
  } else {
    pwm_start(motorR, 100000, 500, 0, 1);
    pwm_start(motorLrev, 100000, 500, 0, 1);
    pwm_start(motorL, 100000, 500, 150, 1);
    pwm_start(motorRrev, 100000, 500, 150, 1);
    delay(250);
  }
  while (!(digitalRead(leftestQRD) || digitalRead(leftQRD) || digitalRead(rightQRD) || digitalRead(rightestQRD))) {
    delay(10); // Keep turning until back on tape
  }

  // Continue forward once tape is found
  pwm_stop(motorLrev);
  pwm_stop(motorRrev);
  pwm_start(motorL, 100000, 500, 100, 1);
  pwm_start(motorR, 100000, 500, 100, 1);
  
  // Place holder code to stop at gauntlet. Will  be replaced by limit switch stop
  delay(1000);
  pwm_start(motorL, 100000, 500, 0, 1);
  pwm_start(motorR, 100000, 500, 0, 1);
  pwm_start(motorLrev, 100000, 500, 0, 1);
  pwm_start(motorRrev, 100000, 500, 0, 1);
  tuneKp = true;
}

// Stops bot
void hardStop(void) {
  // Pulse motors backwards according to how fast they are spinning forwards
  pwm_start(motorL, 100000, 500, 0, 1);
  pwm_start(motorR, 100000, 500, 0, 1);
  pwm_start(motorLrev, 100000, 500, PWMleft, 1);
  pwm_start(motorRrev, 100000, 500, PWMright, 1);
  delay(100);
  pwm_start(motorLrev, 100000, 500, 0, 1);
  pwm_start(motorRrev, 100000, 500, 0, 1);
}

// Function for aqcuiring a stone from a pillar
// a is the Arm object which controls all motion of the robot arm and claw
// Motion must include outward motion to pillar, upward motion to stone, claw closure,
// deposition maneuvering, and claw openning, and return to default position
void getStone(Arm a) {

}

// Turns the robot around on tape line
void pivot(void) {
 //This is possibly necessary to go forward a little before pivoting
  // pwm_start(motorL, 100000, 500, 150, 1);
  // pwm_start(motorR, 100000, 500, 150, 1);
  // delay(100);
  // pwm_start(motorL, 100000, 500, 0, 1);
  // pwm_start(motorR, 100000, 500, 0, 1);

  if (stayLeft) {
    pwm_start(motorL, 100000, 500, 150, 1);
    pwm_start(motorRrev, 100000, 500, 150, 1);
    delay(200);
    while (!(digitalRead(leftestQRD) || digitalRead(leftQRD) || digitalRead(rightQRD) || digitalRead(rightestQRD))) {
      delay(10);
    }
    pwm_start(motorL, 100000, 500, 0, 1);
    pwm_start(motorRrev, 100000, 500, 0, 1);
  } else {
    pwm_start(motorLrev, 100000, 500, 150, 1);
    pwm_start(motorR, 100000, 500, 150, 1);
    delay(200);
    while (!(digitalRead(leftestQRD) || digitalRead(leftQRD) || digitalRead(rightQRD) || digitalRead(rightestQRD))) {
      delay(10);
    }
    pwm_start(motorLrev, 100000, 500, 0, 1);
    pwm_start(motorR, 100000, 500, 0, 1);
  }
}