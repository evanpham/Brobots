#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <FreeMono9pt7b.h>
#include <time.h>

// #if (SSD1306_LCDHEIGHT != 64)
// #error("Height incorrect, please fix Adafruit_SSD1306.h!");
// #endif

#define OLED_RESET -1  // Not used
Adafruit_SSD1306 display(OLED_RESET);

#define pot PA2
#define button PA12
#define motorL PA0
#define motorR PB0
#define leftQRD PB10
#define leftestQRD PB11
#define rightQRD PA4
#define rightestQRD PA3
#define trig PB14
#define echo PB13

int potVal, oldPot, correction;
int minPWM = 0;
int maxPWM = 400;
int PWMleft = (maxPWM - minPWM)/2;
int PWMright = (maxPWM - minPWM)/2;

float Kp = 65.0;
float Kd = 65.0;

int last = millis(); // Timer used to limit mode changes

// Timers used for derivative calculations
int lastSwitch = millis();
int tPrev, tCurrent;

// Bools for split handling
bool stayLeft = false;
bool splitting = false;

// Bools for mode tracking/changing
bool tuneKp = false;
bool tuneKd = false;
bool freeSpins = false;
bool moving = false;

// Bools
bool left = false;
bool right = false;


int error, lastError, deltaError = 0;
float inc = 1;

void change_mode(void);
float calcDerivative(void);
void updateError(void);
int readSonar(void);

void setup() {
  Serial.begin(9600);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  // init done

  // Draw a test
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
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  // attachInterrupt(digitalPinToInterrupt(button), change_mode, LOW);

}

void loop() {
  // Read mode change button
  if (!digitalRead(button)) {
    change_mode();
  }

  if (tuneKp) {
    // Tune Kp mode (Turn off motors)
    pwm_stop(PA_0);
    pwm_stop(PB_0);

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
    pwm_stop(PA_0);
    pwm_stop(PB_0);

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
    pwm_stop(PA_0);
    pwm_stop(PB_0);

    display.clearDisplay();
    display.setCursor(4,40);
    display.print("FREE SPINS");
    display.display();

    } else {
    // If not moving, ramp up speed
    if (!moving) {
      for (int PWMval = 0; PWMval < PWMleft; PWMval++) {
        pwm_start(PB_0, 100000, 500, PWMval, 1);
        pwm_start(PA_0, 100000, 500, PWMval, 1);
        delay(2);
      }
      moving = true;
    }

    // Main PID sequence
    updateError();

    correction = Kp*error + Kd*calcDerivative(); // Correction tend to be + when right of tape

    // Change motor PWM values
    if (PWMleft-correction > minPWM && PWMleft-correction < maxPWM) {
      PWMleft -= correction;
    } else if (PWMleft-correction < minPWM) {
      PWMleft = minPWM;
    } else if (PWMleft-correction > maxPWM) {
      PWMleft = maxPWM;
    }
    if (PWMright+correction > minPWM && PWMright+correction < maxPWM) {
      PWMright += correction;
    } else if (PWMright+correction < minPWM) {
      PWMright = minPWM;
    } else if (PWMright+correction > maxPWM) {
      PWMright = maxPWM;
    }

    pwm_start(PB_0, 100000, 500, PWMright, 1);
    pwm_start(PA_0, 100000, 500, PWMleft, 1);
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

    } else if (!digitalRead(leftestQRD) && !digitalRead(leftQRD) && digitalRead(rightQRD) && !digitalRead(rightestQRD)) {
      // Misaligned leftish
      left = true;
      right = false;
      error = -1;
      Serial.println("leftish");
      
    } else if (!digitalRead(leftestQRD) && digitalRead(leftQRD) && !digitalRead(rightQRD) && !digitalRead(rightestQRD)) {
      // Misaligned rightish
      right = true;
      left = false;
      error = 1;
      Serial.println("rightish");
      
    } else if (!digitalRead(leftestQRD) && !digitalRead(leftQRD) && digitalRead(rightQRD) && digitalRead(rightestQRD)) {
      // Misaligned left
      left = true;
      right = false;
      error = -3;
      Serial.println("left");

    } else if (digitalRead(leftestQRD) && digitalRead(leftQRD) && !digitalRead(rightQRD)&& !digitalRead(rightestQRD)) {
      // Misaligned right
      right = true;
      left = false;
      error = 3;
      Serial.println("right");

    } else if (!digitalRead(leftestQRD) && !digitalRead(leftQRD) && !digitalRead(rightQRD) && digitalRead(rightestQRD)){
      // Misaligned quite left
      left = true;
      right = false;
      error = -5;
      Serial.println("quite left");

    } else if (digitalRead(leftestQRD) && !digitalRead(leftQRD) && !digitalRead(rightQRD) && !digitalRead(rightestQRD)) {
      // Misaligned quite right
      right = true;
      left = false;
      error = 5;
      Serial.println("quite right");

    } else if (left) {
      // Very misaligned left
      error = -8;
      Serial.println("V left");
      
    } else if (right) {
      // Very misaligned right
      error = 8;
      Serial.println("V right");

    }

    // Split conditions
    if (digitalRead(leftestQRD) && digitalRead(leftQRD) && digitalRead(rightQRD) && digitalRead(rightestQRD)) {
      // If staying left, right. If staying right, left
      error = stayLeft ? 5 : -5;
      left = !stayLeft;
      right = stayLeft;
      splitting = true;
    }
    if (digitalRead(leftestQRD) && digitalRead(leftQRD) && digitalRead(rightQRD) && !digitalRead(rightestQRD)) {
        //bool?true:false
      // If staying left, either right or quite right. If staying right, either centered or leftish
      error = stayLeft ? 6 : -2;
      left = !stayLeft;
      right = stayLeft;
      splitting = true;
    } else if (!digitalRead(leftestQRD) && digitalRead(leftQRD) && digitalRead(rightQRD) && digitalRead(rightestQRD)) {
      // If staying left, either centered of rightish. If staying right, either left or quite left
      error = stayLeft ? 2 : -6;
      left = !stayLeft;
      right = stayLeft;
      splitting = true;
    } else if (digitalRead(leftestQRD) && !digitalRead(leftQRD) && digitalRead(rightQRD) && !digitalRead(rightestQRD)) {
      // If staying left, quite right. If staying right, leftish
      error = stayLeft ? 7 : -2;
      left = !stayLeft;
      right = stayLeft;
      splitting = true;
    } else if (!digitalRead(leftestQRD) && digitalRead(leftQRD) && !digitalRead(rightQRD) && digitalRead(rightestQRD)) {
      // If staying left, rightish. If staying right, quite left
      error = stayLeft ? 2 : -7;
      left = !stayLeft;
      right = stayLeft;
      splitting = true;
    } else if (digitalRead(leftestQRD) && !digitalRead(leftQRD) && digitalRead(rightQRD) && digitalRead(rightestQRD)) {
      // If staying left, quite right. If staying right, left
      error = stayLeft ? 7 : -5;
      left = !stayLeft;
      right = stayLeft;
      splitting = true;
    } else if (digitalRead(leftestQRD) && digitalRead(leftQRD) && !digitalRead(rightQRD) && digitalRead(rightestQRD)) {
      // If staying left, right. If staying right, quite left
      error = stayLeft ? 5 : -7;
      left = !stayLeft;
      right = stayLeft;
      splitting = true;
    } else if (digitalRead(leftestQRD) && !digitalRead(leftQRD) && !digitalRead(rightQRD) && digitalRead(rightestQRD)) {
      // If staying left, quite right. If staying right, quite left
      error = stayLeft ? 7 : -7;
      left = !stayLeft;
      right = stayLeft;
      splitting = true;
    }

    // Serial.println(error);
    // Serial.println(left);
    // Serial.println("Rightest");
    // Serial.println(digitalRead(rightestQRD));

    // Serial.println("Right");
    // Serial.println(digitalRead(rightQRD));

    // Serial.println("Left");
    // Serial.println(digitalRead(leftQRD));

    // Serial.println("Leftest");
    // Serial.println(digitalRead(leftestQRD));
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

//shift mode by button press
//tuneKp mode changes Kp with potentiometer
//tuneKd mode changes Kd with potentiometer
//freeSpins mode changes QRD threshold with potentiometer
//default mode (both false) tries to follow tape
void change_mode(void) {
  moving = false;
  delay(100);
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
    last = millis();
  }
}
