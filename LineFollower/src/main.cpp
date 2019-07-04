#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <FreeMono12pt7b.h>
#include <time.h>

// #if (SSD1306_LCDHEIGHT != 64)
// #error("Height incorrect, please fix Adafruit_SSD1306.h!");
// #endif

#define OLED_RESET -1  // Not used
Adafruit_SSD1306 display(OLED_RESET);

#define pot PA2
#define button PB12
#define motorL PA0
#define motorR PB0
#define leftQRD PB10
#define leftestQRD PB11
#define rightQRD PA4
#define rightestQRD PA3

int potVal, oldPot, correction;
int PWMleft = 300;
int PWMright = 300;
float Kp = 40;
float Kd = 120;
int lastSwitch = millis();
int tPrev, tCurrent;
bool tuneKp = false;
bool tuneKd = false;
bool freeSpins = false;
bool left = false;
bool right = false;


int error, lastError, deltaError = 0;
float inc = 1;

void change_mode(void);
int calcDerivative(void);
void updateError(void);

void setup() {
  Serial.begin(9600);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  // init done

  // Draw a test
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setFont(&FreeMono12pt7b);
  display.setCursor(6,30);
  display.println("Jarvis Booting Up...");
  display.display();

  pinMode(button, INPUT_PULLUP);
  pinMode(leftQRD, INPUT_PULLUP);
  pinMode(rightQRD, INPUT_PULLUP);
  pinMode(leftestQRD, INPUT_PULLUP);
  pinMode(rightestQRD, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button), change_mode, FALLING);

}

void loop() {
  oldPot = potVal;

  if (tuneKp) {
    // Tune Kp mode (Turn off motors)
    pwm_stop(PA_0);
    pwm_stop(PB_0);

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
    // Main PID sequence
    updateError();

    correction = Kp*error + Kd*calcDerivative(); // Correction tend to be + when right of tape

    // Change motor PWM values
    if (PWMleft-correction > 100 && PWMleft-correction < 500) {
      PWMleft -= correction;
    } else if (PWMleft-correction < 100) {
      PWMleft = 100;
    } else if (PWMleft-correction > 500) {
      PWMleft = 500;
    }
    if (PWMright+correction > 100 && PWMright+correction < 500) {
      PWMright += correction;
    } else if (PWMright+correction < 100) {
      PWMright = 100;
    } else if (PWMright+correction > 500) {
      PWMright = 500;
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

int calcDerivative(void) {
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
  Serial.println("YO");

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
  }
}