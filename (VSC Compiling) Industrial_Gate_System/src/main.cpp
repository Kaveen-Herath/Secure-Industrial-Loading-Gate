#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <soc/gpio_struct.h>

#define POT_PIN 34
#define SERVO_PIN 13
#define PIR_PIN 27
#define TRIG_PIN 5
#define ECHO_PIN 18
#define BUZZER_PIN 26
#define RED_LED 14
#define MODE_BUTTON 32

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

//OBJECT CREATION
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Servo gateServo;

//VARIABLES
//5 SECOND HOLD BEFORE AUTOMATIC CLOSING
bool gateOpen = false;
unsigned long noMotionStart = 0; //SAVES WHEN THE MOTION IS STOPPED
const unsigned long HOLD_OPEN_TIME = 5000;

//AUTO, MANUAL BTN
bool manualMode = false;
bool lastButtonState = HIGH;

unsigned long lastButtonPress = 0;
const unsigned long DEBOUNCE_TIME = 200;

//SERVO ANGLE
int gateAngle = 0;
bool gateOpening = false;

unsigned long lastGateMove = 0;
const unsigned long OPEN_STEP_TIME = 20; //1 DEGRRE EVERY 20 MILLIS

bool gateClosing = false;
unsigned long lastCloseMove = 0;

//SATEFTY MODE
bool safetyActive = false;
bool safetyReopening = false; 

//BUZZER TIMER
unsigned long safetyStartTime = 0;
unsigned long lastSafetyMove = 0;

const unsigned long BUZZER_TIME = 1000;
const unsigned long SAFETY_OPEN_STEP = 20;

hw_timer_t *monitorTimer = NULL;

volatile bool monitorUpdateReady = false;

// NON-BLOCKING ULTRASONIC
volatile unsigned long echoStartTime = 0;
volatile unsigned long echoDuration = 0;
volatile bool echoComplete = false;

bool ultrasonicTriggerActive = false;
bool ultrasonicWaitingForEcho = false;

unsigned long ultrasonicTriggerStart = 0;
unsigned long ultrasonicWaitStart = 0;
unsigned long lastUltrasonicTrigger = 0;

const unsigned long ULTRASONIC_INTERVAL = 60000;     // 60 ms
const unsigned long ULTRASONIC_TRIGGER_TIME = 10;    // 10 us
const unsigned long ULTRASONIC_TIMEOUT = 30000;      // 30 ms

int distance = 0;

void showGateScreen(const char* message, const char* gateStatus) {

  display.clearDisplay();
  display.setCursor(0, 0);

  if (manualMode == true) {
    display.println("MANUAL MODE");
  }
  else {
    display.println("AUTO MODE");
  }

  display.println("");

  if (message != nullptr) {
    display.println(message);
  }

  display.print("Gate: ");
  display.println(gateStatus);

  display.display();
}

void updateStatusDisplay(int potValue) {

  display.clearDisplay();
  display.setCursor(0, 0);

  if (manualMode == true) {

    display.println("System in");
    display.println("MANUAL MODE");
    display.println("");

  }
  else {

    display.println("System in");
    display.println("AUTO MODE");
    display.println("");

    display.print("Distance: ");
    display.print(distance);
    display.println(" cm");

    display.print("Pot ADC: ");
    display.println(potValue);
  }

  // GATE STATUS
  if (gateOpening == true) {
    display.println("Gate: OPENING");
  }
  else if (gateClosing == true) {
    display.println("Gate: CLOSING");
  }
  else if (gateOpen == true) {
    display.println("Gate: OPEN");
  }
  else {
    display.println("Gate: CLOSED");
  }

  if (manualMode == true) {
    display.println("Use O / C");
  }

  display.display();
}

void showModeScreen() {

  display.clearDisplay();
  display.setCursor(0, 0);

  if (manualMode == true) {
    display.println("MANUAL MODE");
    display.println("");
    display.println("Auto control OFF");
  }
  else {
    display.println("AUTO MODE");
    display.println("");
    display.println("Auto control ON");
  }

  display.display();
}

void showSafetyAlert() {

  display.clearDisplay();
  display.setCursor(0, 0);

  display.println("SAFETY ALERT!");
  display.println("");
  display.println("OBSTRUCTION");
  display.println("Gate STOPPED");

  display.display();
}


void showSafetyReopening() {

  display.clearDisplay();
  display.setCursor(0, 0);

  display.println("SAFETY MODE");
  display.println("");
  display.println("Gate REOPENING");

  display.display();
}


void showSafetyOpen() {

  display.clearDisplay();
  display.setCursor(0, 0);

  display.println("SAFETY MODE");
  display.println("");
  display.println("Gate: OPEN");
  display.println("Clear obstruction");

  display.display();
}


void openGateSlowly() {

  Serial.println("Gate OPENING");

  gateOpening = true;
  lastGateMove = millis();
}

void startClosingGate() {
  Serial.println("Gate CLOSING");
  gateClosing = true;

  gateAngle = 90;
  lastCloseMove = millis();

  showGateScreen(nullptr, "CLOSING");
}

void safetyRecovery() {

  gateClosing = false;
  safetyActive = true;

  Serial.println("SAFETY ALERT - OBSTRUCTION DETECTED");
  Serial.println("Gate STOPPED");

  showSafetyAlert();

  tone(BUZZER_PIN, 2000);
  safetyStartTime = millis();
}

void IRAM_ATTR onMonitorTimer() {
  monitorUpdateReady = true;
}

void IRAM_ATTR onEchoChange() {

  if (digitalRead(ECHO_PIN) == HIGH) {

    // Echo pulse started
    echoStartTime = micros();

  }
  else {

    // Echo pulse ended
    if (echoStartTime != 0) {

      echoDuration = micros() - echoStartTime;
      echoComplete = true;
      echoStartTime = 0;
    }
  }
}

void updateUltrasonic() {

  unsigned long currentTime = micros();

  // FINISH 10 us TRIGGER PULSE
  if (ultrasonicTriggerActive == true &&
      currentTime - ultrasonicTriggerStart >= ULTRASONIC_TRIGGER_TIME) {

    digitalWrite(TRIG_PIN, LOW);

    ultrasonicTriggerActive = false;
    ultrasonicWaitStart = currentTime;
  }

  // ECHO MEASUREMENT COMPLETE
  if (echoComplete == true) {

    noInterrupts();

    unsigned long duration = echoDuration;
    echoComplete = false;

    interrupts();

    distance = duration * 0.034 / 2;

    ultrasonicWaitingForEcho = false;
  }

  // ECHO TIMEOUT
  if (ultrasonicWaitingForEcho == true &&
      currentTime - ultrasonicWaitStart >= ULTRASONIC_TIMEOUT) {

    ultrasonicWaitingForEcho = false;
    distance = 0;
  }

  // START NEW MEASUREMENT
  if (ultrasonicTriggerActive == false &&
      ultrasonicWaitingForEcho == false &&
      currentTime - lastUltrasonicTrigger >= ULTRASONIC_INTERVAL) {

    lastUltrasonicTrigger = currentTime;

    digitalWrite(TRIG_PIN, HIGH);

    ultrasonicTriggerStart = currentTime;
    ultrasonicWaitStart = currentTime;

    ultrasonicTriggerActive = true;
    ultrasonicWaitingForEcho = true;
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  attachInterrupt(
  digitalPinToInterrupt(ECHO_PIN),
  onEchoChange,
  CHANGE
  );

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(MODE_BUTTON, INPUT_PULLUP);

  gateServo.attach(SERVO_PIN);
  gateServo.write(0);

  // OLED STARTUP
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED allocation failed");
    for (;;);
  }

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Industrial Gate");
  display.println("System Started");
  display.println("");
  display.println("Gate: CLOSED");
  display.display();

  Serial.println("Industrial Gate System Started");
  Serial.println("Waiting for vehicle...");
  Serial.println("O = Open Gate");
  Serial.println("C = Close Gate");

// HARDWARE TIMER
monitorTimer = timerBegin(0, 80, true);
timerAttachInterrupt(monitorTimer, &onMonitorTimer, true);
timerAlarmWrite(monitorTimer, 500000, true);
timerAlarmEnable(monitorTimer);
}

void loop() {
  int potValue = analogRead(POT_PIN);
  int pirState = digitalRead(PIR_PIN);
  int buttonState = digitalRead(MODE_BUTTON);

    if (buttonState == LOW &&
      lastButtonState == HIGH &&
      millis() - lastButtonPress >= DEBOUNCE_TIME) {

    lastButtonPress = millis();

      manualMode = !manualMode;

      // Cancels any previous automatic closing timer
      noMotionStart = 0;

      if (manualMode == true) {
        GPIO.out_w1tc = (1UL << RED_LED);

        Serial.println("MANUAL MODE Activated");
        showModeScreen();
      }

      else {
        Serial.println("AUTO MODE Activated");
        showModeScreen();
      }
    }

    lastButtonState = buttonState;

// NON-BLOCKING ULTRASONIC
  updateUltrasonic();

  // SERIAL OUTPUT
  if (monitorUpdateReady == true && safetyActive == false) {
    monitorUpdateReady = false;

  updateStatusDisplay(potValue);
}

// NON-BLOCKING GATE OPENING
if (gateOpening == true) {
  if (millis() - lastGateMove >= OPEN_STEP_TIME) {

    lastGateMove = millis();

    if (gateAngle < 90) {
      gateAngle++;
      gateServo.write(gateAngle);
    }

    else {
      gateOpening = false;
      gateOpen = true;

      Serial.println("Gate OPEN");

      showGateScreen(nullptr, "OPEN");
    }
  }
}

// NON-BLOCKING GATE CLOSING
if (gateClosing == true) {

  int closingDelay = map(potValue, 0, 4095, 60, 10);

  if (millis() - lastCloseMove >= closingDelay) {

    lastCloseMove = millis();

    if (distance > 0 && distance < 20) {
      safetyRecovery();
    }

    else if (gateAngle > 0) {
      gateAngle--;
      gateServo.write(gateAngle);
    }

    else {
      gateClosing = false;
      gateOpen = false;
      gateAngle = 0;

      Serial.println("Gate CLOSED");
    }
  }
}

// SAFETY BUZZER TIMER
if (safetyActive == true && safetyReopening == false) {

  if (millis() - safetyStartTime >= BUZZER_TIME) {

    noTone(BUZZER_PIN);

    gateServo.detach();
    gateServo.attach(SERVO_PIN);

    gateServo.write(gateAngle);

    safetyReopening = true;
    lastSafetyMove = millis();

    Serial.println("Safety Recovery - Gate REOPENING");

    showSafetyReopening();
  }
}


// NON-BLOCKING SAFETY REOPENING
if (safetyReopening == true) {

  if (millis() - lastSafetyMove >= SAFETY_OPEN_STEP) {

    lastSafetyMove = millis();

    if (gateAngle < 90) {

      gateAngle++;
      gateServo.write(gateAngle);
    }

    else {
      safetyReopening = false;
      safetyActive = false;

      gateOpen = true;
      gateAngle = 90;
      noMotionStart = 0;

      Serial.println("Gate OPEN - Waiting for clear path");

      showSafetyOpen();
    }
  }
}

  //PIR AUTOMATIC OPENING
  if (manualMode == false && safetyActive == false) {
    if (pirState == HIGH) {
      GPIO.out_w1ts = (1UL << RED_LED);

      noMotionStart = 0;

      if (gateOpen == false && gateOpening == false) {
      Serial.println("Vehicle Detected - Gate OPEN");

      showGateScreen("Vehicle Detected", "OPENING");

      openGateSlowly();
    }
  }

    else {
      GPIO.out_w1tc = (1UL << RED_LED);

      if (gateOpen == true && gateClosing == false) {
        if (noMotionStart == 0) {
          noMotionStart = millis();
          Serial.println("Motion ended - Hold timer started");
        }

        if (millis() - noMotionStart >= HOLD_OPEN_TIME) {
          Serial.println("Hold time complete - Gate ready to close");

          // Check whether ultrasonic path is clear

        if (distance >= 20) {

          Serial.println("Path CLEAR - Automatic closing");

          noMotionStart = 0;

          startClosingGate();
        }

        else {

          Serial.println("Path BLOCKED - Gate staying OPEN");

          noMotionStart = millis();
        }
      }
    }
  }
}

    // MANUAL SERIAL CONTROL
    if (Serial.available() > 0) {
      char command = Serial.read();

      // ONLY ALLOW O AND C IN MANUAL MODE
      if (manualMode == true) {

        // OPEN GATE
        if (command == 'O' || command == 'o') {

          if (gateOpen == false && gateOpening == false) {

            showGateScreen(nullptr, "OPENING");

            openGateSlowly();
          }

          else {
            Serial.println("Gate is already OPEN");
          }
        }

        // CLOSE GATE
        else if (command == 'C' || command == 'c') {

          if (gateOpen == true && gateClosing == false) {
            startClosingGate();
          }

          else {
            Serial.println("Gate is already CLOSED");
          }
        }
      }
  }
}