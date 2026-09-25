/**
 * @file Secure_Industrial_Loading_Gate.cpp
 * @brief IoT-based secure industrial loading gate control system.
 *
 * @details
 * This project implements an automated industrial loading gate using
 * an ESP32 microcontroller.
 *
 * The system uses:
 * - PIR sensor for vehicle/motion detection
 * - Ultrasonic sensor for obstruction detection
 * - Servo motor for gate movement
 * - Potentiometer for manual closing-speed adjustment
 * - OLED display for system status
 * - Buzzer for safety alerts
 * - Push button for Auto/Manual mode selection
 * - UART commands for manual gate control
 *
 * The system provides Automatic, Manual and Safety operation modes.
 *
 * @author Kaveen
 * @date 25 September 2026
 */

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

// OLED display object using the I2C communication interface
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Servo object used to control the gate mechanism
Servo gateServo;

// ==================== SYSTEM STATE VARIABLES ====================
// Stores whether the gate is currently fully open
bool gateOpen = false;

// Stores the time when PIR motion stops
unsigned long noMotionStart = 0;

// Time the gate remains open after motion stops
const unsigned long HOLD_OPEN_TIME = 5000;

//AUTO, MANUAL BTN
bool manualMode = false;
bool lastButtonState = HIGH; // Used to detect a new btn press

unsigned long lastButtonPress = 0;
// Prevents rapid repeated mode switching caused by button bouncing
const unsigned long DEBOUNCE_TIME = 200;

// ==================== GATE MOVEMENT VARIABLES ====================
int gateAngle = 0;
bool gateOpening = false;

unsigned long lastGateMove = 0;
const unsigned long OPEN_STEP_TIME = 20; //1 DEGRRE EVERY 20 MILLIS

bool gateClosing = false;
unsigned long lastCloseMove = 0;

// ==================== SAFETY MODE VARIABLES ====================
bool safetyActive = false;
bool safetyReopening = false; 

//BUZZER TIMER
unsigned long safetyStartTime = 0;
unsigned long lastSafetyMove = 0;

const unsigned long BUZZER_TIME = 1000;
const unsigned long SAFETY_OPEN_STEP = 20;

// ESP32 hardware timer used to periodically update system status
hw_timer_t *monitorTimer = NULL;

// Flag set by the hardware timer interrupt when a status update is required
volatile bool monitorUpdateReady = false;

// ==================== NON-BLOCKING ULTRASONIC VARIABLES ====================
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

/** 
* @brief Displays the current gate status on the OLED screen. 
* 
* Displays the current operating mode, an optional message, 
* and the current gate status. 
* 
* @param message Optional message displayed on the OLED. 
* @param gateStatus Current gate status such as OPEN, OPENING or CLOSING. 
* @return None. 
*/

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

/** 
* @brief Updates the OLED with live system information. 
* 
* Displays the operating mode, ultrasonic distance, 
* potentiometer ADC value and current gate status. 
* 
* @param potValue Current ADC value read from the potentiometer. 
* @return None. 
*/

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

/** 
* @brief Displays the currently selected operating mode. 
* 
* Shows whether the system is operating in Automatic Mode 
* or Manual Mode. 
* 
* @param None. 
* @return None. 
*/

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

/** 
* @brief Displays the safety obstruction warning. 
* 
* Informs the operator that an obstruction has been detected 
* and that the gate has been stopped. 
* 
* @param None. 
* @return None. 
*/

void showSafetyAlert() {

  display.clearDisplay();
  display.setCursor(0, 0);

  display.println("SAFETY ALERT!");
  display.println("");
  display.println("OBSTRUCTION");
  display.println("Gate STOPPED");

  display.display();
}

/** 
* @brief Displays the safety recovery status. 
* 
* Informs the operator that the gate is automatically 
* reopening after an obstruction was detected. 
* 
* @param None. 
* @return None. 
*/

void showSafetyReopening() {

  display.clearDisplay();
  display.setCursor(0, 0);

  display.println("SAFETY MODE");
  display.println("");
  display.println("Gate REOPENING");

  display.display();
}

/** 
* @brief Displays the final safety recovery status. 
* 
* Indicates that the gate has reopened and that the 
* obstruction should be cleared before normal operation. 
* 
* @param None. 
* @return None. 
*/
void showSafetyOpen() {

  display.clearDisplay();
  display.setCursor(0, 0);

  display.println("SAFETY MODE");
  display.println("");
  display.println("Gate: OPEN");
  display.println("Clear obstruction");

  display.display();
}

/**
* @brief Starts the non-blocking gate opening process.
*
* Activates the gate opening state and records the current
* time so that the servo can move gradually inside loop().
*
* @param None.
* @return None.
*/
void openGateSlowly() {

  Serial.println("Gate OPENING");

  gateOpening = true;
  lastGateMove = millis();
}

/**
 * @brief Starts the non-blocking gate closing process.
 *
 * Sets the gate to its open starting angle and enables
 * gradual closing controlled by the main loop.
 *
 * @param None.
 * @return None.
 */

void startClosingGate() {
  Serial.println("Gate CLOSING");
  gateClosing = true;

  gateAngle = 90;
  lastCloseMove = millis();

  showGateScreen(nullptr, "CLOSING");
}

/**
 * @brief Activates the gate safety recovery procedure.
 *
 * Stops the normal closing process when an obstruction is
 * detected, activates the buzzer and displays a safety alert.
 * The gate is then prepared for automatic reopening.
 *
 * @param None.
 * @return None.
 */

void safetyRecovery() {

  gateClosing = false;
  safetyActive = true;

  Serial.println("SAFETY ALERT - OBSTRUCTION DETECTED");
  Serial.println("Gate STOPPED");

  showSafetyAlert();

  tone(BUZZER_PIN, 2000);
  safetyStartTime = millis();
}

/**
 * @brief Handles the ESP32 hardware timer interrupt.
 *
 * Sets the status update flag when the configured timer
 * interval has elapsed.
 *
 * @param None.
 * @return None.
 */

void IRAM_ATTR onMonitorTimer() {
  monitorUpdateReady = true;
}

/**
 * @brief Handles changes to the ultrasonic echo signal.
 *
 * Records the start and end time of the echo pulse using
 * micros() so that the distance can be calculated without
 * blocking the main program loop.
 *
 * @param None.
 * @return None.
 */

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

/**
 * @brief Performs non-blocking ultrasonic distance measurement.
 *
 * Generates the ultrasonic trigger pulse, processes the echo
 * interrupt, calculates the measured distance and handles
 * echo timeouts without blocking the main program loop.
 *
 * @param None.
 * @return None.
 */

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

/**
 * @brief Performs the initial system configuration.
 *
 * Configures GPIO pins, Serial communication, interrupts,
 * the servo, OLED display and ESP32 hardware timer before
 * normal system operation begins.
 *
 * @param None.
 * @return None.
 */

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

// ==================== OLED INITIALISATION ====================
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

// ==================== HARDWARE TIMER INITIALISATION ====================
monitorTimer = timerBegin(1000000);
timerAttachInterrupt(monitorTimer, &onMonitorTimer);
timerAlarm(monitorTimer, 500000, true, 0);
}

/**
 * @brief Executes the main non-blocking system control loop.
 *
 * Continuously reads sensor inputs, handles Automatic and Manual
 * operation, updates the ultrasonic measurement, controls gate
 * movement, manages Safety Mode and processes UART commands.
 *
 * @param None.
 * @return None.
 */

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

// ==================== ULTRASONIC SENSOR UPDATE ====================
  updateUltrasonic();

// ==================== OLED STATUS UPDATE ====================
  if (monitorUpdateReady == true && safetyActive == false) {
    monitorUpdateReady = false;

  updateStatusDisplay(potValue);
}

// ==================== NON-BLOCKING GATE OPENING ====================
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

// ==================== NON-BLOCKING GATE CLOSING ====================
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

// ==================== SAFETY BUZZER TIMER ====================
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

// ==================== NON-BLOCKING SAFETY REOPENING ====================
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

// ==================== AUTOMATIC PIR CONTROL ====================
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

// ==================== MANUAL UART CONTROL ====================
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
