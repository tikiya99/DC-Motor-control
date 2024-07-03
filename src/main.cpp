#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>

#define PPM_PIN 15 // PPM input pin
#define PPM_SYNC_PULSE_WIDTH 3000 // PPM sync pulse width threshold

volatile uint16_t ppmValues[8]; // Array to store pulse lengths
volatile uint8_t currentChannel = 0;
volatile uint32_t lastPPMTime = 0;
volatile bool ppmUpdated = false;

// Constants for motor control
const float wheelDiameterCm = 44.0;
const float encoderResolution = 1200.0;
const float cmPerPulse = (PI * wheelDiameterCm) / encoderResolution;

volatile long encoderPulsesA = 0;
volatile long encoderPulsesB = 0;

typedef struct {
    int distanceCm;
    int maxSpeed;
} Command;

Command command;
bool commandReceived = false;
bool alertReceived = false;

// Motor control pins
const int FRONT_RIGHT_PWM_FORWARD = 21;
const int FRONT_RIGHT_PWM_REVERSE = 19;
const int REAR_RIGHT_PWM_FORWARD = 23;
const int REAR_RIGHT_PWM_REVERSE = 22;

const int REAR_LEFT_ENCODER_A = 33;
const int REAR_LEFT_ENCODER_B = 32;
const int FRONT_RIGHT_ENCODER_A = 26;
const int FRONT_RIGHT_ENCODER_B = 25;

// Function Prototypes
void IRAM_ATTR handlePPMInterrupt();
void IRAM_ATTR encoderISRA();
void IRAM_ATTR encoderISRB();
void moveForward(int speed);
void moveBackward(int speed);
void stopMotors();
void smoothStopMotors();
void prioritizeStop();
void onDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len);
void moveForwardDistanceSmoothly(int maxSpeed, float distanceCm);
void processPPM();

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataReceived);

  pinMode(FRONT_RIGHT_PWM_FORWARD, OUTPUT);
  pinMode(FRONT_RIGHT_PWM_REVERSE, OUTPUT);
  pinMode(REAR_RIGHT_PWM_FORWARD, OUTPUT);
  pinMode(REAR_RIGHT_PWM_REVERSE, OUTPUT);

  pinMode(REAR_LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(REAR_LEFT_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REAR_LEFT_ENCODER_A), encoderISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REAR_LEFT_ENCODER_B), encoderISRA, CHANGE);

  pinMode(FRONT_RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(FRONT_RIGHT_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_ENCODER_A), encoderISRB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_ENCODER_B), encoderISRB, CHANGE);

  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), handlePPMInterrupt, FALLING);

  Serial.println("Setup complete. Drive system ready.");
}

void loop() {
  if (alertReceived) {
    prioritizeStop();
    alertReceived = false;
    Serial.println("Alert received! Emergency stop executed.");
  } else if (commandReceived) {
    moveForwardDistanceSmoothly(command.maxSpeed, command.distanceCm);
    commandReceived = false;
  } else if (ppmUpdated) {
    processPPM();
    ppmUpdated = false;
  }
  delay(10);
}

void moveForward(int speed) {
  analogWrite(FRONT_RIGHT_PWM_FORWARD, speed);
  analogWrite(FRONT_RIGHT_PWM_REVERSE, 0);
  analogWrite(REAR_RIGHT_PWM_FORWARD, speed);
  analogWrite(REAR_RIGHT_PWM_REVERSE, 0);
}

void moveBackward(int speed) {
  analogWrite(FRONT_RIGHT_PWM_FORWARD, 0);
  analogWrite(FRONT_RIGHT_PWM_REVERSE, speed);
  analogWrite(REAR_RIGHT_PWM_FORWARD, 0);
  analogWrite(REAR_RIGHT_PWM_REVERSE, speed);
}

void stopMotors() {
  analogWrite(FRONT_RIGHT_PWM_FORWARD, 0);
  analogWrite(FRONT_RIGHT_PWM_REVERSE, 0);
  analogWrite(REAR_RIGHT_PWM_FORWARD, 0);
  analogWrite(REAR_RIGHT_PWM_REVERSE, 0);
  Serial.println("Motors stopped.");
}

void smoothStopMotors() {
  int currentSpeed = 255; // Maximum speed
  while (currentSpeed > 0) {
    analogWrite(FRONT_RIGHT_PWM_FORWARD, 0);
    analogWrite(FRONT_RIGHT_PWM_REVERSE, currentSpeed);
    analogWrite(REAR_RIGHT_PWM_FORWARD, 0);
    analogWrite(REAR_RIGHT_PWM_REVERSE, currentSpeed);
    currentSpeed -= 5; // Decrease speed gradually
    delay(50); // Adjust delay for smoother reduction
  }
  // Ensure motors are stopped completely
  analogWrite(FRONT_RIGHT_PWM_FORWARD, 0);
  analogWrite(FRONT_RIGHT_PWM_REVERSE, 0);
  analogWrite(REAR_RIGHT_PWM_FORWARD, 0);
  analogWrite(REAR_RIGHT_PWM_REVERSE, 0);
  Serial.println("Motors smoothly stopped.");
}

void prioritizeStop() {
  noInterrupts();
  stopMotors();
  interrupts();
  Serial.println("Emergency stop executed.");
}

void moveForwardDistanceSmoothly(int maxSpeed, float distanceCm) {
  long targetPulses = distanceCm / cmPerPulse;
  long accelDecelPulses = targetPulses / 3;
  long constantSpeedPulses = targetPulses - 2 * accelDecelPulses;

  encoderPulsesA = 0;
  encoderPulsesB = 0;

  long currentPulses = 0;

  Serial.println("Starting smooth movement...");
  int startSpeed = 40;

  while (currentPulses < targetPulses) {
    if (alertReceived) {
      prioritizeStop();
      alertReceived = false;
      Serial.println("Alert received during movement! Emergency stop executed.");
      return;
    }

    currentPulses = (abs(encoderPulsesA) + abs(encoderPulsesB)) / 2;
    int speed = (currentPulses < accelDecelPulses) ? map(currentPulses, 0, accelDecelPulses, startSpeed, maxSpeed) :
               (currentPulses < accelDecelPulses + constantSpeedPulses) ? maxSpeed :
               map(currentPulses, accelDecelPulses + constantSpeedPulses, targetPulses, maxSpeed, startSpeed);
    moveForward(speed);
    delay(10);
  }

  stopMotors();
  Serial.println("Movement complete.");
}

void IRAM_ATTR handlePPMInterrupt() {
  uint32_t currentTime = micros();
  uint32_t pulseWidth = currentTime - lastPPMTime;
  lastPPMTime = currentTime;

  if (pulseWidth > PPM_SYNC_PULSE_WIDTH) {
    currentChannel = 0;
  } else {
    if (currentChannel < 8) {
      ppmValues[currentChannel] = pulseWidth;
      currentChannel++;
      if (currentChannel == 8) {
        ppmUpdated = true;
      }
    }
  }
}

void IRAM_ATTR encoderISRA() {
  static bool lastStateA = LOW;
  bool currentStateA = digitalRead(REAR_LEFT_ENCODER_A);
  if (currentStateA != lastStateA) {
    encoderPulsesA += (digitalRead(REAR_LEFT_ENCODER_B) == currentStateA) ? -1 : 1;
    lastStateA = currentStateA;
  }
}

void IRAM_ATTR encoderISRB() {
  static bool lastStateB = LOW;
  bool currentStateB = digitalRead(FRONT_RIGHT_ENCODER_A);
  if (currentStateB != lastStateB) {
    encoderPulsesB += (digitalRead(FRONT_RIGHT_ENCODER_B) == currentStateB) ? -1 : 1;
    lastStateB = currentStateB;
  }
}

void onDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(Command)) {
    memcpy(&command, incomingData, sizeof(Command));
    commandReceived = true;
  } else if (strncmp((const char*)incomingData, "Distance alert!", len) == 0) {
    alertReceived = true;
  }
}

void processPPM() {
  int ppmValue = ppmValues[1]; // Read the PPM value for channel 1

  if (ppmValue >= 1480 && ppmValue <= 1520) {
    stopMotors();
    Serial.println("PPM Value within stop range: Motors stopped.");
  } else {
    int speed;
    if (ppmValue < 1480) {
      // Map PPM value to PWM for backward motion
      speed = map(ppmValue, 1000, 1480, 220, 0); 
      analogWrite(FRONT_RIGHT_PWM_FORWARD, 0);
      analogWrite(FRONT_RIGHT_PWM_REVERSE, speed);
      analogWrite(REAR_RIGHT_PWM_FORWARD, 0);
      analogWrite(REAR_RIGHT_PWM_REVERSE, speed);
    } else {
      // Map PPM value to PWM for forward motion
      speed = map(ppmValue, 1520, 2000, 0, 220); 
      analogWrite(FRONT_RIGHT_PWM_FORWARD, speed);
      analogWrite(FRONT_RIGHT_PWM_REVERSE, 0);
      analogWrite(REAR_RIGHT_PWM_FORWARD, speed);
      analogWrite(REAR_RIGHT_PWM_REVERSE, 0);
    }

    Serial.print("PPM Value: ");
    Serial.println(ppmValue);
    Serial.print("Motor Speed: ");
    Serial.println(speed);
  }
}