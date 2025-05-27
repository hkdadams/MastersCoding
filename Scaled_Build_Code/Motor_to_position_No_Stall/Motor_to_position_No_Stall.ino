#include <TMCStepper.h>

// --- Constants ---
#define R_SENSE 0.11f
#define MOTOR_CURRENT_RMS 1500

// --- Pin Definitions ---
// Motor 1 (Using Serial3 for UART)
#define DIR_PIN_1     23
#define STEP_PIN_1    22
#define EN_PIN_1      19

// Motor 2 (Using Serial1 for UART)
#define DIR_PIN_2     3
#define STEP_PIN_2    2
#define EN_PIN_2      16

// Motor 3 (Using Serial2 for UART)
#define DIR_PIN_3     10
#define STEP_PIN_3    9
#define EN_PIN_3      17

#define DRIVER_ADDRESS 0

TMC2209Stepper driver1(&Serial3, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver2(&Serial1, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver3(&Serial2, R_SENSE, DRIVER_ADDRESS);

volatile bool emergencyStopActive = false;

void setupDriver(TMC2209Stepper &driver, HardwareSerial &port, int current, const char* motorName) {
  port.begin(115200);
  driver.begin();
  driver.toff(5);
  driver.rms_current(current);
  driver.microsteps(16);
  Serial.print(motorName); Serial.println(" Initialized.");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000);

  // Set up Step/Dir/Enable pins for all motors
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(EN_PIN_1, OUTPUT);
  digitalWrite(STEP_PIN_1, LOW);
  digitalWrite(DIR_PIN_1, LOW);
  digitalWrite(EN_PIN_1, HIGH); // Disable driver at startup

  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);
  digitalWrite(STEP_PIN_2, LOW);
  digitalWrite(DIR_PIN_2, LOW);
  digitalWrite(EN_PIN_2, HIGH);

  pinMode(STEP_PIN_3, OUTPUT);
  pinMode(DIR_PIN_3, OUTPUT);
  pinMode(EN_PIN_3, OUTPUT);
  digitalWrite(STEP_PIN_3, LOW);
  digitalWrite(DIR_PIN_3, LOW);
  digitalWrite(EN_PIN_3, HIGH);

  setupDriver(driver1, Serial3, MOTOR_CURRENT_RMS, "Motor 1");
  setupDriver(driver2, Serial1, MOTOR_CURRENT_RMS, "Motor 2");
  setupDriver(driver3, Serial2, MOTOR_CURRENT_RMS, "Motor 3");

  Serial.println("Enter commands: 'm[num] p[steps] s[speed]'");
  Serial.println("Example: m1 p2000 s800");
  Serial.println("Or 'stop' to disable all motors.");
}

struct MotorState {
  int dirPin;
  int stepPin;
  int enPin;
  long targetSteps;
  long currentStep;
  int speed_sps;
  unsigned long lastStepTime;
  bool moving;
  int direction;
};

MotorState motors[3] = {
  {DIR_PIN_1, STEP_PIN_1, EN_PIN_1, 0, 0, MOTOR_CURRENT_RMS, 0, false, 1},
  {DIR_PIN_2, STEP_PIN_2, EN_PIN_2, 0, 0, MOTOR_CURRENT_RMS, 0, false, 1},
  {DIR_PIN_3, STEP_PIN_3, EN_PIN_3, 0, 0, MOTOR_CURRENT_RMS, 0, false, 1}
};

void startMotorMove(int motorIndex, long steps, int speed) {
  if (emergencyStopActive) {
    Serial.println("Cannot move during emergency stop");
    return;
  }
  motors[motorIndex].targetSteps = abs(steps);
  motors[motorIndex].currentStep = 0;
  motors[motorIndex].speed_sps = speed;
  motors[motorIndex].lastStepTime = micros();
  motors[motorIndex].moving = true;
  motors[motorIndex].direction = (steps > 0) ? HIGH : LOW;
  digitalWrite(motors[motorIndex].dirPin, motors[motorIndex].direction);
  digitalWrite(motors[motorIndex].enPin, LOW); // Enable driver
  Serial.print("Starting move for Motor "); Serial.print(motorIndex + 1);
  Serial.print(" Steps: "); Serial.print(steps);
  Serial.print(" Speed: "); Serial.println(speed);
}

void updateMotors() {
  unsigned long now = micros();
  for (int i = 0; i < 3; ++i) {
    if (motors[i].moving && !emergencyStopActive) {
      unsigned long delay_us = 1000000L / motors[i].speed_sps / 2;
      if (now - motors[i].lastStepTime >= delay_us) {
        digitalWrite(motors[i].stepPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(motors[i].stepPin, LOW);
        motors[i].currentStep++;
        motors[i].lastStepTime = now;
        if (motors[i].currentStep >= motors[i].targetSteps) {
          motors[i].moving = false;
          digitalWrite(motors[i].enPin, LOW);
          Serial.print("Move complete for M"); Serial.println(i + 1);
        }
      }
    }
    if (emergencyStopActive && motors[i].moving) {
      motors[i].moving = false;
      digitalWrite(motors[i].enPin, HIGH);
      Serial.print("Motor "); Serial.print(i + 1); 
      Serial.println(" stopped due to emergency stop");
    }
  }
}

void loop() {
  updateMotors();

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.equalsIgnoreCase("stop")) {
      emergencyStopActive = true;
      digitalWrite(EN_PIN_1, HIGH);
      digitalWrite(EN_PIN_2, HIGH);
      digitalWrite(EN_PIN_3, HIGH);
      Serial.println("Emergency stop activated. Motors disabled.");
      return;
    }
    
    if (command.equalsIgnoreCase("reset")) {
      emergencyStopActive = false;
      digitalWrite(EN_PIN_1, LOW);
      digitalWrite(EN_PIN_2, LOW);
      digitalWrite(EN_PIN_3, LOW);
      Serial.println("Emergency stop cleared. Motors enabled.");
      return;
    }

    int motorNum = 0;
    long stepsToMove = 0;
    int speed = 800;
    int parsed = sscanf(command.c_str(), "m%d p%ld s%d", &motorNum, &stepsToMove, &speed);

    if (parsed >= 2 && motorNum >= 1 && motorNum <= 3) {
      if (parsed == 2) {
        speed = 800; // Default speed if not provided
      }
      
      if (emergencyStopActive) {
        Serial.println("Cannot move motors during emergency stop.");
      } else {
        startMotorMove(motorNum-1, stepsToMove, speed);
      }
    } else {
      Serial.println("Invalid command. Use: 'm[num] p[steps] s[speed]' or 'stop' or 'reset'");
    }
  }
}