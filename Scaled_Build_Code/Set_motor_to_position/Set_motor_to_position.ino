#include <TMCStepper.h> // Make sure this library is installed

// --- Constants ---
#define R_SENSE 0.11f // Sense resistor value in ohms (typical value is 0.11 ohms)
#define MOTOR_CURRENT_RMS 1500 // Motor current in mA (adjust based on your motor specs)
#define STALLGUARD_THRESHOLD 50 // StallGuard threshold (needs tuning for your specific setup)

// --- Pin Definitions ---

// Motor 1 (Using Serial3 for UART)
#define DIR_PIN_1     23
#define STEP_PIN_1    22
#define EN_PIN_1      19 // Digital pin for Enable
// UART for M1: Serial3 (Teensy D20=TX3, D21=RX3)

// Motor 2 (Using Serial1 for UART)
#define DIR_PIN_2     3
#define STEP_PIN_2    2
#define EN_PIN_2      16 // Changed from D0 to avoid conflict with Serial1 RX0
// UART for M2: Serial1 (Teensy D1=TX1, D0=RX1)

// Motor 3 (Using Serial2 for UART)
#define DIR_PIN_3     10
#define STEP_PIN_3    9
#define EN_PIN_3      17 // Changed from D7 to avoid conflict with Serial2 RX2
// UART for M3: Serial2 (Teensy D8=TX2, D7=RX2)

// --- DIAG Pin Definitions for Stall Detection ---
#define DIAG_PIN_1 25
#define DIAG_PIN_2 28
#define DIAG_PIN_3 33

// Create stepper driver instances - only pass Serial and R_SENSE
#define DRIVER_ADDRESS 0 // Set to your TMC2209 UART address (0 or 1)

TMC2209Stepper driver1(&Serial3, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver2(&Serial1, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver3(&Serial2, R_SENSE, DRIVER_ADDRESS);

volatile bool emergencyStopActive = false;
volatile bool motorStalled[3] = {false, false, false}; // Stall flags for each motor
volatile unsigned long lastStallTime[3] = {0, 0, 0}; // Store last interrupt time for each motor
const unsigned long debounceDelay = 5000; // microseconds (5ms debounce)
volatile unsigned long lastEStopTime = 0;
const unsigned long estopDebounceDelay = 5000; // microseconds (5ms debounce)

void stallISR1() {
  unsigned long now = micros();
  if (now - lastStallTime[0] > debounceDelay) {
    motorStalled[0] = true;
    lastStallTime[0] = now;
  }
}

void stallISR2() {
  unsigned long now = micros();
  if (now - lastStallTime[1] > debounceDelay) {
    motorStalled[1] = true;
    lastStallTime[1] = now;
  }
}

void stallISR3() {
  unsigned long now = micros();
  if (now - lastStallTime[2] > debounceDelay) {
    motorStalled[2] = true;
    lastStallTime[2] = now;
  }
}

void setupDriver(TMC2209Stepper &driver, HardwareSerial &port, int current, uint8_t sg_threshold, const char* motorName) {
  port.begin(115200); // Baud rate for TMC2209 UART
  driver.begin();  driver.toff(5);
  driver.rms_current(current);
  driver.microsteps(16);
  
  // CoolStep configuration - optional current regulation based on motor load
  driver.semin(5);    // Minimum load for CoolStep current reduction (0-15)
  driver.semax(2);    // Hysteresis for CoolStep current restoration (0-15)
  driver.sedn(0b01);  // Current decrease step speed for CoolStep
  
  // StallGuard configuration - critical for stall detection
  driver.SGTHRS(sg_threshold); // StallGuard threshold - VERY IMPORTANT TO TUNE!
  // driver.pwm_autoscale(true); // Recommended for stealthChop
  // driver.en_spreadCycle(false); // Use stealthChop (quieter) by default. Set true for spreadCycle (more torque)
  
  // Removed pin control here - we're already handling enable pins explicitly elsewhere
  Serial.print(motorName); Serial.println(" Initialized.");
}

void setup() {
  Serial.begin(115200); // USB Serial for commands
  while (!Serial && millis() < 4000);

  // Set DIAG pins as inputs and attach interrupts for stall detection
  pinMode(DIAG_PIN_1, INPUT);
  pinMode(DIAG_PIN_2, INPUT);
  pinMode(DIAG_PIN_3, INPUT);
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN_1), stallISR1, FALLING); // DIAG active LOW on stall
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN_2), stallISR2, FALLING);
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN_3), stallISR3, FALLING);

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
  digitalWrite(EN_PIN_2, HIGH); // Disable driver at startup

  pinMode(STEP_PIN_3, OUTPUT);
  pinMode(DIR_PIN_3, OUTPUT);
  pinMode(EN_PIN_3, OUTPUT);
  digitalWrite(STEP_PIN_3, LOW);
  digitalWrite(DIR_PIN_3, LOW);
  digitalWrite(EN_PIN_3, HIGH); // Disable driver at startup

  // Initialize Motor 1 (Serial3: D20 TX3, D21 RX3)
  setupDriver(driver1, Serial3, MOTOR_CURRENT_RMS, STALLGUARD_THRESHOLD, "Motor 1");
  // Initialize Motor 2 (Serial1: D1 TX1, D0 RX1)
  setupDriver(driver2, Serial1, MOTOR_CURRENT_RMS, STALLGUARD_THRESHOLD, "Motor 2");
  // Initialize Motor 3 (Serial2: D8 TX2, D7 RX2)
  setupDriver(driver3, Serial2, MOTOR_CURRENT_RMS, STALLGUARD_THRESHOLD, "Motor 3");

  Serial.println("Enter commands: 'm[num] p[steps] s[speed]' or 'm[num] t[threshold]' to set SGTHRS");
  Serial.println("Example: m1 p2000 s800  OR  m1 t50");
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
  if (motorStalled[motorIndex] || emergencyStopActive) {
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
  motorStalled[motorIndex] = false;
}

void updateMotors() {
  unsigned long now = micros();
  for (int i = 0; i < 3; ++i) {
    if (motors[i].moving && !motorStalled[i] && !emergencyStopActive) {
      unsigned long delay_us = 1000000L / motors[i].speed_sps / 2;
      if (now - motors[i].lastStepTime >= delay_us) {
        digitalWrite(motors[i].stepPin, HIGH);
        delayMicroseconds(5); // Short pulse
        digitalWrite(motors[i].stepPin, LOW);
        motors[i].currentStep++;
        motors[i].lastStepTime = now;
        if (motors[i].currentStep >= motors[i].targetSteps) {
          motors[i].moving = false;
          digitalWrite(motors[i].enPin, LOW); // Keep enabled
          Serial.print("Move complete for M"); Serial.println(i + 1);
        }
      }
    }
    // Stall handling: only stop the stalled motor, not all
    if (motorStalled[i] && motors[i].moving) {
      motors[i].moving = false;
      digitalWrite(motors[i].enPin, HIGH); // Disable only the stalled driver
      Serial.print("STALL DETECTED on Motor "); Serial.println(i + 1);
    }
    // Emergency stop: stop all motors
    if (emergencyStopActive && motors[i].moving) {
      motors[i].moving = false;
      digitalWrite(motors[i].enPin, HIGH); // Disable all drivers
    }
  }
}

void loop() {
  updateMotors();
  if (emergencyStopActive) {
    digitalWrite(EN_PIN_1, HIGH); // Disable driver
    digitalWrite(EN_PIN_2, HIGH);
    digitalWrite(EN_PIN_3, HIGH);
    Serial.println("EMERGENCY STOP! Motors disabled. Send 'resetstalls' to re-enable.");
    // Wait here until user sends 'resetstalls'
    while (emergencyStopActive) {
      if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.equalsIgnoreCase("resetstalls")) {
          for(int i=0; i<3; ++i) motorStalled[i] = false;
          emergencyStopActive = false;
          Serial.println("E-Stop cleared. Stall flags reset. Motors enabled.");
          break;
        }
      }
      delay(100);
    }
    return;
  }

  for (int i = 0; i < 3; ++i) {
    if (motorStalled[i]) {
      // Handle stalled motor (e.g., keep it disabled or log it)
      // For now, it just prevents further movement in moveMotorWithStallDetection
    }
  }

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.print("Received: "); Serial.println(command);    if (command.length() == 0) {
      return; // Ignore empty input
    }

    if (command.equalsIgnoreCase("stop")) {
      emergencyStopActive = true;
      return;
    }
    
    if (command.equalsIgnoreCase("resetstalls")) {
      for(int i=0; i<3; ++i) {
        motorStalled[i] = false;
      }
      if (!emergencyStopActive) {
        digitalWrite(EN_PIN_1, LOW); // Re-enable drivers if E-Stop is not active
        digitalWrite(EN_PIN_2, LOW);
        digitalWrite(EN_PIN_3, LOW);
      }
      Serial.println("Stall flags reset. Motors enabled (if E-Stop is clear).");
      return;
    }
    
    if (command.equalsIgnoreCase("enable1")) {
      if (!emergencyStopActive) {
        digitalWrite(EN_PIN_1, LOW); // Enable driver 1 (active LOW)
        Serial.println("Motor 1 enabled.");
      } else {
        Serial.println("Cannot enable motors during emergency stop.");
      }
      return;
    }
    
    if (command.equalsIgnoreCase("disable1")) {
      digitalWrite(EN_PIN_1, HIGH); // Disable driver 1
      Serial.println("Motor 1 disabled.");
      return;
    }
    
    if (command.equalsIgnoreCase("enable2")) {
      if (!emergencyStopActive) {
        digitalWrite(EN_PIN_2, LOW); // Enable driver 2 (active LOW)
        Serial.println("Motor 2 enabled.");
      } else {
        Serial.println("Cannot enable motors during emergency stop.");
      }
      return;
    }
    
    if (command.equalsIgnoreCase("disable2")) {
      digitalWrite(EN_PIN_2, HIGH); // Disable driver 2
      Serial.println("Motor 2 disabled.");
      return;
    }
    
    if (command.equalsIgnoreCase("enable3")) {
      if (!emergencyStopActive) {
        digitalWrite(EN_PIN_3, LOW); // Enable driver 3 (active LOW)
        Serial.println("Motor 3 enabled.");
      } else {
        Serial.println("Cannot enable motors during emergency stop.");
      }
      return;
    }

    if (command.equalsIgnoreCase("disable3")) {
      digitalWrite(EN_PIN_3, HIGH); // Disable driver 3
      Serial.println("Motor 3 disabled.");
      return;
    }

    if (command.equalsIgnoreCase("diag1")) {
      Serial.print("DIAG1: ");
      Serial.println(digitalRead(DIAG_PIN_1)); // Disable driver 3
      return;
    }

    if (command.equalsIgnoreCase("diag2")) {
      Serial.print("DIAG2: ");
      Serial.println(digitalRead(DIAG_PIN_2)); // Disable driver 3
      return;
    }

    if (command.equalsIgnoreCase("diag3")) {
      Serial.print("DIAG3: ");
      Serial.println(digitalRead(DIAG_PIN_3)); // Disable driver 3
      return;
    }

    int motorNum = 0;
    long stepsToMove = 0;
    int speed = 800;
    int thresholdValue = 0;
    int parsed_move = sscanf(command.c_str(), "m%d p%ld s%d", &motorNum, &stepsToMove, &speed);
    int parsed_thresh = sscanf(command.c_str(), "m%d t%d", &motorNum, &thresholdValue);

    if (parsed_thresh == 2 && motorNum >= 1 && motorNum <= 3) {
        Serial.print("Setting SGTHRS for Motor "); Serial.print(motorNum);
        Serial.print(" to "); Serial.println(thresholdValue);
        if (motorNum == 1) {
            driver1.SGTHRS(thresholdValue);
        } else if (motorNum == 2) {
            driver2.SGTHRS(thresholdValue);
        } else if (motorNum == 3) {
            driver3.SGTHRS(thresholdValue);
        }
        return;
    } else if (parsed_move >= 2 && motorNum >= 1 && motorNum <= 3) { // p[steps] is mandatory, s[speed] is optional
      if (parsed_move == 2) {
          speed = 800; // Default speed if not provided
      }

      Serial.print("Motor: "); Serial.print(motorNum);
      Serial.print(" Steps: "); Serial.print(stepsToMove);
      Serial.print(" Speed: "); Serial.println(speed);

      if (motorStalled[motorNum-1]) {
        Serial.print("Motor "); Serial.print(motorNum); Serial.println(" is stalled. Reset stalls to move.");
      } else if (emergencyStopActive) {
        Serial.println("Cannot move motors during emergency stop.");
      } else {
        startMotorMove(motorNum-1, stepsToMove, speed);
      }
      return;
    } else {
      Serial.println("Invalid command. Use: 'm[num] p[steps] s[speed]' OR 'm[num] t[value]' OR 'resetstalls' OR 'stop'");
      Serial.println("Or 'enable1/2/3' to enable a motor, 'disable1/2/3' to disable a motor.");
      // Flush any remaining input to avoid repeated errors
      while (Serial.available() > 0) {
          Serial.read();
      }
      return;
    }
  }
}