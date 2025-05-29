#include <TMCStepper.h> // Make sure this library is installed

// --- Constants ---
#define R_SENSE 0.11f // Sense resistor value in ohms (typical value is 0.11 ohms)
#define MOTOR_CURRENT_RMS 1500 // Motor current in mA (increase for better torque)
#define STALLGUARD_THRESHOLD 100 // StallGuard threshold (increase to be less sensitive)

// --- Pin Definitions ---

// Motor 1 (Using Serial3 for UART)
#define DIR_PIN_1     23
#define STEP_PIN_1    22
#define EN_PIN_1      19 // Digital pin for Enable
// UART for M1: Serial3 (Teensy D20=TX3, D21=RX3)

// Motor 2 (Using Serial1 for UART)
#define DIR_PIN_2     3
#define STEP_PIN_2    2
#define EN_PIN_2      18 // Changed from D0 to avoid conflict with Serial1 RX0
// UART for M2: Serial1 (Teensy D1=TX1, D0=RX1)

// Motor 3 (Using Serial2 for UART)
#define DIR_PIN_3     10
#define STEP_PIN_3    9
#define EN_PIN_3      15 // Changed from D7 and D11 to avoid conflict with Serial2 RX2 and MOSI
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

// Motor state structure - must be declared before ISR functions
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

// Motor array - must be declared before ISR functions
MotorState motors[3] = {
  {DIR_PIN_1, STEP_PIN_1, EN_PIN_1, 0, 0, MOTOR_CURRENT_RMS, 0, false, 1},
  {DIR_PIN_2, STEP_PIN_2, EN_PIN_2, 0, 0, MOTOR_CURRENT_RMS, 0, false, 1},
  {DIR_PIN_3, STEP_PIN_3, EN_PIN_3, 0, 0, MOTOR_CURRENT_RMS, 0, false, 1}
};

// Instruction queue system for batch movements
#define MAX_INSTRUCTIONS 10  // Maximum instructions per motor

struct Instruction {
  long steps;
  int speed;
  bool valid;
};

// Instruction queues for each motor
Instruction instructionQueue[3][MAX_INSTRUCTIONS];
int instructionCount[3] = {0, 0, 0};  // Number of instructions queued for each motor
int currentInstruction[3] = {0, 0, 0}; // Current instruction being executed for each motor
bool batchMode = false;  // Flag to indicate if we're in batch execution mode
bool batchActive = false; // Flag to indicate batch is currently running

// Speed limiting system
int maxSpeed[3] = {3000, 3000, 3000}; // Maximum allowed speed for each motor (steps per second)

// Position limiting system  
long positionLimit[3] = {14000, 14000, 14000}; // Maximum allowed steps from home position (±steps)

// Position tracking system
struct PositionTracker {
  long totalSteps;           // Total steps from home position
  float mmPerStep;          // Conversion factor: millimeters per step
  float currentPosition_mm; // Current position in millimeters
  long homeOffset;          // Steps offset from absolute zero
  bool isHomed;             // Whether this motor has been homed
};

// Position trackers for each motor
PositionTracker position[3] = {
  {0, 0.0125, 0.0, 0, false},  // Motor 1: 0.0125mm per step (belt drive calibration)
  {0, 0.0125, 0.0, 0, false},  // Motor 2: 0.0125mm per step (belt drive calibration)
  {0, 0.0125, 0.0, 0, false}   // Motor 3: 0.0125mm per step (belt drive calibration)
};

volatile bool emergencyStopActive = false;
volatile bool motorStalled[3] = {false, false, false}; // Stall flags for each motor
volatile unsigned long lastStallTime[3] = {0, 0, 0}; // Store last interrupt time for each motor
volatile unsigned long falseInterruptCount[3] = {0, 0, 0}; // Count false interrupts for debugging
const unsigned long debounceDelay = 10000; // microseconds (10ms debounce)
volatile unsigned long lastEStopTime = 0;
const unsigned long estopDebounceDelay = 5000; // microseconds (5ms debounce)

void stallISR1() {
  // Only process stall if motor is actually moving
  if (motors[0].moving) {
    // Double-check: Read the actual DIAG pin state to filter false interrupts
    if (digitalRead(DIAG_PIN_1) == HIGH) {
      unsigned long now = micros();
      if (now - lastStallTime[0] > debounceDelay) {
        motorStalled[0] = true;
        lastStallTime[0] = now;
        Serial.println("STALL DETECTED on Motor 1 (DIAG1 triggered)");
      }
    } else {
      // False interrupt - DIAG pin is not actually HIGH
      falseInterruptCount[0]++;
    }
  }
}

void stallISR2() {
  // Only process stall if motor is actually moving
  if (motors[1].moving) {
    // Double-check: Read the actual DIAG pin state to filter false interrupts
    if (digitalRead(DIAG_PIN_2) == HIGH) {
      unsigned long now = micros();
      if (now - lastStallTime[1] > debounceDelay) {
        motorStalled[1] = true;
        lastStallTime[1] = now;
        Serial.println("STALL DETECTED on Motor 2 (DIAG2 triggered)");
      }
    } else {
      falseInterruptCount[1]++;
    }
  }
}

void stallISR3() {
  // Only process stall if motor is actually moving
  if (motors[2].moving) {
    // Double-check: Read the actual DIAG pin state to filter false interrupts
    if (digitalRead(DIAG_PIN_3) == HIGH) {
      unsigned long now = micros();
      if (now - lastStallTime[2] > debounceDelay) {
        motorStalled[2] = true;
        lastStallTime[2] = now;
        Serial.println("STALL DETECTED on Motor 3 (DIAG3 triggered)");
      }
    } else {
      falseInterruptCount[2]++;
    }
  }
}

void setupDriver(TMC2209Stepper &driver, HardwareSerial &port, int current, uint8_t sg_threshold, const char* motorName) {
  port.begin(115200); // Baud rate for TMC2209 UART
  driver.begin();  
  driver.toff(5);
  driver.rms_current(current);
  driver.microsteps(16);
  
  // Force spreadCycle mode for more reliable stall detection
  driver.en_spreadCycle(true);  // Use spreadCycle instead of stealthChop for stall detection
  driver.pwm_autoscale(true);   // Enable PWM autoscaling
  
  // CoolStep configuration - make less aggressive
  driver.semin(0);    // Disable CoolStep initially (0 = disabled)
  driver.semax(0);    // Disable CoolStep initially  
  driver.sedn(0b01);  // Current decrease step speed for CoolStep
  
  // StallGuard configuration - start with very high threshold (less sensitive)
  driver.SGTHRS(255); // Maximum threshold - least sensitive to start
  
  // Configure DIAG pin for stall detection using direct register access
  uint32_t gconf = driver.GCONF();
  gconf |= 0x80;  // Set diag0_stall bit (bit 7) to enable stall output on DIAG
  gconf &= ~0x40; // Clear diag0_otpw bit (bit 6) to not output overtemp/short circuit on DIAG
  gconf &= ~0x100; // Clear diag1_stall bit (bit 8) - we're using DIAG0 only
  gconf &= ~0x200; // Clear diag1_index bit (bit 9) - not using step direction for diagnostics
  driver.GCONF(gconf);
  
  // Small delay to ensure settings are applied
  delay(100);
  
  Serial.print(motorName); Serial.println(" Initialized.");
  Serial.print("GCONF: 0x"); Serial.println(driver.GCONF(), HEX);
  Serial.print("SGTHRS: "); Serial.println(driver.SGTHRS());
  Serial.print("spreadCycle: "); Serial.println(driver.en_spreadCycle());
}

void setup() {
  Serial.begin(115200); // USB Serial for commands
  while (!Serial && millis() < 4000);  // Set DIAG pins with pullup resistors to reduce noise sensitivity
  pinMode(DIAG_PIN_1, INPUT_PULLUP);  // Use pullup to reduce noise sensitivity
  pinMode(DIAG_PIN_2, INPUT_PULLUP);
  pinMode(DIAG_PIN_3, INPUT_PULLUP);
  
  // Don't attach interrupts initially - we'll enable them after motor initialization
  // attachInterrupt(digitalPinToInterrupt(DIAG_PIN_1), stallISR1, FALLING);  
  // attachInterrupt(digitalPinToInterrupt(DIAG_PIN_2), stallISR2, FALLING);
  // attachInterrupt(digitalPinToInterrupt(DIAG_PIN_3), stallISR3, FALLING);

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
  digitalWrite(EN_PIN_3, HIGH); // Disable driver at startup  // Initialize Motor 1 (Serial3: D20 TX3, D21 RX3)
  setupDriver(driver1, Serial3, MOTOR_CURRENT_RMS, STALLGUARD_THRESHOLD, "Motor 1");
  // Initialize Motor 2 (Serial1: D1 TX1, D0 RX1)
  setupDriver(driver2, Serial1, MOTOR_CURRENT_RMS, STALLGUARD_THRESHOLD, "Motor 2");
  // Initialize Motor 3 (Serial2: D8 TX2, D7 RX2)
  setupDriver(driver3, Serial2, MOTOR_CURRENT_RMS, STALLGUARD_THRESHOLD, "Motor 3");

  // Temporarily enable drivers to get proper DIAG pin initialization
  Serial.println("Temporarily enabling drivers for DIAG pin setup...");
  digitalWrite(EN_PIN_1, LOW);
  digitalWrite(EN_PIN_2, LOW);
  digitalWrite(EN_PIN_3, LOW);
  delay(100); // Let drivers stabilize
  
  // Check DIAG pin readings while drivers are enabled
  Serial.print("DIAG readings with drivers enabled - DIAG1: ");
  Serial.print(digitalRead(DIAG_PIN_1));
  Serial.print(", DIAG2: ");
  Serial.print(digitalRead(DIAG_PIN_2));
  Serial.print(", DIAG3: ");
  Serial.println(digitalRead(DIAG_PIN_3));
  
  // Now disable drivers again and wait
  digitalWrite(EN_PIN_1, HIGH);
  digitalWrite(EN_PIN_2, HIGH);
  digitalWrite(EN_PIN_3, HIGH);
  delay(100);

  // Don't attach interrupts immediately - let the system stabilize first
  delay(500);
    // Now attach interrupts after drivers are initialized and stabilized
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN_1), stallISR1, RISING);  // Stall detection on RISING edge
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN_2), stallISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN_3), stallISR3, RISING);

  // Clear any false stall flags that might have been set during initialization
  for (int i = 0; i < 3; i++) {
    motorStalled[i] = false;
  }  Serial.println("Enter commands: 'm[num] p[steps] s[speed]' or 'm[num] t[threshold]' to set SGTHRS");
  Serial.println("Example: m1 p2000 s800  OR  m1 t50");
  Serial.println("Or 'stop' to disable all motors.");
  Serial.println("Type 'help' for complete command reference.");
  Serial.println("Additional commands: 'diag1/2/3' to read DIAG pin values, 'sgr1/2/3' to read StallGuard result");
  Serial.println("Debug commands: 'status' for system info, 'stalloff'/'stallon' to disable/enable stall detection");
  Serial.println("Test commands: 'testmove' for movement without stall detection, 'testall' for simultaneous 3-motor test");  Serial.println("Batch commands: 'add m[num] p[steps] s[speed]' to queue instructions, 'show' to view queues");
  Serial.println("               'run' to execute all queued instructions, 'clear' to clear all queues");  Serial.println("Speed commands: 'setspeed m[num] s[speed]' to set max speed, 'showspeeds' to view current limits");
  Serial.println("Position commands: 'positions' to show all positions, 'home m[num]' to home motor");
  Serial.println("                  'cal m[num] f[mm/step]' to set calibration, 'goto m[num] p[mm] s[speed]' for absolute move");
  Serial.println("Limit commands: 'showlimits' to view position limits, 'setlimit m[num] l[steps]' to modify limits");
  Serial.println("Noise debugging: 'noisecount' to check false interrupts, 'resetnoise' to reset counters");
    // Print current speed limits
  Serial.println("=== Current Speed Limits ===");
  for (int i = 0; i < 3; i++) {
    Serial.print("Motor "); Serial.print(i + 1); 
    Serial.print(": "); Serial.print(maxSpeed[i]); Serial.println(" sps");
  }
  
  // Print current position limits
  Serial.println("=== Current Position Limits ===");
  for (int i = 0; i < 3; i++) {
    Serial.print("Motor "); Serial.print(i + 1); 
    Serial.print(": ±"); Serial.print(positionLimit[i]); Serial.println(" steps");
  }
  
  // Print initial position tracking status
  Serial.println("=== Position Tracking Status ===");
  for (int i = 0; i < 3; i++) {
    Serial.print("Motor "); Serial.print(i + 1); 
    Serial.print(": "); Serial.print(position[i].mmPerStep, 4); Serial.print(" mm/step, ");
    Serial.print(position[i].isHomed ? "HOMED" : "NOT HOMED");
    Serial.print(" ("); Serial.print(position[i].totalSteps); Serial.println(" steps)");
  }
  
  // Print final DIAG pin readings
  Serial.print("Final DIAG readings - DIAG1: ");
  Serial.print(digitalRead(DIAG_PIN_1));
  Serial.print(", DIAG2: ");
  Serial.print(digitalRead(DIAG_PIN_2));
  Serial.print(", DIAG3: ");
  Serial.println(digitalRead(DIAG_PIN_3));  Serial.println("Setup complete. System ready for motor commands.");
}

// Speed validation function
int validateSpeed(int motorIndex, int requestedSpeed) {
  if (requestedSpeed <= 0) {
    Serial.println("Warning: Speed must be positive, using default 800 sps");
    return 800;
  }
  
  if (requestedSpeed > maxSpeed[motorIndex]) {
    Serial.print("Warning: Requested speed "); Serial.print(requestedSpeed);
    Serial.print(" exceeds maximum for Motor "); Serial.print(motorIndex + 1);
    Serial.print(" ("); Serial.print(maxSpeed[motorIndex]); Serial.print(" sps). ");
    Serial.print("Limiting to "); Serial.print(maxSpeed[motorIndex]); Serial.println(" sps");
    return maxSpeed[motorIndex];
  }
    return requestedSpeed;
}

// Time-based speed calculation function
int calculateSpeedFromTime(long steps, unsigned long time_ms) {
  if (time_ms <= 0) {
    Serial.println("Warning: Time must be positive, using default 800 sps");
    return 800;
  }
  
  if (steps == 0) {
    Serial.println("Warning: No movement requested, using default 800 sps");
    return 800;
  }
  
  // Calculate steps per second: steps / (time_ms / 1000)
  float calculatedSpeed = (float)abs(steps) * 1000.0 / (float)time_ms;
  
  // Round to nearest integer
  int speed = (int)(calculatedSpeed + 0.5);
  
  // Ensure minimum speed of 1 sps
  if (speed < 1) {
    Serial.print("Warning: Calculated speed too low ("); Serial.print(calculatedSpeed, 2);
    Serial.println(" sps), using minimum 1 sps");
    return 1;
  }
  
  Serial.print("Calculated speed: "); Serial.print(speed);
  Serial.print(" sps for "); Serial.print(abs(steps));
  Serial.print(" steps in "); Serial.print(time_ms); Serial.println(" ms");
  
  return speed;
}

// Position validation function
bool validatePosition(int motorIndex, long requestedSteps, long &validatedSteps) {
  if (motorIndex < 0 || motorIndex >= 3) {
    Serial.println("Error: Invalid motor index");
    return false;
  }
  
  // Calculate what the new position would be after the move
  long newPosition = position[motorIndex].totalSteps + requestedSteps;
  
  // Check if the new position would exceed limits
  if (abs(newPosition) > positionLimit[motorIndex]) {
    Serial.print("Error: Move would exceed position limit for Motor "); Serial.print(motorIndex + 1);
    Serial.print(". Current: "); Serial.print(position[motorIndex].totalSteps);
    Serial.print(" steps, Requested: "); Serial.print(requestedSteps);
    Serial.print(" steps, Would result in: "); Serial.print(newPosition);
    Serial.print(" steps (limit: ±"); Serial.print(positionLimit[motorIndex]); Serial.println(" steps)");
    
    // Calculate maximum allowed move in the requested direction
    long maxAllowedPosition = (requestedSteps > 0) ? positionLimit[motorIndex] : -positionLimit[motorIndex];
    long maxAllowedMove = maxAllowedPosition - position[motorIndex].totalSteps;
    
    if (abs(maxAllowedMove) < 1) {
      Serial.println("Motor is already at position limit. Move rejected.");
      return false;
    }
    
    validatedSteps = maxAllowedMove;
    Serial.print("Limiting move to "); Serial.print(validatedSteps); 
    Serial.println(" steps to stay within position limits");
    return true;
  }
  
  // Move is within limits
  validatedSteps = requestedSteps;
  return true;
}

// Position tracking functions
void updatePosition(int motorIndex, long steps) {
  if (motorIndex < 0 || motorIndex >= 3) return;
  
  position[motorIndex].totalSteps += steps;
  position[motorIndex].currentPosition_mm = position[motorIndex].totalSteps * position[motorIndex].mmPerStep;
}

void setPositionCalibration(int motorIndex, float mmPerStep) {
  if (motorIndex < 0 || motorIndex >= 3) return;
  
  position[motorIndex].mmPerStep = mmPerStep;
  // Recalculate current position with new calibration
  position[motorIndex].currentPosition_mm = position[motorIndex].totalSteps * position[motorIndex].mmPerStep;
  
  Serial.print("Motor "); Serial.print(motorIndex + 1);
  Serial.print(" calibration set to "); Serial.print(mmPerStep, 4);
  Serial.println(" mm/step");
}

void homeMotor(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= 3) return;
  
  position[motorIndex].homeOffset = position[motorIndex].totalSteps;
  position[motorIndex].totalSteps = 0;
  position[motorIndex].currentPosition_mm = 0.0;
  position[motorIndex].isHomed = true;
  
  Serial.print("Motor "); Serial.print(motorIndex + 1); Serial.println(" homed. Position reset to 0.0mm");
}

void moveToPosition(int motorIndex, float targetPosition_mm, int speed) {
  if (motorIndex < 0 || motorIndex >= 3) return;
  
  if (!position[motorIndex].isHomed) {
    Serial.print("Motor "); Serial.print(motorIndex + 1); 
    Serial.println(" not homed. Use 'home m[num]' first.");
    return;
  }
  
  // Calculate steps needed to reach target position
  long targetSteps = (long)(targetPosition_mm / position[motorIndex].mmPerStep);
  long stepsToMove = targetSteps - position[motorIndex].totalSteps;
  
  Serial.print("Moving Motor "); Serial.print(motorIndex + 1);
  Serial.print(" from "); Serial.print(position[motorIndex].currentPosition_mm, 2);
  Serial.print("mm to "); Serial.print(targetPosition_mm, 2);
  Serial.print("mm ("); Serial.print(stepsToMove); Serial.println(" steps)");
  
  startMotorMove(motorIndex, stepsToMove, speed);
}

void showPositions() {
  Serial.println("=== Current Motor Positions ===");
  for (int i = 0; i < 3; i++) {
    Serial.print("Motor "); Serial.print(i + 1); Serial.print(": ");
    
    if (position[i].isHomed) {
      Serial.print(position[i].currentPosition_mm, 3); Serial.print("mm (");
      Serial.print(position[i].totalSteps); Serial.println(" steps)");
    } else {
      Serial.print(position[i].totalSteps); Serial.println(" steps [NOT HOMED]");
    }
    
    Serial.print("  Calibration: "); Serial.print(position[i].mmPerStep, 4);
    Serial.println(" mm/step");
  }
}

void startMotorMove(int motorIndex, long steps, int speed) {
  if (emergencyStopActive) {
    Serial.println("Cannot move during emergency stop");
    return;
  }
  if (motorStalled[motorIndex]) {
    Serial.print("Motor "); Serial.print(motorIndex + 1); 
    Serial.println(" is marked as stalled. Use 'resetstalls' command to clear.");
    return;  }
    // Validate and limit speed
  int validatedSpeed = validateSpeed(motorIndex, speed);
  
  // Validate and limit position
  long validatedSteps;
  if (!validatePosition(motorIndex, steps, validatedSteps)) {
    return; // Position validation failed, move rejected
  }
  
  // Debug: Check DIAG pin and StallGuard before starting movement
  int diagPin = (motorIndex == 0) ? DIAG_PIN_1 : (motorIndex == 1) ? DIAG_PIN_2 : DIAG_PIN_3;
  Serial.print("PRE-MOVE Debug - DIAG"); Serial.print(motorIndex + 1); 
  Serial.print(": "); Serial.print(digitalRead(diagPin));
  
  if (motorIndex == 0) {
    Serial.print(", SG_RESULT: "); Serial.print(driver1.SG_RESULT());
    Serial.print(", SGTHRS: "); Serial.println(driver1.SGTHRS());
  } else if (motorIndex == 1) {
    Serial.print(", SG_RESULT: "); Serial.print(driver2.SG_RESULT());
    Serial.print(", SGTHRS: "); Serial.println(driver2.SGTHRS());
  } else if (motorIndex == 2) {
    Serial.print(", SG_RESULT: "); Serial.print(driver3.SG_RESULT());
    Serial.print(", SGTHRS: "); Serial.println(driver3.SGTHRS());
  }
    motors[motorIndex].targetSteps = abs(validatedSteps);
  motors[motorIndex].currentStep = 0;
  motors[motorIndex].speed_sps = validatedSpeed;  // Use validated speed
  motors[motorIndex].lastStepTime = micros();
  motors[motorIndex].moving = true;
  motors[motorIndex].direction = (validatedSteps > 0) ? HIGH : LOW;
  digitalWrite(motors[motorIndex].dirPin, motors[motorIndex].direction);
  digitalWrite(motors[motorIndex].enPin, LOW); // Enable driver
  
  Serial.print("Starting move for Motor "); Serial.print(motorIndex + 1);
  Serial.print(" Steps: "); Serial.print(validatedSteps);
  Serial.print(" Speed: "); Serial.println(validatedSpeed);  // Show validated speed
  
  // Debug: Check DIAG pin immediately after enabling driver
  delay(10); // Short delay to let driver stabilize
  Serial.print("POST-ENABLE Debug - DIAG"); Serial.print(motorIndex + 1); 
  Serial.print(": "); Serial.println(digitalRead(diagPin));
}

void updateMotors() {
  unsigned long now = micros();
  for (int i = 0; i < 3; ++i) {
    if (motors[i].moving) {      if (motorStalled[i]) {
        // Motor stalled during movement - update position with partial movement
        long partialSteps = motors[i].direction == HIGH ? motors[i].currentStep : -motors[i].currentStep;
        updatePosition(i, partialSteps);
        
        motors[i].moving = false;
        digitalWrite(motors[i].enPin, HIGH); // Disable driver
        Serial.print("Motor "); Serial.print(i + 1); 
        Serial.println(" movement stopped due to stall detection");      }else if (!emergencyStopActive) {
        unsigned long delay_us = 1000000L / motors[i].speed_sps;
        if (now - motors[i].lastStepTime >= delay_us) {
          digitalWrite(motors[i].stepPin, HIGH);
          delayMicroseconds(5); // Short pulse
          digitalWrite(motors[i].stepPin, LOW);
          motors[i].currentStep++;
          motors[i].lastStepTime = now;          if (motors[i].currentStep >= motors[i].targetSteps) {
            motors[i].moving = false;
            
            // Update position tracking for completed move
            long actualSteps = motors[i].direction == HIGH ? motors[i].targetSteps : -motors[i].targetSteps;
            updatePosition(i, actualSteps);
            
            Serial.print("Move complete for M"); Serial.println(i + 1);
            
            // Check if we're in batch mode and have more instructions
            if (batchActive && currentInstruction[i] < instructionCount[i] - 1) {
              // Move to next instruction for this motor              currentInstruction[i]++;
              Instruction nextInstr = instructionQueue[i][currentInstruction[i]];
                // Validate speed for next instruction
              int validatedSpeed = validateSpeed(i, nextInstr.speed);
              
              // Validate position for next instruction
              long validatedSteps;
              if (!validatePosition(i, nextInstr.steps, validatedSteps)) {
                Serial.print("M"); Serial.print(i + 1); 
                Serial.println(" next instruction skipped due to position limit");
                // Skip this instruction and try the next one
                continue;
              }
              
              // Start the next movement immediately
              motors[i].targetSteps = abs(validatedSteps);
              motors[i].currentStep = 0;
              motors[i].speed_sps = validatedSpeed;
              motors[i].lastStepTime = micros();
              motors[i].moving = true;
              motors[i].direction = (validatedSteps > 0) ? HIGH : LOW;
              digitalWrite(motors[i].dirPin, motors[i].direction);
              // Keep driver enabled for next move
              
              Serial.print("M"); Serial.print(i + 1); 
              Serial.print(" starting next instruction: "); Serial.print(validatedSteps);
              Serial.print(" steps at "); Serial.print(validatedSpeed); Serial.println(" sps");
            } else {
              // No more instructions or not in batch mode - disable driver
              digitalWrite(motors[i].enPin, HIGH);
              
              // Check if all motors have completed their instruction queues
              if (batchActive) {
                bool allComplete = true;
                for (int j = 0; j < 3; j++) {
                  if (motors[j].moving || currentInstruction[j] < instructionCount[j] - 1) {
                    allComplete = false;
                    break;
                  }
                }
                if (allComplete) {
                  batchActive = false;
                  Serial.println("=== BATCH EXECUTION COMPLETE ===");
                  Serial.println("All motors have finished their instruction sequences.");
                }
              }
            }
          }
        }
      }
    }
    // Emergency stop handling
    if (emergencyStopActive && motors[i].moving) {
      motors[i].moving = false;
      digitalWrite(motors[i].enPin, HIGH); // Disable all drivers
      Serial.print("Motor "); Serial.print(i + 1); 
      Serial.println(" stopped due to emergency stop");
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
    }    if (command.equalsIgnoreCase("help")) {
      Serial.println("========================================");
      Serial.println("           MOTOR CONTROL HELP");
      Serial.println("========================================");
      Serial.println();      Serial.println("=== BASIC MOTOR COMMANDS ===");
      Serial.println("m[num] p[steps] s[speed]  - Move motor with steps and speed");
      Serial.println("                            Example: m1 p2000 s800");
      Serial.println("m[num] p[steps] t[time]   - Move motor with steps in specified time");
      Serial.println("                            Example: m1 p2000 t5000 (2000 steps in 5 seconds)");
      Serial.println("m[num] t[threshold]       - Set StallGuard threshold (0-255)");
      Serial.println("                            Example: m1 t100");
      Serial.println();      
      Serial.println("=== EMERGENCY & CONTROL ===");
      Serial.println("stop                      - Emergency stop all motors");
      Serial.println("resetstalls               - Clear stall flags and re-enable motors");
      Serial.println("enable1/2/3               - Enable specific motor driver");
      Serial.println("disable1/2/3              - Disable specific motor driver");
      Serial.println("enableall                 - Enable all motor drivers");
      Serial.println("disableall                - Disable all motor drivers");
      Serial.println();      Serial.println("=== BATCH INSTRUCTION SYSTEM ===");
      Serial.println("add m[num] p[steps] s[speed] - Queue instruction for motor");
      Serial.println("                               Example: add m1 p1000 s500");
      Serial.println("add m[num] p[steps] t[time]  - Queue timed instruction for motor");
      Serial.println("                               Example: add m1 p1000 t2000 (1000 steps in 2 seconds)");
      Serial.println("show                      - Display all queued instructions");
      Serial.println("run                       - Execute all queued instructions");
      Serial.println("clear                     - Clear all instruction queues");
      Serial.println();
      Serial.println("=== POSITION TRACKING ===");
      Serial.println("positions                 - Show current positions of all motors");
      Serial.println("home m[num]               - Set current position as home (0.0mm)");
      Serial.println("                            Example: home m1");
      Serial.println("cal m[num] f[mm/step]     - Set calibration factor");
      Serial.println("                            Example: cal m1 f0.0125");      Serial.println("goto m[num] p[mm] s[speed] - Move to absolute position");
      Serial.println("                            Example: goto m1 p25.5 s1000");
      Serial.println("goto m[num] p[mm] t[time]  - Move to absolute position in specified time");
      Serial.println("                            Example: goto m1 p25.5 t3000 (to 25.5mm in 3 seconds)");
      Serial.println();
      Serial.println("=== SPEED MANAGEMENT ===");
      Serial.println("setspeed m[num] s[speed]  - Set maximum speed limit for motor");
      Serial.println("                            Example: setspeed m1 s5000");
      Serial.println("showspeeds                - Display current speed limits");
      Serial.println();
      Serial.println("=== POSITION LIMITS ===");
      Serial.println("showlimits                - Display current position limits");
      Serial.println("setlimit m[num] l[steps]  - Set position limit for motor");
      Serial.println("                            Example: setlimit m1 l20000");
      Serial.println();
      Serial.println("=== DIAGNOSTICS & DEBUG ===");
      Serial.println("status                    - Show system status and motor states");
      Serial.println("diag1/2/3                 - Read DIAG pin values");
      Serial.println("sgr1/2/3                  - Read StallGuard results");
      Serial.println("stalloff/stallon          - Disable/enable stall detection");
      Serial.println("noisecount                - Show false interrupt counts");
      Serial.println("resetnoise                - Reset false interrupt counters");
      Serial.println();
      Serial.println("=== TEST COMMANDS ===");
      Serial.println("testmove                  - Test move without stall detection");
      Serial.println("testall                   - Test all three motors simultaneously");
      Serial.println();      Serial.println("=== NOTES ===");
      Serial.println("- Motor numbers: 1, 2, or 3");
      Serial.println("- Speed in steps per second (sps)");
      Serial.println("- Time in milliseconds (ms)");
      Serial.println("- Position in millimeters (mm)");
      Serial.println("- Default calibration: 0.0125mm per step");
      Serial.println("- Motors must be homed before absolute positioning");
      Serial.println("- Position limits prevent travel beyond ±14,000 steps from home");
      Serial.println("- Time-based commands automatically calculate speed from distance and time");
      Serial.println("========================================");
      return;
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
    }    if (command.equalsIgnoreCase("disable3")) {
      digitalWrite(EN_PIN_3, HIGH); // Disable driver 3
      Serial.println("Motor 3 disabled.");
      return;
    }

    if (command.equalsIgnoreCase("enableall")) {
      if (!emergencyStopActive) {
        digitalWrite(EN_PIN_1, LOW); // Enable all drivers (active LOW)
        digitalWrite(EN_PIN_2, LOW);
        digitalWrite(EN_PIN_3, LOW);
        Serial.println("All motors enabled.");
      } else {
        Serial.println("Cannot enable motors during emergency stop.");
      }
      return;
    }

    if (command.equalsIgnoreCase("disableall")) {
      digitalWrite(EN_PIN_1, HIGH); // Disable all drivers
      digitalWrite(EN_PIN_2, HIGH);
      digitalWrite(EN_PIN_3, HIGH);
      Serial.println("All motors disabled.");
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
    }    if (command.equalsIgnoreCase("diag3")) {
      Serial.print("DIAG3: ");
      Serial.println(digitalRead(DIAG_PIN_3));
      return;
    }    if (command.equalsIgnoreCase("sgr1")) {
      Serial.print("StallGuard Result Motor 1: ");
      Serial.println(driver1.SG_RESULT());
      return;
    }

    if (command.equalsIgnoreCase("sgr2")) {
      Serial.print("StallGuard Result Motor 2: ");
      Serial.println(driver2.SG_RESULT());
      return;
    }    if (command.equalsIgnoreCase("sgr3")) {
      Serial.print("StallGuard Result Motor 3: ");
      Serial.println(driver3.SG_RESULT());
      return;
    }

    if (command.equalsIgnoreCase("status")) {
      Serial.println("=== System Status ===");
      Serial.print("Emergency Stop: "); Serial.println(emergencyStopActive ? "ACTIVE" : "CLEAR");
      for (int i = 0; i < 3; i++) {
        Serial.print("Motor "); Serial.print(i+1); 
        Serial.print(" - Stalled: "); Serial.print(motorStalled[i] ? "YES" : "NO");
        Serial.print(", Moving: "); Serial.print(motors[i].moving ? "YES" : "NO");
        Serial.print(", EN Pin: "); Serial.println(digitalRead(motors[i].enPin) == LOW ? "ENABLED" : "DISABLED");
      }
      Serial.print("DIAG Pins - 1: "); Serial.print(digitalRead(DIAG_PIN_1));
      Serial.print(", 2: "); Serial.print(digitalRead(DIAG_PIN_2));
      Serial.print(", 3: "); Serial.println(digitalRead(DIAG_PIN_3));
      Serial.print("StallGuard Results - 1: "); Serial.print(driver1.SG_RESULT());
      Serial.print(", 2: "); Serial.print(driver2.SG_RESULT());      Serial.print(", 3: "); Serial.println(driver3.SG_RESULT());
      return;
    }

    if (command.equalsIgnoreCase("stalloff")) {
      detachInterrupt(digitalPinToInterrupt(DIAG_PIN_1));
      detachInterrupt(digitalPinToInterrupt(DIAG_PIN_2));
      detachInterrupt(digitalPinToInterrupt(DIAG_PIN_3));
      Serial.println("Stall detection interrupts DISABLED");
      return;
    }    if (command.equalsIgnoreCase("stallon")) {
      attachInterrupt(digitalPinToInterrupt(DIAG_PIN_1), stallISR1, RISING);
      attachInterrupt(digitalPinToInterrupt(DIAG_PIN_2), stallISR2, RISING);
      attachInterrupt(digitalPinToInterrupt(DIAG_PIN_3), stallISR3, RISING);
      Serial.println("Stall detection interrupts ENABLED");
      return;
    }    if (command.equalsIgnoreCase("testmove")) {
      // Temporarily disable stall detection for testing
      detachInterrupt(digitalPinToInterrupt(DIAG_PIN_1));
      detachInterrupt(digitalPinToInterrupt(DIAG_PIN_2));
      detachInterrupt(digitalPinToInterrupt(DIAG_PIN_3));
      Serial.println("Stall detection DISABLED for testing. Starting test move...");
      startMotorMove(0, 100, 400); // Small test move for motor 1
      return;
    }

    if (command.equalsIgnoreCase("noisecount")) {
      Serial.println("=== False Interrupt Counts ===");
      Serial.print("Motor 1: "); Serial.println(falseInterruptCount[0]);
      Serial.print("Motor 2: "); Serial.println(falseInterruptCount[1]);
      Serial.print("Motor 3: "); Serial.println(falseInterruptCount[2]);
      return;
    }

    if (command.equalsIgnoreCase("resetnoise")) {
      falseInterruptCount[0] = 0;
      falseInterruptCount[1] = 0;
      falseInterruptCount[2] = 0;
      Serial.println("False interrupt counters reset.");
      return;
    }

    if (command.equalsIgnoreCase("testall")) {
      // Test all three motors simultaneously
      Serial.println("Starting simultaneous test of all three motors...");
      if (emergencyStopActive) {
        Serial.println("Cannot test motors during emergency stop.");
        return;
      }
      
      // Check if any motors are stalled
      bool anyStalled = false;
      for (int i = 0; i < 3; i++) {
        if (motorStalled[i]) {
          Serial.print("Motor "); Serial.print(i+1); Serial.println(" is stalled. Reset stalls first.");
          anyStalled = true;
        }
      }
      
      if (!anyStalled) {
        // Start all three motors with moderate movements
        startMotorMove(0, 200, 400);  // Motor 1: 200 steps at 400 sps
        startMotorMove(1, 300, 500);  // Motor 2: 300 steps at 500 sps  
        startMotorMove(2, 250, 450);  // Motor 3: 250 steps at 450 sps
        Serial.println("All three motors started with different step counts and speeds.");
        Serial.println("Use 'status' command to monitor progress.");
      }
      return;
    }

    // Batch instruction commands
    if (command.equalsIgnoreCase("clear")) {
      clearInstructions();
      return;
    }
    
    if (command.equalsIgnoreCase("show")) {
      showInstructions();
      return;
    }
      if (command.equalsIgnoreCase("run")) {
      executeBatch();
      return;
    }
    
    // Speed limit commands
    if (command.equalsIgnoreCase("showspeeds")) {
      Serial.println("=== Current Speed Limits ===");
      for (int i = 0; i < 3; i++) {
        Serial.print("Motor "); Serial.print(i + 1); 
        Serial.print(": "); Serial.print(maxSpeed[i]); Serial.println(" sps");
      }
      return;
    }
    
    // Parse setspeed command: setspeed m[motor] s[speed]
    int speedMotorNum = 0;
    int newMaxSpeed = 0;
    int parsed_speed = sscanf(command.c_str(), "setspeed m%d s%d", &speedMotorNum, &newMaxSpeed);
    
    if (parsed_speed == 2 && speedMotorNum >= 1 && speedMotorNum <= 3) {
      if (newMaxSpeed <= 0) {
        Serial.println("Speed must be positive");
        return;
      }
      if (newMaxSpeed > 10000) {
        Serial.println("Warning: Speed limit above 10,000 sps may damage your device!");
        Serial.print("Setting maximum to 10,000 sps for safety. Use values <= 3000 for normal operation.");
        newMaxSpeed = 10000;
      }
      
      maxSpeed[speedMotorNum - 1] = newMaxSpeed;
      Serial.print("Motor "); Serial.print(speedMotorNum); 
      Serial.print(" maximum speed set to "); Serial.print(newMaxSpeed); Serial.println(" sps");
      return;
    }
    
    // Position limit commands
    if (command.equalsIgnoreCase("showlimits")) {
      Serial.println("=== Current Position Limits ===");
      for (int i = 0; i < 3; i++) {
        Serial.print("Motor "); Serial.print(i + 1); 
        Serial.print(": ±"); Serial.print(positionLimit[i]); Serial.println(" steps");
      }
      return;
    }
    
    // Parse setlimit command: setlimit m[motor] l[steps]
    int limitMotorNum = 0;
    long newPositionLimit = 0;
    int parsed_limit = sscanf(command.c_str(), "setlimit m%d l%ld", &limitMotorNum, &newPositionLimit);
    
    if (parsed_limit == 2 && limitMotorNum >= 1 && limitMotorNum <= 3) {
      if (newPositionLimit <= 0) {
        Serial.println("Position limit must be positive");
        return;
      }
      if (newPositionLimit > 50000) {
        Serial.println("Warning: Position limit above 50,000 steps may be unsafe!");
        Serial.println("Consider using values <= 20,000 for normal operation.");
      }
      
      positionLimit[limitMotorNum - 1] = newPositionLimit;
      Serial.print("Motor "); Serial.print(limitMotorNum); 
      Serial.print(" position limit set to ±"); Serial.print(newPositionLimit); Serial.println(" steps");
      return;
    }
      // Parse time-based add command first: add m[num] p[steps] t[time_ms]
    int addTimeMotorNum = 0;
    long addTimeSteps = 0;
    unsigned long addTime_ms = 0;
    int parsed_add_time = sscanf(command.c_str(), "add m%d p%ld t%lu", &addTimeMotorNum, &addTimeSteps, &addTime_ms);
    
    if (parsed_add_time == 3 && addTimeMotorNum >= 1 && addTimeMotorNum <= 3) {
        // Handle time-based add command: add m[num] p[steps] t[time_ms]
        int calculatedSpeed = calculateSpeedFromTime(addTimeSteps, addTime_ms);
        int validatedSpeed = validateSpeed(addTimeMotorNum - 1, calculatedSpeed);
        
        Serial.print("Time-based instruction - Motor: "); Serial.print(addTimeMotorNum);
        Serial.print(" Steps: "); Serial.print(addTimeSteps);
        Serial.print(" Time: "); Serial.print(addTime_ms); Serial.print("ms");
        Serial.print(" Calculated Speed: "); Serial.println(validatedSpeed);
        
        addInstruction(addTimeMotorNum - 1, addTimeSteps, validatedSpeed);
        return;
    }

    // Parse regular add command: add m[motor] p[steps] s[speed]
    int addMotorNum = 0;
    long addSteps = 0;
    int addSpeed = 800;
    int parsed_add = sscanf(command.c_str(), "add m%d p%ld s%d", &addMotorNum, &addSteps, &addSpeed);
    
    if (parsed_add >= 2 && addMotorNum >= 1 && addMotorNum <= 3) {
      if (parsed_add == 2) {
        addSpeed = 800; // Default speed if not provided
      }
      addInstruction(addMotorNum - 1, addSteps, addSpeed);
      return;
    }

    // Position tracking commands
    if (command.equalsIgnoreCase("positions")) {
      showPositions();
      return;
    }
    
    // Parse home command: home m[num]
    int homeMotorNum = 0;
    int parsed_home = sscanf(command.c_str(), "home m%d", &homeMotorNum);
    if (parsed_home == 1 && homeMotorNum >= 1 && homeMotorNum <= 3) {
      homeMotor(homeMotorNum - 1);
      return;
    }
    
    // Parse calibration command: cal m[num] f[value]
    int calMotorNum = 0;
    float mmPerStep = 0.0;
    int parsed_cal = sscanf(command.c_str(), "cal m%d f%f", &calMotorNum, &mmPerStep);
    if (parsed_cal == 2 && calMotorNum >= 1 && calMotorNum <= 3 && mmPerStep > 0) {
      setPositionCalibration(calMotorNum - 1, mmPerStep);
      return;
    }
    
    // Parse goto command: goto m[num] p[position] s[speed]
    int gotoMotorNum = 0;
    float targetPosition = 0.0;
    int gotoSpeed = 800;
    int parsed_goto = sscanf(command.c_str(), "goto m%d p%f s%d", &gotoMotorNum, &targetPosition, &gotoSpeed);
    if (parsed_goto >= 2 && gotoMotorNum >= 1 && gotoMotorNum <= 3) {
      if (parsed_goto == 2) {
        gotoSpeed = 800; // Default speed if not provided
      }
      
      if (motorStalled[gotoMotorNum-1]) {
        Serial.print("Motor "); Serial.print(gotoMotorNum); Serial.println(" is stalled. Reset stalls to move.");
      } else if (emergencyStopActive) {
        Serial.println("Cannot move motors during emergency stop.");
      } else {
        moveToPosition(gotoMotorNum - 1, targetPosition, gotoSpeed);
      }
      return;
    }    // Parse time-based movement commands
    int timeMotorNum = 0;
    long timeStepsToMove = 0;
    unsigned long moveTime_ms = 0;
    int parsed_time_move = sscanf(command.c_str(), "m%d p%ld t%lu", &timeMotorNum, &timeStepsToMove, &moveTime_ms);
      // Parse time-based goto command: goto m[num] p[position] t[time_ms]
    int gotoTimeMotorNum = 0;
    float gotoTimePosition = 0.0;
    unsigned long gotoTime_ms = 0;
    int parsed_goto_time = sscanf(command.c_str(), "goto m%d p%f t%lu", &gotoTimeMotorNum, &gotoTimePosition, &gotoTime_ms);

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
            driver3.SGTHRS(thresholdValue);        }
        return;
    } else if (parsed_time_move == 3 && timeMotorNum >= 1 && timeMotorNum <= 3) {
        // Handle time-based movement command: m[num] p[steps] t[time_ms]
        int calculatedSpeed = calculateSpeedFromTime(timeStepsToMove, moveTime_ms);
        int validatedSpeed = validateSpeed(timeMotorNum - 1, calculatedSpeed);
        
        Serial.print("Time-based move - Motor: "); Serial.print(timeMotorNum);
        Serial.print(" Steps: "); Serial.print(timeStepsToMove);
        Serial.print(" Time: "); Serial.print(moveTime_ms); Serial.print("ms");
        Serial.print(" Calculated Speed: "); Serial.println(validatedSpeed);

        if (motorStalled[timeMotorNum-1]) {
            Serial.print("Motor "); Serial.print(timeMotorNum); Serial.println(" is stalled. Reset stalls to move.");
        } else if (emergencyStopActive) {
            Serial.println("Cannot move motors during emergency stop.");
        } else {
            startMotorMove(timeMotorNum-1, timeStepsToMove, validatedSpeed);
        }
        return;
    } else if (parsed_goto_time == 3 && gotoTimeMotorNum >= 1 && gotoTimeMotorNum <= 3) {
        // Handle time-based goto command: goto m[num] p[position] t[time_ms]
        if (!position[gotoTimeMotorNum-1].isHomed) {
            Serial.print("Motor "); Serial.print(gotoTimeMotorNum); 
            Serial.println(" not homed. Use 'home m[num]' first.");
            return;
        }
        
        // Calculate steps needed to reach target position
        long targetSteps = (long)(gotoTimePosition / position[gotoTimeMotorNum-1].mmPerStep);
        long stepsToMove = targetSteps - position[gotoTimeMotorNum-1].totalSteps;
        
        int calculatedSpeed = calculateSpeedFromTime(stepsToMove, gotoTime_ms);
        int validatedSpeed = validateSpeed(gotoTimeMotorNum - 1, calculatedSpeed);
        
        Serial.print("Time-based goto - Motor "); Serial.print(gotoTimeMotorNum);
        Serial.print(" from "); Serial.print(position[gotoTimeMotorNum-1].currentPosition_mm, 2);
        Serial.print("mm to "); Serial.print(gotoTimePosition, 2);
        Serial.print("mm in "); Serial.print(gotoTime_ms); Serial.print("ms");
        Serial.print(" ("); Serial.print(stepsToMove); Serial.print(" steps at ");
        Serial.print(validatedSpeed); Serial.println(" sps)");

        if (motorStalled[gotoTimeMotorNum-1]) {
            Serial.print("Motor "); Serial.print(gotoTimeMotorNum); Serial.println(" is stalled. Reset stalls to move.");
        } else if (emergencyStopActive) {
            Serial.println("Cannot move motors during emergency stop.");
        } else {
            startMotorMove(gotoTimeMotorNum-1, stepsToMove, validatedSpeed);        }
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
      }      return;    } else {      Serial.println("Invalid command. Type 'help' for complete command reference.");
      Serial.println("Basic: 'm[num] p[steps] s[speed]' OR 'm[num] p[steps] t[time_ms]' OR 'm[num] t[value]' OR 'resetstalls' OR 'stop'");
      // Flush any remaining input to avoid repeated errors
      while (Serial.available() > 0) {
          Serial.read();
      }
      return;
    }
  }
}

// Batch instruction management functions
void clearInstructions() {
  for (int motor = 0; motor < 3; motor++) {
    instructionCount[motor] = 0;
    currentInstruction[motor] = 0;
    for (int i = 0; i < MAX_INSTRUCTIONS; i++) {
      instructionQueue[motor][i].valid = false;
    }
  }
  batchActive = false;
  Serial.println("All instruction queues cleared.");
}

void addInstruction(int motorIndex, long steps, int speed) {
  if (motorIndex < 0 || motorIndex >= 3) {
    Serial.println("Invalid motor index");
    return;
  }
  
  if (instructionCount[motorIndex] >= MAX_INSTRUCTIONS) {
    Serial.print("Instruction queue full for Motor "); Serial.println(motorIndex + 1);
    return;
  }
    // Validate speed before adding to queue
  int validatedSpeed = validateSpeed(motorIndex, speed);
  
  // Validate position before adding to queue
  long validatedSteps;
  if (!validatePosition(motorIndex, steps, validatedSteps)) {
    Serial.println("Instruction not added due to position limit violation");
    return;
  }
  
  instructionQueue[motorIndex][instructionCount[motorIndex]].steps = validatedSteps;
  instructionQueue[motorIndex][instructionCount[motorIndex]].speed = validatedSpeed;
  instructionQueue[motorIndex][instructionCount[motorIndex]].valid = true;
  instructionCount[motorIndex]++;
  
  Serial.print("Added instruction to M"); Serial.print(motorIndex + 1);
  Serial.print(": "); Serial.print(validatedSteps); Serial.print(" steps at ");
  Serial.print(validatedSpeed); Serial.print(" sps ("); Serial.print(instructionCount[motorIndex]);
  Serial.println(" total instructions)");
}

void showInstructions() {
  Serial.println("=== Current Instruction Queues ===");
  for (int motor = 0; motor < 3; motor++) {
    Serial.print("Motor "); Serial.print(motor + 1); Serial.print(": ");
    if (instructionCount[motor] == 0) {
      Serial.println("No instructions");
    } else {
      Serial.print(instructionCount[motor]); Serial.println(" instructions:");
      for (int i = 0; i < instructionCount[motor]; i++) {
        Serial.print("  "); Serial.print(i + 1); Serial.print(": ");
        Serial.print(instructionQueue[motor][i].steps); Serial.print(" steps at ");
        Serial.print(instructionQueue[motor][i].speed); Serial.println(" sps");
      }
    }
  }
}

void executeBatch() {
  if (emergencyStopActive) {
    Serial.println("Cannot execute batch during emergency stop");
    return;
  }
  
  // Check if any motors are stalled
  for (int i = 0; i < 3; i++) {
    if (motorStalled[i]) {
      Serial.print("Motor "); Serial.print(i + 1); 
      Serial.println(" is stalled. Reset stalls before batch execution.");
      return;
    }
  }
  
  // Check if any motor has instructions
  bool hasInstructions = false;
  for (int motor = 0; motor < 3; motor++) {
    if (instructionCount[motor] > 0) {
      hasInstructions = true;
      break;
    }
  }
  
  if (!hasInstructions) {
    Serial.println("No instructions queued. Use 'add' commands first.");
    return;
  }
  
  Serial.println("=== STARTING BATCH EXECUTION ===");
  showInstructions();
  
  // Reset current instruction pointers
  for (int motor = 0; motor < 3; motor++) {
    currentInstruction[motor] = 0;
  }
  
  batchActive = true;
  
  // Start first instruction for each motor that has instructions
  for (int motor = 0; motor < 3; motor++) {
    if (instructionCount[motor] > 0) {
      Instruction firstInstr = instructionQueue[motor][0];
      startMotorMove(motor, firstInstr.steps, firstInstr.speed);
    }
  }
}