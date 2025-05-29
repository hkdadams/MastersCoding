#include <SimpleFOC.h>

// ——— Hardware pins ———
#define PWMA    4    // phase U
#define PWMB    5    // phase V
#define PWMC    6    // phase W
#define EN_GATE 25    // enable gate (active HIGH)
#define ENC_CS  14   // AS5048A chip-select, SPI(MISO=12, SCK=13)

// ——— Motor/driver/encoder setup ———
BLDCDriver3PWM driver(PWMA, PWMB, PWMC, EN_GATE);
MagneticSensorSPI sensor = MagneticSensorSPI(ENC_CS, 14, 0x3FFF);
BLDCMotor motor = BLDCMotor(7);  // adjust pole-pairs for your GBM3506


// ——— Control parameters ———
const float    nominalVoltage = 12.0;  // your supply
const unsigned focFrequency  = 25000;   // FOC loop Hz

// ——— State ———
String    commandLine = "";
bool      newCommand  = false;

// ——— Safety and control state ———
volatile bool emergencyStopActive = false;
bool motorEnabled = true;
const float MIN_POSITION = -6.28;  // -2π radians
const float MAX_POSITION = 6.28;   // +2π radians
const float MAX_VELOCITY = 10.0;   // Maximum velocity in rad/s (matches motor.velocity_limit)
float positionLimit = 6.28;        // ±2π radians (one full rotation)
float maxVelocity = 10.0;          // Maximum velocity in rad/s (matches motor.velocity_limit)
float currentTarget = 0.0;         // Current target position

// ——— Motion profiling state ———
unsigned long moveStartTime = 0;
float moveStartPosition = 0;
float moveTargetPosition = 0;
float moveDuration = 0;
bool inTimedMove = false;

// ——— Debugging state ———
bool debugMode = false;
unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL = 1000; // Debug output every 1 second

// ——— AS5048A Register definitions for debugging ———
#define AS5048A_PROG_REG        0x03
#define AS5048A_DIAAGC_REG      0x3FFC
#define AS5048A_MAG_REG         0x3FFD
#define AS5048A_ANGLE_REG       0x3FFE
#define AS5048A_ERROR_REG       0x0001
#define AS5048A_PARITY_BIT      0x8000
#define AS5048A_ERROR_MASK      0x4000

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Teensy4.0 + SimpleFOC Mini gimbal control");

  // SPI init for encoder
  SPI.begin();
  sensor.init();
  motor.linkSensor(&sensor);

  // driver init
  driver.voltage_power_supply = nominalVoltage;
  driver.init();
  pinMode(EN_GATE, OUTPUT);
  digitalWrite(EN_GATE, HIGH);  // EN_GATE is active HIGH
  Serial.println("Motor driver enabled (EN_GATE = HIGH)");
  delay(100);  // Allow driver to stabilize
  motor.linkDriver(&driver);
  
  // motor FOC settings - BALANCED FOR CONTROL AND SAFETY
  motor.voltage_sensor_align = 2;      // Slightly higher for better alignment
  motor.voltage_limit        = 6;      // Increased to 6V for better torque while safe
  motor.velocity_limit       = 10;     // Increased for better response
  
  // Improved PID settings for actual motion control
  motor.PID_velocity.P       = 0.5;    // Increased for better response
  motor.PID_velocity.I       = 5.0;    // Moderate integral term
  motor.PID_velocity.D       = 0.005;  // Small D term for stability
  motor.PID_velocity.output_ramp = 1000; // Limit rate of change
  motor.PID_velocity.limit   = motor.voltage_limit; // Limit PID output
  
  // Position control PID (for angle mode)
  motor.P_angle.P            = 15;     // Higher for better position tracking
  motor.P_angle.I            = 0;      // No integral for position
  motor.P_angle.D            = 0.3;    // Higher derivative for damping
  motor.P_angle.output_ramp  = 0;      // No limit on position output
    motor.LPF_velocity.Tf      = 0.01;   // Faster filtering
  motor.controller           = MotionControlType::angle;  // Position control mode
  
  // Initialize FOC with comprehensive error checking
  Serial.println("Initializing FOC...");
  motor.enable();
  
  // Test encoder before FOC init
  float testAngle = sensor.getAngle();
  if (isnan(testAngle)) {
    Serial.println("❌ ERROR: Encoder not responding! Check connections.");
    while(1) delay(1000); // Stop here if encoder fails
  } else {
    Serial.printf("✅ Encoder working: %.3f rad\n", testAngle);
  }
  
  // Initialize FOC
  if (motor.initFOC() == 1) {
    Serial.println("✅ FOC initialization successful!");
  } else {
    Serial.println("❌ ERROR: FOC initialization failed!");
    Serial.println("Possible causes:");
    Serial.println("- Encoder not properly connected");
    Serial.println("- Motor phases not connected correctly");
    Serial.println("- Motor driver not enabled");
    Serial.println("- Insufficient power supply");
    while(1) delay(1000); // Stop here if FOC fails
  }

  // start the FOC & motion loops
  motor.target   = 0;
  motor.useMonitoring(Serial);
    Serial.println("SimpleFOC initialization complete!");
  Serial.printf("Initial position: %.3f rad\n", motor.shaftAngle());
  Serial.println("Commands available:");
  Serial.println("  pos <angle_rad> vel <rad/s> - Move to angle with max velocity");
  Serial.println("  pos <angle_rad> time <sec>  - Move to angle in specified time");
  Serial.println("  force <angle_rad>           - Force move (bypass safety checks)");
  Serial.println("  stop                        - Emergency stop");
  Serial.println("  enable                      - Enable motor");
  Serial.println("  disable                     - Disable motor");
  Serial.println("  status                      - Show current status");
  Serial.println("  check                       - Comprehensive motor diagnostics");
  Serial.println("  voltage [V]                 - Set/show voltage limit");
  Serial.println("  gains [P] [I] [D]          - Set/show PID gains");
  Serial.println("  debug                       - Toggle debug output");
  Serial.println("  enctest                     - Test encoder communication");
  Serial.println("  reset                       - Reset motor position to zero");
}

void loop() {
  // Handle emergency stop first
  if (emergencyStopActive) {
    motor.disable();
    Serial.println("EMERGENCY STOP ACTIVE! Send 'enable' to resume operation.");
    while (emergencyStopActive) {
      if (Serial.available()) {
        commandLine = Serial.readStringUntil('\n');
        commandLine.trim();
        if (commandLine.equalsIgnoreCase("enable")) {
          emergencyStopActive = false;
          motorEnabled = true;
          motor.enable();
          Serial.println("Emergency stop cleared. Motor enabled.");
          break;
        }
      }
      delay(100);
    }
    return;
  }

  // Only run FOC if motor is enabled
  if (motorEnabled) {
    // Handle timed moves
    if (inTimedMove) {
      unsigned long currentTime = millis();
      float elapsed = (currentTime - moveStartTime) / 1000.0; // Convert to seconds
      
      if (elapsed >= moveDuration) {
        // Move complete
        motor.target = moveTargetPosition;
        inTimedMove = false;
        Serial.printf("Timed move complete. Position: %.3f rad\n", motor.shaftAngle());
      } else {
        // Interpolate position based on time
        float progress = elapsed / moveDuration;
        float currentPos = moveStartPosition + (moveTargetPosition - moveStartPosition) * progress;
        motor.target = currentPos;
      }
    }
    
    // Run FOC control loop
    motor.loopFOC();
    motor.move();
  }
  // Handle serial commands
  if (Serial.available()) {
    commandLine = Serial.readStringUntil('\n');
    commandLine.trim();
    if (commandLine.length()) {
      Serial.printf("DEBUG: Got input: '%s' (length=%d)\n", commandLine.c_str(), commandLine.length());
      newCommand = true;
    }
  }

  // Process new command
  if (newCommand) {
    handleCommand(commandLine);
    newCommand = false;
  }

  // Debugging output for encoder values
  if (debugMode && (millis() - lastDebugTime >= DEBUG_INTERVAL)) {
    lastDebugTime = millis();
    printEncoderDebugInfo();
  }
}

// ——— Enhanced command handling similar to stepper system ———
void handleCommand(const String &cmd) {
  Serial.print("Received: "); Serial.println(cmd);
  
  if (cmd.length() == 0) {
    return; // Ignore empty input
  }

  // Emergency stop command
  if (cmd.equalsIgnoreCase("stop")) {
    emergencyStopActive = true;
    inTimedMove = false;
    Serial.println("Emergency stop activated!");
    return;
  }

  // Enable/disable commands
  if (cmd.equalsIgnoreCase("enable")) {
    if (!emergencyStopActive) {
      motorEnabled = true;
      motor.enable();
      Serial.println("Motor enabled.");
    } else {
      Serial.println("Cannot enable motor during emergency stop.");
    }
    return;
  }

  if (cmd.equalsIgnoreCase("disable")) {
    motorEnabled = false;
    motor.disable();
    inTimedMove = false;
    Serial.println("Motor disabled.");
    return;
  }

  // Status command
  if (cmd.equalsIgnoreCase("status")) {
    Serial.println("=== Gimbal Motor Status ===");
    Serial.printf("Current Position: %.3f rad (%.1f deg)\n", motor.shaftAngle(), motor.shaftAngle() * 180.0 / PI);
    Serial.printf("Target Position: %.3f rad (%.1f deg)\n", motor.target, motor.target * 180.0 / PI);
    Serial.printf("Current Velocity: %.3f rad/s\n", motor.shaftVelocity());
    Serial.printf("Voltage Limit: %.2f V\n", motor.voltage_limit);
    Serial.printf("Velocity Limit: %.2f rad/s\n", motor.velocity_limit);
    Serial.printf("Motor Enabled: %s\n", motorEnabled ? "YES" : "NO");
    Serial.printf("Emergency Stop: %s\n", emergencyStopActive ? "ACTIVE" : "CLEAR");
    Serial.printf("In Timed Move: %s\n", inTimedMove ? "YES" : "NO");
    return;
  }

  // Help command
  if (cmd.equalsIgnoreCase("help")) {
    Serial.println("========================================");
    Serial.println("         GIMBAL MOTOR CONTROL HELP");
    Serial.println("========================================");
    Serial.println();
    Serial.println("=== BASIC COMMANDS ===");
    Serial.println("pos <rad> vel <rad/s>  - Move to position with max velocity");
    Serial.println("                         Example: pos 1.57 vel 2.0");
    Serial.println("pos <rad> time <sec>   - Move to position in specified time");
    Serial.println("                         Example: pos 3.14 time 2.5");
    Serial.println();
    Serial.println("=== CONTROL COMMANDS ===");
    Serial.println("stop                   - Emergency stop");
    Serial.println("enable                 - Enable motor");
    Serial.println("disable                - Disable motor");
    Serial.println("status                 - Show current status");
    Serial.println("help                   - Show this help");
    Serial.println();
    Serial.println("=== NOTES ===");
    Serial.println("- Angles in radians (π ≈ 3.14159)");
    Serial.println("- Position limit: ±6.28 rad (±2π)");
    Serial.println("- Max velocity: 5 rad/s");  // Updated to match actual limit
    Serial.println("========================================");
    return;
  }

  // Parse position commands
  float pos = 0, vel = 0, t = 0;
  
  // Try "pos <pos> vel <vel>" format
  int n = sscanf(cmd.c_str(), "pos %f vel %f", &pos, &vel);
  if (n == 2) {
    if (!motorEnabled) {
      Serial.println("Motor is disabled. Use 'enable' first.");
      return;
    }
    if (emergencyStopActive) {
      Serial.println("Cannot move during emergency stop.");
      return;
    }
    
    // Validate position limits
    if (abs(pos) > positionLimit) {
      Serial.printf("Position %.3f rad exceeds limit of ±%.2f rad\n", pos, positionLimit);
      return;
    }
      // Validate velocity limits
    if (abs(vel) > maxVelocity) {
      Serial.printf("Velocity %.3f rad/s exceeds limit of %.2f rad/s\n", abs(vel), maxVelocity);
      vel = (vel > 0) ? maxVelocity : -maxVelocity;
      Serial.printf("Limited to %.2f rad/s\n", vel);
    }
    
    // Set velocity limit for this move only, but ensure minimum effective velocity
    float effectiveVel = max(abs(vel), 2.0);  // Minimum 2.0 rad/s for reliable movement
    motor.velocity_limit = effectiveVel;
    motor.target = pos;
    currentTarget = pos;
    inTimedMove = false;
    
    Serial.printf("→ Moving to %.3f rad (%.1f deg) @ %.3f rad/s (effective: %.3f rad/s)\n", 
                  pos, pos * 180.0 / PI, vel, effectiveVel);
    return;
  }
  
  // Try "pos <pos> time <time>" format
  n = sscanf(cmd.c_str(), "pos %f time %f", &pos, &t);
  if (n == 2 && t > 0) {
    if (!motorEnabled) {
      Serial.println("Motor is disabled. Use 'enable' first.");
      return;
    }
    if (emergencyStopActive) {
      Serial.println("Cannot move during emergency stop.");
      return;
    }
    
    // Validate position limits
    if (abs(pos) > positionLimit) {
      Serial.printf("Position %.3f rad exceeds limit of ±%.2f rad\n", pos, positionLimit);
      return;
    }
    
    float cur = motor.shaftAngle();
    float delta = pos - cur;
    float vcmd = abs(delta) / t;
    
    // Validate calculated velocity
    if (vcmd > maxVelocity) {
      Serial.printf("Required velocity %.3f rad/s exceeds limit of %.2f rad/s\n", vcmd, maxVelocity);
      Serial.printf("Minimum time for this move: %.2f seconds\n", abs(delta) / maxVelocity);
      return;
    }
    
    // Set up timed move
    motor.velocity_limit = vcmd;
    moveStartTime = millis();
    moveStartPosition = cur;
    moveTargetPosition = pos;
    moveDuration = t;
    inTimedMove = true;
    currentTarget = pos;
    
    Serial.printf("→ Timed move from %.3f rad to %.3f rad in %.2fs (vel=%.3f rad/s)\n", 
                  cur, pos, t, vcmd);
    return;
  }
  // Debug and diagnostic commands
  if (cmd.equalsIgnoreCase("debug")) {
    debugMode = !debugMode;
    Serial.print("Debug mode ");
    Serial.println(debugMode ? "enabled" : "disabled");
    return;
  }
  
  if (cmd.equalsIgnoreCase("enctest")) {
    testEncoderCommunication();
    return;
  }
  
  if (cmd.equalsIgnoreCase("encraw")) {
    readRawEncoderValues();
    return;
  }
    if (cmd.equalsIgnoreCase("encregs")) {
    readAllEncoderRegisters();
    return;
  }
    // Try "force <pos>" format
  n = sscanf(cmd.c_str(), "force %f", &pos);
  if (n == 1) {
    Serial.printf("FORCE MOVE to %.3f rad (bypassing safety checks)\n", pos);
    motor.target = pos;
    currentTarget = pos;
    inTimedMove = false; // Cancel any timed move
    Serial.printf("Target set. Position error: %.3f rad\n", pos - motor.shaftAngle());
    return;
  }
  
  // Try "voltage <value>" format
  n = sscanf(cmd.c_str(), "voltage %f", &vel);
  if (n == 1) {
    if (vel >= 1.0 && vel <= 12.0) {
      motor.voltage_limit = vel;
      Serial.printf("Voltage limit set to %.1f V\n", vel);
    } else {
      Serial.println("Voltage must be between 1.0 and 12.0 V");
    }
    return;
  }
    if (cmd.equalsIgnoreCase("voltage")) {
    Serial.printf("Current voltage limit: %.1f V\n", motor.voltage_limit);
    return;
  }
  
  // Velocity limit command
  if (cmd.equalsIgnoreCase("velreset")) {
    motor.velocity_limit = maxVelocity;
    Serial.printf("Velocity limit reset to %.1f rad/s\n", maxVelocity);
    return;
  }
  
  // Try "gains <P> <I> <D>" format
  float p_gain, i_gain, d_gain;
  n = sscanf(cmd.c_str(), "gains %f %f %f", &p_gain, &i_gain, &d_gain);
  if (n == 3) {
    motor.PID_velocity.P = p_gain;
    motor.PID_velocity.I = i_gain;
    motor.PID_velocity.D = d_gain;
    Serial.printf("PID gains set: P=%.3f, I=%.3f, D=%.3f\n", p_gain, i_gain, d_gain);
    return;
  }
    if (cmd.equalsIgnoreCase("gains")) {
    Serial.printf("Current PID gains: P=%.3f, I=%.3f, D=%.3f\n", 
                  motor.PID_velocity.P, motor.PID_velocity.I, motor.PID_velocity.D);
    return;
  }
  
  if (cmd.equalsIgnoreCase("limits")) {
    showLimits();
    return;
  }
  
  if (cmd.equalsIgnoreCase("reset")) {
    resetMotorPosition();
    return;
  }
  
  if (cmd.equalsIgnoreCase("check")) {
    checkMotorControl();
    return;
  }
  // If we get here, command was not recognized
  Serial.println("Invalid command. Use:");
  Serial.println("  pos <rad> vel <rad/s>  OR  pos <rad> time <sec>");
  Serial.println("  force <rad>                - Force move (bypass safety)");
  Serial.println("  voltage [V]               - Set/show voltage limit");
  Serial.println("  velreset                  - Reset velocity limit to maximum");
  Serial.println("  gains [P] [I] [D]         - Set/show PID gains");
  Serial.println("  stop, enable, disable, status, help");
  Serial.println("  debug, enctest, encraw, encregs, limits");
  Serial.println("  reset, check");
}

// ——— Command handling functions ———

// Enhanced help function
void printHelp() {
  Serial.println("\n=== GIMBAL CONTROL COMMANDS ===");
  Serial.println("pos <rad> vel <rad/s>  - Move to position with velocity");
  Serial.println("pos <rad> time <sec>   - Move to position in time");
  Serial.println("enable                 - Enable motor");
  Serial.println("disable                - Disable motor");
  Serial.println("stop                   - Emergency stop");
  Serial.println("status                 - Show current status");
  Serial.println("help                   - Show this help");
  Serial.println("debug                  - Toggle debug mode");
  Serial.println("enctest                - Test encoder communication");
  Serial.println("encraw                 - Read raw encoder values");
  Serial.println("encregs                - Read all encoder registers");
  Serial.println("limits                 - Show position/velocity limits");
  Serial.println("reset                  - Reset motor position to zero");
  Serial.println("check                  - Check motor control diagnostics");
  Serial.println("\nLimits: Position ±6.28 rad, Velocity ±5 rad/s");
}

// ——— Encoder debugging functions ———

// Read raw SPI data from AS5048A - Compatible with SimpleFOC
uint16_t readEncoderRaw(uint16_t address) {
  // Save current SPI settings
  SPISettings currentSettings = SPISettings(1000000, MSBFIRST, SPI_MODE1);
  
  // Use SimpleFOC compatible SPI settings
  SPI.beginTransaction(currentSettings);
  
  // Prepare command - AS5048A uses 16-bit commands
  uint16_t command = address;
  
  // Calculate parity bit
  uint8_t parity = 0;
  for (int i = 0; i < 15; i++) {
    if ((command >> i) & 1) parity++;
  }
  if (parity % 2 == 1) command |= 0x8000;  // Set parity bit if odd
  
  // SPI transaction
  digitalWrite(ENC_CS, LOW);
  delayMicroseconds(2);  // AS5048A needs minimum 1µs
  uint16_t result = SPI.transfer16(command);
  delayMicroseconds(2);
  digitalWrite(ENC_CS, HIGH);
  
  SPI.endTransaction();
  delayMicroseconds(10);  // Allow settling time
  
  return result;
}

// Test encoder communication
void testEncoderCommunication() {
  Serial.println("\n=== ENCODER COMMUNICATION TEST ===");
  
  // Test SPI pins
  Serial.print("ENC_CS pin (");
  Serial.print(ENC_CS);
  Serial.print("): ");
  pinMode(ENC_CS, OUTPUT);
  digitalWrite(ENC_CS, HIGH);
  Serial.println(digitalRead(ENC_CS) ? "HIGH ✓" : "LOW ✗");
  
  // Test basic SPI communication
  Serial.println("\nTesting SPI communication...");
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32); // Slower clock for testing
  
  // Read angle register multiple times
  Serial.println("Reading angle register 10 times:");
  for (int i = 0; i < 10; i++) {
    uint16_t raw = readEncoderRaw(AS5048A_ANGLE_REG);
    float angle = (raw & 0x3FFF) * 2 * PI / 16384.0;
    Serial.print("  ");
    Serial.print(i);
    Serial.print(": Raw=0x");
    Serial.print(raw, HEX);
    Serial.print(" (");
    Serial.print(raw & 0x3FFF);
    Serial.print("), Angle=");
    Serial.println(angle);
    delay(100);
  }
  
  // Test error register
  uint16_t errorReg = readEncoderRaw(AS5048A_ERROR_REG);
  Serial.print("\nError register: 0x");
  Serial.println(errorReg, HEX);
  
  // Test diagnostic register
  uint16_t diagReg = readEncoderRaw(AS5048A_DIAAGC_REG);
  Serial.print("Diagnostic register: 0x");
  Serial.println(diagReg, HEX);
  
  // Test magnitude register
  uint16_t magReg = readEncoderRaw(AS5048A_MAG_REG);
  Serial.print("Magnitude register: 0x");
  Serial.println(magReg, HEX);
  
  Serial.println("\nEncoder test complete.");
}

// Read raw encoder values
void readRawEncoderValues() {
  Serial.println("\n=== RAW ENCODER VALUES ===");
  
  // Get raw angle from SimpleFOC sensor
  float sensorAngle = sensor.getAngle();
  float sensorVelocity = sensor.getVelocity();
  
  // Get raw SPI reading
  uint16_t rawSPI = readEncoderRaw(AS5048A_ANGLE_REG);
  float spiAngle = (rawSPI & 0x3FFF) * 2 * PI / 16384.0;
  
  Serial.print("SimpleFOC sensor angle: ");
  Serial.println(sensorAngle);
  Serial.print("SimpleFOC sensor velocity: ");
  Serial.println(sensorVelocity);
  Serial.print("Raw SPI reading: 0x");
  Serial.print(rawSPI, HEX);
  Serial.print(" -> ");
  Serial.println(spiAngle);
  
  // Check for NaN
  if (isnan(sensorAngle)) {
    Serial.println("WARNING: SimpleFOC sensor returning NaN!");
  }
  if (isnan(sensorVelocity)) {
    Serial.println("WARNING: SimpleFOC velocity returning NaN!");
  }
}

// Read all encoder registers
void readAllEncoderRegisters() {
  Serial.println("\n=== ALL ENCODER REGISTERS ===");
  
  struct {
    uint16_t address;
    const char* name;
  } registers[] = {
    {AS5048A_ERROR_REG, "Error"},
    {AS5048A_PROG_REG, "Programming"},
    {AS5048A_DIAAGC_REG, "Diagnostic/AGC"},
    {AS5048A_MAG_REG, "Magnitude"},
    {AS5048A_ANGLE_REG, "Angle"}
  };
  
  for (int i = 0; i < 5; i++) {
    uint16_t value = readEncoderRaw(registers[i].address);
    Serial.print(registers[i].name);
    Serial.print(" (0x");
    Serial.print(registers[i].address, HEX);
    Serial.print("): 0x");
    Serial.print(value, HEX);
    Serial.print(" (");
    Serial.print(value);
    Serial.println(")");
    
    // Decode specific registers
    if (registers[i].address == AS5048A_ANGLE_REG) {
      float angle = (value & 0x3FFF) * 360.0 / 16384.0;
      Serial.print("  -> Angle: ");
      Serial.print(angle);
      Serial.println(" degrees");
    }
    else if (registers[i].address == AS5048A_DIAAGC_REG) {
      uint8_t agc = (value >> 8) & 0xFF;
      bool magHigh = (value >> 11) & 1;
      bool magLow = (value >> 10) & 1;
      Serial.print("  -> AGC: ");
      Serial.print(agc);
      Serial.print(", MagHigh: ");
      Serial.print(magHigh);
      Serial.print(", MagLow: ");
      Serial.println(magLow);
    }
    delay(10);
  }
}

// Show position and velocity limits
void showLimits() {
  Serial.println("\n=== SYSTEM LIMITS ===");
  Serial.print("Position limits: ");
  Serial.print(MIN_POSITION);
  Serial.print(" to ");
  Serial.print(MAX_POSITION);
  Serial.println(" rad");
  Serial.print("Velocity limit: ±");
  Serial.print(MAX_VELOCITY);
  Serial.println(" rad/s");
  Serial.print("Current position: ");
  Serial.println(motor.shaft_angle);
  Serial.print("Current target: ");
  Serial.println(motor.target);
  Serial.print("Motor enabled: ");
  Serial.println(motorEnabled ? "YES" : "NO");
}

// Enhanced status function
void printStatus() {
  Serial.println("=== Gimbal Motor Status ===");
  Serial.printf("Current Position: %.3f rad (%.1f deg)\n", motor.shaftAngle(), motor.shaftAngle() * 180.0 / PI);
  Serial.printf("Target Position: %.3f rad (%.1f deg)\n", motor.target, motor.target * 180.0 / PI);
  Serial.printf("Current Velocity: %.3f rad/s\n", motor.shaftVelocity());
  Serial.printf("Voltage Limit: %.2f V\n", motor.voltage_limit);
  Serial.printf("Velocity Limit: %.2f rad/s\n", motor.velocity_limit);
  Serial.printf("Motor Enabled: %s\n", motorEnabled ? "YES" : "NO");
  Serial.printf("Emergency Stop: %s\n", emergencyStopActive ? "ACTIVE" : "CLEAR");
  Serial.printf("In Timed Move: %s\n", inTimedMove ? "YES" : "NO");
}

// Print encoder debug information
void printEncoderDebugInfo() {
  Serial.println("\n=== DEBUG INFO ===");
  
  // SimpleFOC sensor readings
  float sensorAngle = sensor.getAngle();
  float sensorVelocity = sensor.getVelocity();
  
  // Raw SPI reading
  uint16_t rawSPI = readEncoderRaw(AS5048A_ANGLE_REG);
  float spiAngle = (rawSPI & 0x3FFF) * 2 * PI / 16384.0;
  
  // Motor state
  Serial.printf("Motor Target: %.3f rad\n", motor.target);
  Serial.printf("Motor Position: %.3f rad\n", motor.shaft_angle);
  Serial.printf("Motor Velocity: %.3f rad/s\n", motor.shaft_velocity);
  Serial.printf("SimpleFOC Sensor: %.3f rad (NaN: %s)\n", sensorAngle, isnan(sensorAngle) ? "YES" : "NO");
  Serial.printf("SimpleFOC Velocity: %.3f rad/s (NaN: %s)\n", sensorVelocity, isnan(sensorVelocity) ? "YES" : "NO");
  Serial.printf("Raw SPI: 0x%04X -> %.3f rad\n", rawSPI, spiAngle);
  Serial.printf("Motor Enabled: %s\n", motorEnabled ? "YES" : "NO");
  Serial.printf("Emergency Stop: %s\n", emergencyStopActive ? "ACTIVE" : "CLEAR");
  
  // Check for error conditions
  if (isnan(sensorAngle) || isnan(sensorVelocity)) {
    Serial.println("⚠️  ENCODER ERROR: NaN values detected!");
  }
  
  if (rawSPI == 0 || rawSPI == 0xFFFF) {
    Serial.println("⚠️  SPI ERROR: Invalid raw reading!");
  }
}

// Reset encoder position and motor state
void resetMotorPosition() {
  Serial.println("\n=== RESETTING MOTOR POSITION ===");
  
  // Stop motor movement immediately
  motor.target = motor.shaftAngle();
  motor.velocity_limit = 0.1;  // Very slow for safety
  
  Serial.printf("Current position: %.3f rad (%.1f rotations)\n", 
                motor.shaftAngle(), motor.shaftAngle() / (2 * PI));
  
  // Reset the sensor zero point
  sensor.update();
  float currentSensorAngle = sensor.getAngle();
  Serial.printf("Sensor angle before reset: %.3f rad\n", currentSensorAngle);
  
  // Reset SimpleFOC sensor offset - sets current position as new zero
  motor.sensor_offset = currentSensorAngle;
  motor.zero_electric_angle = motor.electricalAngle();
  
  // Force immediate position reset
  motor.shaft_angle = 0.0;
  motor.target = 0.0;
  
  Serial.println("Position reset complete. Current position is now 0.000 rad");
  Serial.println("Motor target set to 0.000 rad");
  
  // Restore normal velocity limit
  motor.velocity_limit = maxVelocity;
  
  Serial.println("Use 'status' to verify the reset");
}

// Check if motor is spinning freely (indicates control issues)
void checkMotorControl() {
  Serial.println("\n=== MOTOR CONTROL DIAGNOSTICS ===");
  
  float position = motor.shaftAngle();
  float velocity = motor.shaftVelocity();
  float target = motor.target;
  
  Serial.printf("Position: %.3f rad (%.1f rotations)\n", position, position / (2 * PI));
  Serial.printf("Target: %.3f rad\n", target);
  Serial.printf("Velocity: %.3f rad/s\n", velocity);
  Serial.printf("Position error: %.3f rad\n", target - position);
  
  // Check for excessive rotation (indicates free spinning)
  float rotations = abs(position) / (2 * PI);
  if (rotations > 10) {
    Serial.println("⚠️  WARNING: Excessive rotation detected!");
    Serial.println("   Motor appears to be spinning freely (no control)");
    Serial.println("   Possible causes:");
    Serial.println("   - Magnet not attached to motor shaft");
    Serial.println("   - Magnet too far from encoder (should be 0.5-1.5mm)");
    Serial.println("   - Motor driver not energized properly");
    Serial.println("   - Electrical angle calibration failed");
    Serial.println("   *** Use 'reset' command to stop accumulation ***");
  }
  
  // Check velocity
  if (abs(velocity) > 50) {
    Serial.println("⚠️  WARNING: High velocity detected!");
    Serial.println("   Motor may be out of control");
  }
  
  // Check control parameters
  Serial.printf("\nControl Parameters:\n");
  Serial.printf("  Voltage limit: %.2f V\n", motor.voltage_limit);
  Serial.printf("  Velocity limit: %.2f rad/s\n", motor.velocity_limit);
  Serial.printf("  PID_velocity.P: %.3f\n", motor.PID_velocity.P);
  Serial.printf("  PID_velocity.I: %.3f\n", motor.PID_velocity.I);
  Serial.printf("  PID_velocity.D: %.3f\n", motor.PID_velocity.D);
    // Check if motor is actually energized
  Serial.printf("\nMotor Status:\n");
  Serial.printf("  Enabled: %s\n", motorEnabled ? "YES" : "NO");
  Serial.printf("  Driver enabled: %s\n", motor.enabled ? "YES" : "NO");
  Serial.printf("  EN_GATE pin state: %s\n", digitalRead(EN_GATE) ? "HIGH (enabled)" : "LOW (disabled)");
  
  // Check motor voltages (indicates if FOC is working)
  Serial.printf("\nMotor Drive Voltages:\n");
  Serial.printf("  Voltage q-axis: %.3f V\n", motor.voltage.q);
  Serial.printf("  Voltage d-axis: %.3f V\n", motor.voltage.d);
  Serial.printf("  Total voltage magnitude: %.3f V\n", sqrt(motor.voltage.q*motor.voltage.q + motor.voltage.d*motor.voltage.d));
  
  // Check electrical angle
  Serial.printf("  Electrical angle: %.3f rad\n", motor.electrical_angle);
  
  if (abs(motor.voltage.q) < 0.01 && abs(motor.voltage.d) < 0.01) {
    Serial.println("⚠️  WARNING: No motor drive voltages detected!");
    Serial.println("   FOC may not be functioning properly");
    Serial.println("   Possible causes:");
    Serial.println("   - Position error too small (motor thinks it's at target)");
    Serial.println("   - PID gains too low");
    Serial.println("   - Motor disabled or FOC not initialized");
  }
  
  // Check encoder readings
  float sensorAngle = sensor.getAngle();
  Serial.printf("\nEncoder Status:\n");
  Serial.printf("  SimpleFOC sensor: %.3f rad\n", sensorAngle);
  Serial.printf("  Sensor valid: %s\n", !isnan(sensorAngle) ? "YES" : "NO");
  
  // Recommendations
  Serial.println("\nRecommendations:");
  if (rotations > 10) {
    Serial.println("1. URGENT: Use 'reset' command to stop position accumulation");
    Serial.println("2. Check magnet attachment to motor shaft");
    Serial.println("3. Verify magnet distance from encoder (0.5-1.5mm optimal)");
    Serial.println("4. Check if motor shaft rotates freely by hand");
  }
  if (digitalRead(EN_GATE) == LOW) {
    Serial.println("5. Motor driver appears disabled - check EN_GATE connection");
  }
}