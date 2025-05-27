#include <TMC2209Stepper.h>

//——— Configuration ———//
// Sense resistor value on your TMC2209 board (Ω)
#define R_SENSE 0.11f

// TMC2209 driver address (set via MS1/MS2 on your board; here 0b00)
#define DRIVER_ADDR 0b00

// UART: connect TMC2209 PDN_UART pin → Teensy Serial1 TX (pin 1)
// If you have separate RX pad wired, hook it to Serial1 RX (pin 0)
#define ENABLE_SERIAL1_RX false  // set true if you’ve wired RX1←TX
#define DRIVER_SERIAL       Serial1
#define DRIVER_BAUD         115200

// STEP / DIR / EN pins
const int STEP_PIN = 2;
const int DIR_PIN  = 3;
const int EN_PIN   = 4;

TMC2209Stepper driver(&DRIVER_SERIAL, R_SENSE, DRIVER_ADDR);

void setup() {
  // Terminal
  Serial.begin(115200);
  while(!Serial) ;
  Serial.println("\nTMC2209 terminal control");
  Serial.println("Commands: enable, disable, dir [0|1], step N, move N, status");

  // UART to driver
  if(ENABLE_SERIAL1_RX) Serial1.begin(DRIVER_BAUD);
  else                  DRIVER_SERIAL.begin(DRIVER_BAUD, SERIAL_8N1, -1, 1);

  // Pins
  pinMode(EN_PIN,   OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  // Driver init
  digitalWrite(EN_PIN, HIGH);  // disable motor
  driver.begin();
  driver.rms_current(800);     // 800 mA RMS (adjust to your motor)
  driver.microsteps(16);
  driver.en_spreadCycle(true); // SpreadCycle for torque
  driver.coolstep(0);          // CoolStep off
  driver.TCOOLTHRS(0xFFFFF);   // StallGuard always on (if you use it)
  driver.diag1_error(true);    // DIAG1 = error flag

  Serial.println("Driver initialized.");
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "enable") {
    digitalWrite(EN_PIN, LOW);
    Serial.println("Motor ENABLED");
  }
  else if (cmd == "disable") {
    digitalWrite(EN_PIN, HIGH);
    Serial.println("Motor DISABLED");
  }
  else if (cmd.startsWith("dir ")) {
    int d = cmd.substring(4).toInt();
    digitalWrite(DIR_PIN, d ? HIGH : LOW);
    Serial.printf("DIR set to %d\n", d);
  }
  else if (cmd.startsWith("step ")) {
    long steps = cmd.substring(5).toInt();
    Serial.printf("Stepping %ld pulses…\n", steps);
    for (long i = 0; i < steps; i++) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(2);
    }
    Serial.println("Done.");
  }
  else if (cmd.startsWith("move ")) {
    long steps = cmd.substring(5).toInt();
    Serial.printf("Moving %ld microsteps…\n", steps);
    digitalWrite(EN_PIN, LOW);
    delay(10);
    driver.push(); // ensure any pending UART writes done
    for (long i = 0; i < steps; i++) {
      driver.step(); // hardware toggles STEP pin for you
      delayMicroseconds( driver.microstepInterval() );
    }
    Serial.println("Move complete.");
  }
  else if (cmd == "status") {
    uint32_t drvStatus = driver.GSTAT();
    Serial.printf("GSTAT = 0x%04X\n", drvStatus);
  }
  else {
    Serial.println("Unknown command");
  }
}
