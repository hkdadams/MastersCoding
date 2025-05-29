#include <SimpleFOC.h>

// MagneticSensorSPI(int cs, float bit_resolution, int angle_register)
MagneticSensorSPI sensor = MagneticSensorSPI(10, 14, 0x3FFF);

//  BLDCMotor( pole_pairs , ( phase_resistance, KV_rating  optional) )
BLDCMotor motor = BLDCMotor(11);
//  BLDCDriver3PWM( pin_pwmA, pin_pwmB, pin_pwmC, enable (optional))
BLDCDriver3PWM driver = BLDCDriver3PWM(4, 5, 6);

//  InlineCurrentSense(shunt_resistance, gain, adc_a, adc_b)
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50, A0, A2);

//instantiate commander
Commander commander = Commander(Serial);
void doMotor(char* cmd){commander.motor(&motor, cmd);}

void setup() {
  // init the serial port
  Serial.begin(115200);

  // enable the debugging output
  SimpleFOCDebug::enable(&Serial);

  // init sensor
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 20000;
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;
  // driver init
  driver.init();

  // link the motor to the driver
  motor.linkDriver(&driver);
  // link the driver with the current sense
  current_sense.linkDriver(&driver);
  // link the motor to current sense
  motor.linkCurrentSense(&current_sese);

  // use monitoring with the BLDCMotor
  Serial.begin(115200);
  // monitoring port
  motor.useMonitoring(Serial);

  // subscribe motor to the commands
  commander.add('M',doMotor,"motor");

  // set control loop type to be used
  motor.controller = MotionControlType::velocity;
  // initialize motor
  motor.init();
  
  
  // init current sense
  current_sense.init();

  // align encoder and start FOC
  motor.initFOC();
}

void loop() {
  // FOC algorithm function
  motor.loopFOC();

  // velocity control loop function
  // setting the target velocity to 2rad/s
  motor.move(2);

  // monitoring function outputting motor variables to the serial terminal 
  motor.monitor();

  // read user commands
  commander.run();
}