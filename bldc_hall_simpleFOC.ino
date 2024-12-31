#include <SimpleFOC.h>

// Define Hall sensor pins
#define HALL_A PB6
#define HALL_B PB7
#define HALL_C PB8

// Define PWM pins for 6-PWM control
#define INHA PA8  // High-side phase A
#define INHB PA9  // High-side phase B
#define INHC PA10 // High-side phase C
#define INLA PA7  // Low-side phase A
#define INLB PB0  // Low-side phase B
#define INLC PB1  // Low-side phase C

// Define Enable pin for the driver
#define ENABLE_PIN PA15

// Define current sensing pins
#define SENSE_IA PA0
#define SENSE_IB PA1
#define SENSE_IC PA2

// Define throttle pin
#define THROTTLE_PIN PA3

// Motor and driver configuration
BLDCMotor motor(12); // 12 poles
BLDCDriver6PWM driver(INHA, INLA, INHB, INLB, INHC, INLC);

// Hall sensor instance
HallSensor hallSensor(HALL_A, HALL_B, HALL_C, 12); // 12 poles

// Current sensing instance
InlineCurrentSense currentSense(0.01, 40, SENSE_IA, SENSE_IB, SENSE_IC); // Shunt resistor = 0.01Ω, gain = 50

// Interrupt service routines for Hall sensors
void doA() { hallSensor.handleA(); }
void doB() { hallSensor.handleB(); }
void doC() { hallSensor.handleC(); }

void setup() {
  // Initialize the driver
  driver.voltage_power_supply = 48;  // Motor supply voltage
  driver.init();
  driver.enable_pin = ENABLE_PIN;   // Set enable pin
  driver.enable_active_high = true; // Driver is enabled with a HIGH signal

  // Link the driver to the motor
  motor.linkDriver(&driver);

  // Initialize the Hall sensor
  hallSensor.init();
  hallSensor.enableInterrupts(doA, doB, doC);
  
  // Link the Hall sensor to the motor
  motor.linkSensor(&hallSensor);

  // Initialize current sensing
  currentSense.init();
  motor.linkCurrentSense(&currentSense);

  // Motor settings
  motor.foc_modulation = FOCModulationType::SinePWM; // Sine wave PWM
  motor.torque_controller = TorqueControlType::foc_current; // Current-based torque control
  motor.controller = MotionControlType::velocity;    // Control the velocity

  motor.voltage_limit = 12;      // Max voltage applied to the motor
  motor.velocity_limit = 50;     // Max velocity in rad/s (adjust as needed)

  // Initialize the motor
  motor.init();
  
  // Initialize the driver enable pin
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); // Enable the driver

  // Initialize the throttle pin
  pinMode(THROTTLE_PIN, INPUT_ANALOG);

  Serial.begin(115200);
  Serial.println("SimpleFOC running");
}

void loop() {
  // Read throttle value (0 to 1023) and map to velocity range
  int throttleValue = analogRead(THROTTLE_PIN); // Read potentiometer value
  float targetVelocity = map(throttleValue, 0, 1023, 0, motor.velocity_limit);

  // Set the target velocity for the motor
  motor.move(targetVelocity);

  // Monitor motor state (for debugging)
  motor.monitor();
}
