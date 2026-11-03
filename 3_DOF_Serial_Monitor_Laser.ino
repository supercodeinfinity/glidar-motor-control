/*
GLiDAR Serial Monitor Control
Description: Type commands in Serial Monitor to move, turn, or bend the scope
             Give a distance to move, an angle to turn, and a tip angle to aim for
             Check motor readings, return to the starting point, or send laser commands
Components: Teensy 4.1, NEMA 14/17 steppers, TB6600 drivers, encoders,
            ST3020 servo, bus adapter, and Dornier laser interface
*/

// Use SCServo to talk to the servo that bends the tip
#include <SCServo.h>

#include <math.h>

SMS_STS servo;

// Linear motor: NEMA 14 with a TB6600 stepper driver
const int linear_enaPin = 2;  // Driver enable
const int linear_dirPin = 3;  // Motor direction
const int linear_stepPin = 4;  // Step pulses

// The encoder counts turns of the motor that moves the scope in and out
const int linear_encA = 17;  // Encoder A
const int linear_encB = 16;  // Encoder B

// Rotation motor: NEMA 17 with a second TB6600 driver
const int rotation_enaPin = 5;  // Driver enable
const int rotation_dirPin = 6;  // Motor direction
const int rotation_stepPin = 7;  // Step pulses

// Read both encoder channels to track how far the motor turns and which way
const int rotation_encA = 20;  // Encoder A
const int rotation_encB = 21;  // Encoder B

// Pins for the laser: one Ready/Standby contact and two Fire outputs
const int relay_laser = 8;  // Ready/Standby
const int fire1_laser = 9;  // Fire channel 1
const int fire2_laser = 10;  // Fire channel 2

// With this wiring, LOW closes the Ready/Standby contact
const int LASER_RELAY_ACTIVE = LOW;
const int LASER_RELAY_RELEASED = HIGH;

// HIGH turns the Fire outputs on
const int LASER_FIRE_ACTIVE = HIGH;
const int LASER_FIRE_RELEASED = LOW;

// Hold the contact closed for three seconds for Ready or Standby
const unsigned long laserControlPulse_ms = 3000;

// Remember the laser commands sent; these flags do not read the actual laser
bool laserEnabled = false;
bool laserReady = false;
bool laserFiring = false;

// Set these pins LOW to turn on the stepper drivers
const int enaPin_Enable = LOW;
const int enaPin_Disable = HIGH;

// Change these HIGH/LOW values if the motors turn the wrong way
const int linear_dirForward = HIGH;
const int linear_dirBackward = LOW;

const int rotation_dirClockwise = LOW;
const int rotation_dirCounter = HIGH;

const int degreesPerRevolution = 360;
const int halfRevolutionDegrees = 180;
const int servoMaximumStep = 4095;

const unsigned long printInterval_ms = 40;

// One full motor turn moves the TR8x4 lead screw by 4 mm
const float linearMMPerRev = 4.0;

// This needs to match the microstep setting on the TB6600
const float linearPulsesPerRev = 1600.0;

// Count when encoder channel A goes HIGH, once per cycle (1x decoding)
const float linearEncoderCPR = 1000.0;

// Stop when the move is within 0.005 mm of the target, about one encoder count
const float linearStopMarginMM = 0.005;

// The allowed in/out range in mm, starting from where the scope was at startup
const float linearMinPositionMM = 0.0;
const float linearMaxPositionMM = 177.0;

// Leave a little room for rounding at the ends of the travel range
const float linearRangeToleranceMM = 0.01;

// Flip this sign if moving forward makes the encoder count go down
const int linearEncoderSign = +1;

// The encoder interrupt keeps this count updated while the rest of the code runs
volatile long linearEncoderCount = 0;

// Start the hardware at 0 mm; this code does not search for a home switch
float currentLinearPositionMM = 0.0;

// Encoder counts for one full rotation motor turn, using 1x decoding
const float rotationEncoderCPR = 1000.0;

// Use this to estimate how many step pulses to send before giving up on a move
const float rotationPulsesPerRev = 1600.0;

// The motor has to turn 2.5 degrees to turn the scope by 1 degree
const float rotationGearRatio = 2.5;

// How close the motor angle should get before stopping, about half an encoder count
const float rotationStopMarginMotorDeg = 0.18;

// Flip this sign if the rotation reading goes the wrong way
const int rotationEncoderSign = -1;

volatile long rotationEncoderCount = 0;

// Start with the scope flat; this angle goes back to 0 after a full turn
float currentRotationMapDeg = 0.0;

const float rotationMapZeroToleranceDeg = 0.5;

// Connect the ST3020 through Serial1: pin 0 is RX1 and pin 1 is TX1
const long servo_baud = 1000000;
const int servo_ID = 1;
const int servo_speed = 500;
const int servo_acceleration = 100;

// Slow the servo down when the requested tip bend is over 100 degrees
const int flexionSlowdownStartTipDeg = 100;

// Take off 2.5 speed units for each extra degree, down to the minimum speed
const float flexionSpeedReductionPerTipDeg = 2.5;

const int flexionMinimumSpeed = 50;

// Show stronger warnings as the requested bend gets bigger
const int flexionSmallWarningTipDeg = 150;
const int flexionModerateWarningTipDeg = 200;
const int flexionMajorWarningTipDeg = 250;

// Position 2048 on the servo means the tip is straight
const int flexionNeutralStep = 2048;

// Each servo encoder step is about 0.088 degrees of motor rotation
const int flexionStopMarginSteps = 1;

// These numbers relate servo rotation to how much the tip bends
// Use the same ranges when bending in either direction
const float flexionTwistFactor0To90 = 6.0;
const float flexionTwistFactor90To180 = 6.5;
const float flexionTwistFactor180To270 = 7.2;

// Gear ratio from the servo to the parts that bend the tip
const float flexionGearRatio = 3.5;

// Measure positive and negative servo angles from the straight-tip position
const float flexionMaxMotorDeg = 180.0;
const float flexionMinMotorDeg = -180.0;

// How far the tip is allowed to bend, in degrees
const float flexionMaxTipDeg = 270.0;
const float flexionMinTipDeg = -270.0;

// Use one servo encoder count to work out how close the angles need to be
const float flexionRangeToleranceMotorDeg = degreesPerRevolution / (float)servoMaximumStep;
const float flexionRangeToleranceTipDeg =
  flexionRangeToleranceMotorDeg * (flexionTwistFactor180To270 / flexionGearRatio);

// Stop waiting if the servo reading does not change for half a second
const unsigned long flexionNoChangeTimeout_ms = 500;

// Keep each step pulse HIGH for 800 microseconds, then LOW for 800
const unsigned int stepDelay_us = 800;

// Give the driver time to read the direction before sending a step pulse
const unsigned int directionSetup_us = 10;

// Remember if the servo answered during startup
bool servoConnected = false;

// Save the three numbers entered for the move
float requestedLinearMM = 0.0;
float requestedRotationDeg = 0.0;
float requestedFlexionDeg = 0.0;

// Home moves back to the saved zero positions; it does not look for a switch
bool homeCommandRequested = false;

// Skip Home if the position readings are already close enough to zero
bool homeCommandAlreadyAtHome = false;

// Remember if x was entered while a move was running
bool movementEmergencyStop = false;

// Save the requested distance and where that move should end up
float targetLinearRelativeMM = 0.0;

float targetLinearAbsoluteMM = 0.0;

float targetLinearMagnitudeMM = 0.0;

float targetLinearRevolutions = 0.0;
long targetLinearPulses = 0;

int linearMovementSign = +1;

// Keep track of which moves are done so all three can run together
bool linearDone = true;
long linearPulsesSent = 0;
long signedLinearCount = 0;
float measuredLinearMotorDeg = 0.0;
float measuredLinearUreteroMM = 0.0;

float targetUreteroscopeRotationDeg = 0.0;

float targetRotationMotorDeg = 0.0;
float targetRotationMotorMagnitudeDeg = 0.0;

long targetRotationPulses = 0;

int rotationMovementSign = +1;

bool rotationDone = true;
long rotationPulsesSent = 0;
long signedRotationCount = 0;
float measuredRotationMotorDeg = 0.0;
float measuredUreteroRotationDeg = 0.0;

float targetTipDeg = 0.0;

// Pick the calibration values for the requested bend
float selectedFlexionTwistFactor = flexionTwistFactor0To90;
float selectedFlexionTipGain = flexionTwistFactor0To90 / flexionGearRatio;

float targetFlexionMotorDeg = 0.0;

// Shift the angle so it fits the servo's 0-to-360-degree scale
float targetFlexionPhysicalDeg = 0.0;

int targetFlexionSteps = flexionNeutralStep;

int selectedFlexionSpeed = servo_speed;

bool flexionDone = true;
int currentFlexionStep = -1;
int lastMovingFlexionStep = -1;
float measuredFlexionMotorDeg = 0.0;
float measuredTipDeg = 0.0;

// Space out prints and servo readings so they do not slow down the move too much
unsigned long lastPrintTime = 0;
unsigned long lastFlexionReadTime = 0;
unsigned long lastFlexionChangeTime = 0;

void linearSetup();
void rotationSetup();
void flexionSetup();

void diagnostics();
void linearDiagnostics();
void rotationDiagnostics();
void flexionDiagnostics();
void printFlexionTuningSummary();
void printLinearZeroReminder();
void printRotationNeutralReminder();
void printCombinedCommandPrompt();
void printCommandPrompt();
void printLaserCommandHelp();
void printReceivedInput(const char inputLine[]);

bool readSerialLine(char inputLine[], size_t inputLineSize);
bool readMovementCommandFromLine(const char inputLine[]);
char getSingleLetterCommand(const char inputLine[]);
bool handleLaserCommand(char command);
void clearSerialInput();

bool calculateAllTargets();
bool calculateLinearTarget();
bool calculateRotationTarget();
float selectFlexionTwistFactor(float tipDeg);
bool calculateFlexionTarget();
bool linearTargetIsInsideRange();
bool flexionTargetIsInsideRange();
int calculateFlexionSpeed();
void printFlexionOverstretchWarning();

void moveAllAxesTogether();
void resetMovementMeasurements();
void printRequestedTargets();
void prepareMotorsForMovement();
void startFlexionMovement();
void runSimultaneousMovementLoop();
void pulseActiveSteppers();
void readLinearFeedback();
void readRotationFeedback();
void readFlexionFeedbackIfReady();
void printMovementStatusIfReady();
void updateAbsoluteLinearPosition();
void updateAbsoluteRotationPosition();
void printFinalSummary();

void linearEncoderISR();
void rotationEncoderISR();
long getLinearEncoderCount();
long getRotationEncoderCount();
void resetLinearEncoder();
void resetRotationEncoder();

void disableAllMotors();
void driveLaserOutputsToReleasedState();
void lockLaserSoftwareState();
void sendLaserControlPulse(unsigned long durationMs);
void pauseLaserFiring();
void stopLaserAndRelock(bool sendReadyClearPulse);
void setupLaserOutputs();
void checkForEmergencyStopDuringMovement();
bool inputLineEqualsWord(const char inputLine[], const char word[]);
bool softwareIsAtHome();
void setHomeCommandTargets();
float normalizeRotationMapDeg(float angleDeg);

void driveLaserOutputsToReleasedState() {
  digitalWrite(fire1_laser, LASER_FIRE_RELEASED);
  digitalWrite(fire2_laser, LASER_FIRE_RELEASED);
  digitalWrite(relay_laser, LASER_RELAY_RELEASED);
  laserFiring = false;
}

void lockLaserSoftwareState() {
  laserEnabled = false;
  laserReady = false;
  laserFiring = false;
}

// Ready and Standby both use this contact pulse
void sendLaserControlPulse(unsigned long durationMs) {
  if (!laserEnabled) {
    Serial.println("Laser pulse blocked: type Enable [e] first.");
    return;
  }

  Serial.print("Laser relay/contact pulse: ");
  Serial.print(durationMs);
  Serial.println(" ms");

  digitalWrite(relay_laser, LASER_RELAY_ACTIVE);
  delay(durationMs);
  digitalWrite(relay_laser, LASER_RELAY_RELEASED);

  Serial.println("Laser relay/contact pulse finished.");
}

void pauseLaserFiring() {
  digitalWrite(fire1_laser, LASER_FIRE_RELEASED);
  digitalWrite(fire2_laser, LASER_FIRE_RELEASED);
  laserFiring = false;
}

// Turn off the Fire outputs and lock the commands again; reset Ready if asked
void stopLaserAndRelock(bool sendReadyClearPulse) {
  pauseLaserFiring();

  if (sendReadyClearPulse && laserEnabled && laserReady) {
    sendLaserControlPulse(laserControlPulse_ms);
  }

  driveLaserOutputsToReleasedState();
  lockLaserSoftwareState();
}

void setupLaserOutputs() {

  // Set the laser pins to their off levels before turning them into outputs
  driveLaserOutputsToReleasedState();

  pinMode(relay_laser, OUTPUT);
  pinMode(fire1_laser, OUTPUT);
  pinMode(fire2_laser, OUTPUT);

  driveLaserOutputsToReleasedState();
  lockLaserSoftwareState();
}

void printLaserCommandHelp() {
  Serial.println("LASER: [e] Enable | [r] Ready | [f] Fire | [p] Pause | [s] Standby | [x] Stop");
}

// Read the letter commands and make sure Enable and Ready happen before Fire
bool handleLaserCommand(char command) {
  if (command == '\0') {
    return false;
  }

  Serial.print("Laser/motion command input: ");
  Serial.println(command);

  if (command == 'e') {
    if (laserFiring) {
      Serial.println("Enable [e] is not allowed while firing. Emergency stop triggered.");
      stopLaserAndRelock(true);
      Serial.println("All laser outputs are OFF. Laser commands are locked.");
      return true;
    }

    if (laserEnabled) {
      Serial.println("Enable [e] ignored: laser commands are already unlocked.");
      return true;
    }

    driveLaserOutputsToReleasedState();
    laserEnabled = true;
    laserReady = false;

    Serial.println("Laser command: Enable [e]");
    Serial.println("Laser commands are unlocked. Next type Ready [r] before Fire [f].");
    return true;
  }

  if (command == 'r') {
    if (laserFiring) {
      Serial.println("Ready [r] is not allowed while firing. Emergency stop triggered.");
      stopLaserAndRelock(true);
      Serial.println("All laser outputs are OFF. Laser commands are locked.");
      return true;
    }

    if (!laserEnabled) {
      Serial.println("Ready blocked: type Enable [e] first.");
      printLaserCommandHelp();
      return true;
    }

    if (laserReady) {
      Serial.println("Ready [r] ignored: laser is already ready. No pulse was sent.");
      return true;
    }

    driveLaserOutputsToReleasedState();
    Serial.println("Laser command: Ready [r]");
    sendLaserControlPulse(laserControlPulse_ms);
    laserReady = true;
    Serial.println("Laser ready sequence finished. Fire [f] is now allowed.");
    return true;
  }

  if (command == 'f') {
    if (!laserEnabled || !laserReady) {
      Serial.println("Fire blocked: type Enable [e], then Ready [r], then Fire [f].");
      printLaserCommandHelp();
      return true;
    }

    if (laserFiring) {
      Serial.println("Fire [f] ignored: laser is already firing.");
      return true;
    }

    Serial.println("Laser command: Fire [f]");
    Serial.println("Turning ON fire output 1 and fire output 2.");
    digitalWrite(fire1_laser, LASER_FIRE_ACTIVE);
    digitalWrite(fire2_laser, LASER_FIRE_ACTIVE);
    laserFiring = true;
    Serial.println("Fire outputs are ON. Type Pause [p], Standby [s], or Stop [x] to turn them OFF.");
    return true;
  }

  if (command == 'p') {
    Serial.println("Laser command: Pause [p]");
    pauseLaserFiring();
    Serial.println("Fire outputs are OFF. Laser remains enabled and ready.");
    return true;
  }

  if (command == 's') {
    if (!laserEnabled) {
      Serial.println("Standby blocked: type Enable [e] first.");
      printLaserCommandHelp();
      return true;
    }

    Serial.println("Laser command: Standby [s]");
    pauseLaserFiring();

    if (laserReady) {
      sendLaserControlPulse(laserControlPulse_ms);
    } else {
      Serial.println("Standby pulse skipped: laser is not marked ready.");
    }

    laserReady = false;
    Serial.println("Standby finished. Ready has been cleared; type Ready [r] before Fire [f].");
    return true;
  }

  if (command == 'x') {
    Serial.println("Laser command: Stop [x]");
    stopLaserAndRelock(true);
    Serial.println("All laser outputs are OFF. Laser commands are locked.");
    return true;
  }

  return false;
}

// Keep looking for x so a move can be stopped while it is running
void checkForEmergencyStopDuringMovement() {
  if (Serial.available() <= 0) {
    return;
  }

  int incoming = Serial.peek();

  if (incoming == 'x' || incoming == 'X') {
    Serial.println("\nEmergency Stop [x] received during movement.");
    stopLaserAndRelock(true);
    disableAllMotors();
    linearDone = true;
    rotationDone = true;
    flexionDone = true;
    movementEmergencyStop = true;
    clearSerialInput();
  }
}

void linearSetup() {

  pinMode(linear_stepPin, OUTPUT);
  pinMode(linear_dirPin, OUTPUT);
  pinMode(linear_enaPin, OUTPUT);

  digitalWrite(linear_stepPin, LOW);
  digitalWrite(linear_dirPin, linear_dirForward);
  digitalWrite(linear_enaPin, enaPin_Enable);

  pinMode(linear_encA, INPUT_PULLUP);
  pinMode(linear_encB, INPUT_PULLUP);

  // Add a count each time encoder channel A goes HIGH
  attachInterrupt(digitalPinToInterrupt(linear_encA), linearEncoderISR, RISING);

  resetLinearEncoder();
}

void rotationSetup() {

  pinMode(rotation_stepPin, OUTPUT);
  pinMode(rotation_dirPin, OUTPUT);
  pinMode(rotation_enaPin, OUTPUT);

  digitalWrite(rotation_stepPin, LOW);
  digitalWrite(rotation_dirPin, rotation_dirClockwise);
  digitalWrite(rotation_enaPin, enaPin_Enable);

  pinMode(rotation_encA, INPUT_PULLUP);
  pinMode(rotation_encB, INPUT_PULLUP);

  // Count pulses on channel A and read channel B to work out the direction
  attachInterrupt(digitalPinToInterrupt(rotation_encA), rotationEncoderISR, RISING);

  resetRotationEncoder();
}

void flexionSetup() {

  Serial1.begin(servo_baud);

  servo.pSerial = &Serial1;

  // Move the tip servo to its straight position at startup
  servo.WritePosEx(servo_ID, flexionNeutralStep, servo_speed, servo_acceleration);
}

void diagnostics() {
  Serial.println("\n-------------------- DIAGNOSTICS --------------------");
  linearDiagnostics();
  rotationDiagnostics();
  flexionDiagnostics();
  Serial.println("-----------------------------------------------------");
}

// Print the pin settings and encoder readings to help check the wiring
void linearDiagnostics() {
  Serial.println("\nLinear Axis Diagnostics");

  Serial.print("ENA Pin (expected 0): ");
  Serial.println(digitalRead(linear_enaPin));
  Serial.print("DIR Pin: ");
  Serial.println(digitalRead(linear_dirPin));
  Serial.print("STEP Pin: ");
  Serial.println(digitalRead(linear_stepPin));
  Serial.print("Pulses per Motor Revolution: ");
  Serial.println(linearPulsesPerRev, 0);

  resetLinearEncoder();

  Serial.print("Encoder Count: ");
  Serial.println(getLinearEncoderCount());

  Serial.print("Tracked Absolute Linear Position: ");
  Serial.print(currentLinearPositionMM, 3);
  Serial.println(" mm");
}

void rotationDiagnostics() {
  Serial.println("\nRotation Axis Diagnostics");

  Serial.print("ENA Pin (expected 0): ");
  Serial.println(digitalRead(rotation_enaPin));
  Serial.print("DIR Pin: ");
  Serial.println(digitalRead(rotation_dirPin));
  Serial.print("STEP Pin: ");
  Serial.println(digitalRead(rotation_stepPin));
  Serial.print("Pulses per Motor Revolution: ");
  Serial.println(rotationPulsesPerRev, 0);

  resetRotationEncoder();

  Serial.print("Encoder Count: ");
  Serial.println(getRotationEncoderCount());
}

void flexionDiagnostics() {
  Serial.println("\nFlexion Axis Diagnostics");

  // Check that the servo answers; -1 means it did not reply
  int detectedID = servo.Ping(servo_ID);

  if (detectedID == -1) {
    servoConnected = false;
    Serial.println("Servo Communication Failed");
    return;
  }

  servoConnected = true;
  Serial.print("Servo Connected with ID: ");
  Serial.println(detectedID);

  int torqueAcknowledged = servo.EnableTorque(servo_ID, 1);
  Serial.print("Torque Enable Acknowledged: ");
  Serial.println(torqueAcknowledged == 1 ? "YES" : "NO");

  // The servo sends voltage in tenths, so divide by 10 to get volts
  int rawVoltage = servo.ReadVoltage(servo_ID);
  Serial.print("Servo Supply Voltage: ");

  if (rawVoltage == -1) {
    Serial.println("READ FAILED");
  } else {
    Serial.print(rawVoltage / 10.0, 1);
    Serial.println(" V");
  }

  int temperatureC = servo.ReadTemper(servo_ID);
  Serial.print("Servo Temperature: ");

  if (temperatureC == -1) {
    Serial.println("READ FAILED");
  } else {
    Serial.print(temperatureC);
    Serial.println(" C");
  }
}

void printFlexionTuningSummary() {
  Serial.println("\n---------------- FLEXION CALIBRATION ----------------");
  Serial.print("Twist factor for 0 to 90 deg: ");
  Serial.println(flexionTwistFactor0To90, 4);
  Serial.print("Twist factor for over 90 to 180 deg: ");
  Serial.println(flexionTwistFactor90To180, 4);
  Serial.print("Twist factor for over 180 to 270 deg: ");
  Serial.println(flexionTwistFactor180To270, 4);
  Serial.print("Flexion gear ratio: ");
  Serial.println(flexionGearRatio, 4);
  Serial.print("Allowed absolute tip range: ");
  Serial.print(flexionMinTipDeg, 2);
  Serial.print(" deg to ");
  Serial.print(flexionMaxTipDeg, 2);
  Serial.println(" deg");
  Serial.print("Representable ST3020 motor range: ");
  Serial.print(flexionMinMotorDeg, 2);
  Serial.print(" deg to ");
  Serial.print(flexionMaxMotorDeg, 2);
  Serial.println(" deg");
  Serial.print("Speed reduction per tip degree beyond +/-100 deg: ");
  Serial.println(flexionSpeedReductionPerTipDeg, 1);
  Serial.println("Edit the three factors in the FLEXION CALIBRATION block near the top of this file.");
  Serial.println("-----------------------------------------------------");
}

void setup() {

  Serial.begin(115200);
  Serial.setTimeout(500);

  setupLaserOutputs();

  linearSetup();
  rotationSetup();
  flexionSetup();

  // Use the starting position as 0 mm
  currentLinearPositionMM = linearMinPositionMM;

  // Use the starting rotation as the flat position
  currentRotationMapDeg = 0.0;

  delay(1000);

  diagnostics();
  printFlexionTuningSummary();

  printLinearZeroReminder();
  printRotationNeutralReminder();

  Serial.println("\nCombined Movement + Laser Setup Complete");
  Serial.println("Enter movement as: Linear(mm) Rotation(deg) Flexion(deg)");
  Serial.println("Example: 4 90 20");
  Serial.println("Type home or h to return to the home state");
  printLaserCommandHelp();
}

void printLinearZeroReminder() {
  Serial.println("\n---------------- LINEAR ZERO REMINDER ----------------");
  Serial.println("Before sending movement commands, make sure the TR8x4 lead screw begins at 0 mm");
  Serial.println("This code will treat the startup position as absolute 0 mm");
  Serial.print("Allowed absolute linear range: ");
  Serial.print(linearMinPositionMM, 1);
  Serial.print(" mm to ");
  Serial.print(linearMaxPositionMM, 1);
  Serial.println(" mm");
  Serial.println("------------------------------------------------------");
}

void printRotationNeutralReminder() {
  Serial.println("\n-------------- ROTATION NEUTRAL REMINDER ------------");
  Serial.println("Before sending movement commands, make sure the ureteroscope is flat at rotation neutral");
  Serial.println("This code does not enforce rotation min/max range");
  Serial.println("This startup flat position becomes 0 degrees on the rotation map");
  Serial.println("Rotation commands are relative, but the code tracks mapped absolute rotation");
  Serial.println("A full revolution back to 0 in either direction resets the map to 0 degrees");
  Serial.println("------------------------------------------------------");
}

// Read and check a command, then run the move or laser command
void loop() {
  printCombinedCommandPrompt();

  while (Serial.available() == 0) {
  }

  char inputLine[64];

  if (!readSerialLine(inputLine, sizeof(inputLine))) {
    return;
  }

  printReceivedInput(inputLine);

  char singleLetterCommand = getSingleLetterCommand(inputLine);

  if (handleLaserCommand(singleLetterCommand)) {
    clearSerialInput();
    return;
  }

  // Stop if a move or an unknown command is entered while firing
  if (laserFiring) {
    Serial.println("Movement or unknown input is not allowed while firing. Emergency stop triggered.");
    stopLaserAndRelock(true);
    Serial.println("All laser outputs are OFF. Laser commands are locked.");
    clearSerialInput();
    return;
  }

  if (!readMovementCommandFromLine(inputLine)) {
    Serial.println("Wrong Format");
    Serial.println("Movement format: Linear(mm) Rotation(deg) Flexion(deg)");
    Serial.println("Example: 4 90 20");
    Serial.println("Next: type a valid movement command, home/h, or a laser command.");
    printLaserCommandHelp();
    clearSerialInput();
    return;
  }

  if (homeCommandAlreadyAtHome) {
    Serial.println("Home command ignored: the software already believes the ureteroscope is at home.");
    Serial.println("No movement was started.");
    Serial.println("Next: type a movement command, a laser command, or x for laser stop.");
    clearSerialInput();
    return;
  }

  if (!calculateAllTargets()) {
    clearSerialInput();
    return;
  }

  moveAllAxesTogether();
  clearSerialInput();
}

void printCombinedCommandPrompt() {
  printCommandPrompt();
  printLaserCommandHelp();
}

void printReceivedInput(const char inputLine[]) {
  Serial.print("Serial Monitor input received: [");
  Serial.print(inputLine);
  Serial.println("]");
}

void printCommandPrompt() {

  Serial.println("\nPOSITIONING: Linear = RELATIVE | Rotation = RELATIVE | Flexion = ABSOLUTE");
  Serial.println("Enter Linear Additional(mm) Rotation Additional(deg) Flexion Target(deg):");

  Serial.print("Current absolute linear position: ");
  Serial.print(currentLinearPositionMM, 3);
  Serial.println(" mm");

  Serial.print("Current mapped rotation position: ");
  Serial.print(currentRotationMapDeg, 2);
  Serial.println(" deg");

  Serial.print("Linear range allowed: ");
  Serial.print(linearMinPositionMM, 1);
  Serial.print(" mm to ");
  Serial.print(linearMaxPositionMM, 1);
  Serial.println(" mm");

  Serial.println("Return to home state with: home OR h");
}

// Read a line from Serial Monitor and skip it if it is blank
bool readSerialLine(char inputLine[], size_t inputLineSize) {
  size_t inputLength = Serial.readBytesUntil('\n', inputLine, inputLineSize - 1);

  // Remove the extra carriage return if Serial Monitor sends both line endings
  if (inputLength > 0 && inputLine[inputLength - 1] == '\r') {
    inputLength--;
  }

  inputLine[inputLength] = '\0';

  for (size_t i = 0; i < inputLength; i++) {
    if (inputLine[i] != ' ' && inputLine[i] != '\t' && inputLine[i] != '\r') {
      return true;
    }
  }

  return false;
}

// Allow spaces and either uppercase or lowercase for letter commands
char getSingleLetterCommand(const char inputLine[]) {
  int inputStart = 0;

  while (inputLine[inputStart] == ' ' || inputLine[inputStart] == '\t' || inputLine[inputStart] == '\r') {
    inputStart++;
  }

  char command = inputLine[inputStart];

  if (command == '\0') {
    return '\0';
  }

  int inputEnd = inputStart + 1;

  while (inputLine[inputEnd] == ' ' || inputLine[inputEnd] == '\t' || inputLine[inputEnd] == '\r') {
    inputEnd++;
  }

  if (inputLine[inputEnd] != '\0') {
    return '\0';
  }

  if (command >= 'A' && command <= 'Z') {
    command = command + 32;
  }

  return command;
}

bool readMovementCommandFromLine(const char inputLine[]) {

  homeCommandRequested = false;
  homeCommandAlreadyAtHome = false;

  if (inputLineEqualsWord(inputLine, "home") || inputLineEqualsWord(inputLine, "h")) {
    if (softwareIsAtHome()) {
      homeCommandRequested = true;
      homeCommandAlreadyAtHome = true;
      return true;
    }

    setHomeCommandTargets();
    return true;
  }

  // Make sure there are exactly three valid numbers and no extra text
  char extraCharacter;
  int valuesRead = sscanf(inputLine, " %f %f %f %c", &requestedLinearMM, &requestedRotationDeg, &requestedFlexionDeg, &extraCharacter);

  if (valuesRead != 3) {
    return false;
  }

  if (!isfinite(requestedLinearMM) || !isfinite(requestedRotationDeg) || !isfinite(requestedFlexionDeg)) {
    return false;
  }

  return true;
}

void clearSerialInput() {

  while (Serial.available() > 0) {
    Serial.read();
  }
}

// Ignore uppercase/lowercase and extra spaces when checking a command
bool inputLineEqualsWord(const char inputLine[], const char word[]) {

  int inputStart = 0;

  while (inputLine[inputStart] == ' ' || inputLine[inputStart] == '\t' || inputLine[inputStart] == '\r') {
    inputStart++;
  }

  int inputEnd = inputStart;

  while (inputLine[inputEnd] != '\0') {
    inputEnd++;
  }

  while (inputEnd > inputStart &&
         (inputLine[inputEnd - 1] == ' ' ||
          inputLine[inputEnd - 1] == '\t' ||
          inputLine[inputEnd - 1] == '\r')) {
    inputEnd--;
  }

  int wordLength = 0;

  while (word[wordLength] != '\0') {
    wordLength++;
  }

  if (inputEnd - inputStart != wordLength) {
    return false;
  }

  for (int i = 0; i < wordLength; i++) {

    char inputChar = inputLine[inputStart + i];

    char wordChar = word[i];

    if (inputChar >= 'A' && inputChar <= 'Z') {
      inputChar = inputChar + 32;
    }

    if (wordChar >= 'A' && wordChar <= 'Z') {
      wordChar = wordChar + 32;
    }

    if (inputChar != wordChar) {
      return false;
    }
  }

  return true;
}

// Check if the saved positions are close enough to home, allowing for rounding
// If there is no tip reading, this check skips the tip
bool softwareIsAtHome() {

  bool linearAtHome =
    fabsf(currentLinearPositionMM - linearMinPositionMM) <= linearRangeToleranceMM;

  bool rotationAtHome =
    fabsf(currentRotationMapDeg) <= rotationMapZeroToleranceDeg;

  bool flexionAtHome =
    !servoConnected ||
    currentFlexionStep == -1 ||
    fabsf(measuredTipDeg) <= flexionRangeToleranceTipDeg;

  return linearAtHome && rotationAtHome && flexionAtHome;
}

// Work out how far to move and turn to get back to zero
// The tip uses a target angle, so set that angle to zero for Home
void setHomeCommandTargets() {

  homeCommandRequested = true;

  requestedLinearMM = linearMinPositionMM - currentLinearPositionMM;
  requestedRotationDeg = -currentRotationMapDeg;
  requestedFlexionDeg = 0.0;

  Serial.println("\nHome command received");
  Serial.println("Target home state: Linear 0 mm, Rotation 0 deg, Flexion 0 deg");
}

// Check all three requested positions before moving anything
bool calculateAllTargets() {

  if (!calculateLinearTarget()) {
    return false;
  }

  if (!calculateRotationTarget()) {
    return false;
  }

  if (!calculateFlexionTarget()) {
    return false;
  }

  return true;
}

bool calculateLinearTarget() {

  targetLinearRelativeMM = requestedLinearMM;

  // Add the new distance to the current saved position
  targetLinearAbsoluteMM = currentLinearPositionMM + targetLinearRelativeMM;

  if (!linearTargetIsInsideRange()) {
    return false;
  }

  linearMovementSign = (targetLinearRelativeMM > 0.0 ? +1 : -1);

  targetLinearMagnitudeMM = fabsf(targetLinearRelativeMM);

  // Work out how many motor turns and step pulses the distance needs
  targetLinearRevolutions = targetLinearMagnitudeMM / linearMMPerRev;

  targetLinearPulses = lroundf(targetLinearRevolutions * linearPulsesPerRev);

  linearDone = (targetLinearPulses == 0);

  return true;
}

// Reject a move past the travel limits, with a little room for rounding
bool linearTargetIsInsideRange() {

  bool belowMinimum = targetLinearAbsoluteMM < linearMinPositionMM - linearRangeToleranceMM;
  bool aboveMaximum = targetLinearAbsoluteMM > linearMaxPositionMM + linearRangeToleranceMM;

  if (belowMinimum || aboveMaximum) {
    Serial.println("\nLinear Range Error");
    Serial.println("No motors will move because the linear command would leave the allowed range");

    Serial.print("Current absolute linear position: ");
    Serial.print(currentLinearPositionMM, 3);
    Serial.println(" mm");

    Serial.print("Requested additional linear movement: ");
    Serial.print(targetLinearRelativeMM, 3);
    Serial.println(" mm");

    Serial.print("Requested absolute linear position: ");
    Serial.print(targetLinearAbsoluteMM, 3);
    Serial.println(" mm");

    Serial.print("Allowed absolute linear range: ");
    Serial.print(linearMinPositionMM, 1);
    Serial.print(" mm to ");
    Serial.print(linearMaxPositionMM, 1);
    Serial.println(" mm");

    return false;
  }

  if (targetLinearAbsoluteMM < linearMinPositionMM) {
    targetLinearAbsoluteMM = linearMinPositionMM;
  }

  if (targetLinearAbsoluteMM > linearMaxPositionMM) {
    targetLinearAbsoluteMM = linearMaxPositionMM;
  }

  return true;
}

bool calculateRotationTarget() {

  targetUreteroscopeRotationDeg = requestedRotationDeg;

  rotationMovementSign = (targetUreteroscopeRotationDeg > 0.0 ? +1 : -1);

  // Use the gear ratio to work out how far the motor needs to turn
  targetRotationMotorDeg = rotationGearRatio * targetUreteroscopeRotationDeg;

  targetRotationMotorMagnitudeDeg = fabsf(targetRotationMotorDeg);

  // Estimate a backup pulse limit; the encoder reading is what normally stops the move
  targetRotationPulses = lroundf((targetRotationMotorMagnitudeDeg / degreesPerRevolution) * rotationPulsesPerRev);

  rotationDone = (targetRotationPulses == 0);

  return true;
}

// Pick a calibration value based on how far the tip should bend
float selectFlexionTwistFactor(float tipDeg) {

  float absoluteTipDeg = fabsf(tipDeg);

  if (absoluteTipDeg <= 90.0) {
    return flexionTwistFactor0To90;
  }

  if (absoluteTipDeg <= 180.0) {
    return flexionTwistFactor90To180;
  }

  return flexionTwistFactor180To270;
}

bool calculateFlexionTarget() {

  targetTipDeg = requestedFlexionDeg;

  selectedFlexionTwistFactor = selectFlexionTwistFactor(targetTipDeg);
  selectedFlexionTipGain = selectedFlexionTwistFactor / flexionGearRatio;

  // Work out the servo angle needed for the requested tip bend
  targetFlexionMotorDeg = targetTipDeg / selectedFlexionTipGain;

  if (!flexionTargetIsInsideRange()) {
    return false;
  }

  // Shift the angle to match the position scale used by the servo
  targetFlexionPhysicalDeg = targetFlexionMotorDeg + halfRevolutionDegrees;

  targetFlexionSteps = lroundf((targetFlexionPhysicalDeg * servoMaximumStep) / degreesPerRevolution);

  selectedFlexionSpeed = calculateFlexionSpeed();

  printFlexionOverstretchWarning();

  flexionDone = !servoConnected;

  return true;
}

// Slow down bigger bends, but keep the speed at or above the minimum
int calculateFlexionSpeed() {

  float absoluteTargetTipDeg = fabsf(targetTipDeg);

  if (absoluteTargetTipDeg <= flexionSlowdownStartTipDeg) {
    return servo_speed;
  }

  float degreesBeyondSlowdown = absoluteTargetTipDeg - flexionSlowdownStartTipDeg;

  int speedReduction = lroundf(degreesBeyondSlowdown * flexionSpeedReductionPerTipDeg);
  int calculatedSpeed = servo_speed - speedReduction;

  if (calculatedSpeed < flexionMinimumSpeed) {
    calculatedSpeed = flexionMinimumSpeed;
  }

  return calculatedSpeed;
}

// Only show the strongest warning for the requested bend
void printFlexionOverstretchWarning() {

  float absoluteTargetTipDeg = fabsf(targetTipDeg);

  if (absoluteTargetTipDeg >= flexionMaxTipDeg - flexionRangeToleranceTipDeg) {
    Serial.println("WARNING: Reaching maximum allowed flexion");
  } else if (absoluteTargetTipDeg >= flexionMajorWarningTipDeg) {
    Serial.println("MAJOR PRECAUTION: Flexion target is near the maximum and may overstretch the ureteroscope");
  } else if (absoluteTargetTipDeg >= flexionModerateWarningTipDeg) {
    Serial.println("WARNING: High flexion target may overstretch the ureteroscope");
  } else if (absoluteTargetTipDeg >= flexionSmallWarningTipDeg) {
    Serial.println("CAUTION: Flexion target is entering the overstretch precaution range");
  }
}

// Check that both the tip angle and servo angle are within their limits
bool flexionTargetIsInsideRange() {

  bool belowTipMinimum = targetTipDeg < flexionMinTipDeg;
  bool aboveTipMaximum = targetTipDeg > flexionMaxTipDeg;
  bool belowMotorMinimum = targetFlexionMotorDeg < flexionMinMotorDeg;
  bool aboveMotorMaximum = targetFlexionMotorDeg > flexionMaxMotorDeg;

  if (belowTipMinimum || aboveTipMaximum || belowMotorMinimum || aboveMotorMaximum) {
    Serial.println("\nFlexion Range Error");
    Serial.println("No motors will move because the requested flexion is outside the allowed range");

    Serial.print("Requested tip flexion: ");
    Serial.print(targetTipDeg, 2);
    Serial.println(" deg");

    Serial.print("Required ST3020 motor angle: ");
    Serial.print(targetFlexionMotorDeg, 2);
    Serial.println(" deg");

    Serial.print("Selected flexionTwistFactor: ");
    Serial.println(selectedFlexionTwistFactor, 4);

    Serial.print("Selected flexionTipGain: ");
    Serial.println(selectedFlexionTipGain, 4);

    Serial.print("Representable ST3020 motor range: ");
    Serial.print(flexionMinMotorDeg, 1);
    Serial.print(" deg to ");
    Serial.print(flexionMaxMotorDeg, 1);
    Serial.println(" deg");

    Serial.print("Allowed requested tip range: ");
    Serial.print(flexionMinTipDeg, 1);
    Serial.print(" deg to ");
    Serial.print(flexionMaxTipDeg, 1);
    Serial.println(" deg");

    return false;
  }

  return true;
}

// When channel A goes HIGH, read B to decide which way to count
void linearEncoderISR() {

  if (digitalRead(linear_encB) == HIGH) {
    linearEncoderCount++;
  } else {
    linearEncoderCount--;
  }
}

// Read channel B to see which way the rotation motor is turning
void rotationEncoderISR() {

  if (digitalRead(rotation_encB) == HIGH) {
    rotationEncoderCount++;
  } else {
    rotationEncoderCount--;
  }
}

// Pause interrupts for a moment so the count stays still while it is copied
long getLinearEncoderCount() {

  noInterrupts();
  long count = linearEncoderCount;
  interrupts();
  return count;
}

// Copy the rotation count while the encoder interrupt is paused
long getRotationEncoderCount() {

  noInterrupts();
  long count = rotationEncoderCount;
  interrupts();
  return count;
}

// Pause interrupts for a moment while setting the encoder count back to zero
void resetLinearEncoder() {

  noInterrupts();
  linearEncoderCount = 0;
  interrupts();
}

// Set the rotation count back to zero between commands
void resetRotationEncoder() {

  noInterrupts();
  rotationEncoderCount = 0;
  interrupts();
}

// Start the moves together, check the readings, and save where they finish
void moveAllAxesTogether() {

  Serial.println("\nStarting 3-DOF Movement");

  resetMovementMeasurements();
  printRequestedTargets();
  prepareMotorsForMovement();
  runSimultaneousMovementLoop();
  updateAbsoluteLinearPosition();
  updateAbsoluteRotationPosition();
  printFinalSummary();
}

// Clear the old readings and timers before starting another move
void resetMovementMeasurements() {

  linearPulsesSent = 0;
  rotationPulsesSent = 0;

  signedLinearCount = 0;
  signedRotationCount = 0;

  measuredLinearMotorDeg = 0.0;
  measuredLinearUreteroMM = 0.0;
  measuredRotationMotorDeg = 0.0;
  measuredUreteroRotationDeg = 0.0;

  currentFlexionStep = -1;
  lastMovingFlexionStep = -1;
  measuredFlexionMotorDeg = 0.0;
  measuredTipDeg = 0.0;

  lastPrintTime = 0;
  lastFlexionReadTime = 0;
  lastFlexionChangeTime = millis();
  movementEmergencyStop = false;
}

void printRequestedTargets() {

  Serial.print("Linear current absolute position: ");
  Serial.print(currentLinearPositionMM, 3);
  Serial.println(" mm");

  Serial.print("Linear requested additional movement: ");
  Serial.print(targetLinearRelativeMM, 3);
  Serial.println(" mm");

  Serial.print("Linear target absolute position: ");
  Serial.print(targetLinearAbsoluteMM, 3);
  Serial.println(" mm");

  Serial.print("Linear Motor Revolutions: ");
  Serial.println(targetLinearRevolutions, 3);

  Serial.print("Linear Estimated STEP Pulses: ");
  Serial.println(targetLinearPulses);

  Serial.print("Rotation Target: ");
  Serial.print(targetUreteroscopeRotationDeg, 2);
  Serial.println(" deg ureteroscope");

  Serial.print("Rotation Map Target: ");
  Serial.print(normalizeRotationMapDeg(currentRotationMapDeg + targetUreteroscopeRotationDeg), 2);
  Serial.println(" deg");

  Serial.print("Rotation Required Motor Angle: ");
  Serial.print(targetRotationMotorDeg, 2);
  Serial.println(" deg motor");

  Serial.print("Rotation Estimated STEP Pulses: ");
  Serial.println(targetRotationPulses);

  Serial.print("Flexion Target: ");
  Serial.print(targetTipDeg, 2);
  Serial.println(" deg tip");

  Serial.print("Flexion Selected Twist Factor: ");
  Serial.println(selectedFlexionTwistFactor, 4);

  Serial.print("Flexion Selected Tip Gain: ");
  Serial.println(selectedFlexionTipGain, 4);

  Serial.print("Flexion Required Motor Angle: ");
  Serial.print(targetFlexionMotorDeg, 2);
  Serial.println(" deg motor");

  Serial.print("Flexion ST3020 Target Step: ");
  Serial.println(targetFlexionSteps);

  Serial.print("Flexion Selected Speed: ");
  Serial.println(selectedFlexionSpeed);
}

// Set the directions, reset the counts, and turn on the motor drivers
void prepareMotorsForMovement() {

  if (!linearDone) {
    digitalWrite(linear_dirPin, linearMovementSign > 0 ? linear_dirForward : linear_dirBackward);
  }

  if (!rotationDone) {
    digitalWrite(rotation_dirPin, rotationMovementSign > 0 ? rotation_dirClockwise : rotation_dirCounter);
  }

  delayMicroseconds(directionSetup_us);

  resetLinearEncoder();
  resetRotationEncoder();

  digitalWrite(linear_enaPin, enaPin_Enable);
  digitalWrite(rotation_enaPin, enaPin_Enable);

  startFlexionMovement();
}

// The servo only needs one move command while the steppers need repeated pulses
void startFlexionMovement() {

  if (!servoConnected) {
    Serial.println("Flexion Skipped because ST3020 Communication is Unavailable");
    flexionDone = true;
    return;
  }

  int torqueAcknowledged = servo.EnableTorque(servo_ID, 1);

  if (torqueAcknowledged != 1) {
    Serial.println("ST3020 did not Acknowledge Torque Enable");
    flexionDone = true;
    return;
  }

  int commandAcknowledged = servo.WritePosEx(servo_ID, targetFlexionSteps, selectedFlexionSpeed, servo_acceleration);

  if (commandAcknowledged != 1) {
    Serial.println("ST3020 did not Acknowledge the Movement Command");
    flexionDone = true;
    return;
  }

  lastFlexionChangeTime = millis();
}

void runSimultaneousMovementLoop() {

  // Stop after 300 extra pulses if the encoder still has not reached the target
  // Those extra pulses allow up to 0.75 mm of travel or 27 degrees of scope rotation
  long linearBackupPulseLimit = targetLinearPulses + 300;
  long rotationBackupPulseLimit = targetRotationPulses + 300;

  while (!linearDone || !rotationDone || !flexionDone) {
    pulseActiveSteppers();
    checkForEmergencyStopDuringMovement();

    if (movementEmergencyStop) {
      return;
    }

    readLinearFeedback();
    readRotationFeedback();
    readFlexionFeedbackIfReady();

    if (!linearDone && linearPulsesSent >= linearBackupPulseLimit) {
      Serial.println("Linear stopped because encoder target was not reached");
      linearDone = true;
    }

    if (!rotationDone && rotationPulsesSent >= rotationBackupPulseLimit) {
      Serial.println("Rotation stopped because encoder target was not reached");
      rotationDone = true;
    }

    printMovementStatusIfReady();

    if (linearDone && rotationDone && !flexionDone) {
      delay(5);
    }
  }
}

// Send pulses to both unfinished steppers in the same loop so they move together
void pulseActiveSteppers() {

  bool pulseLinear = !linearDone;
  bool pulseRotation = !rotationDone;

  if (pulseLinear) {
    digitalWrite(linear_stepPin, HIGH);
  }

  if (pulseRotation) {
    digitalWrite(rotation_stepPin, HIGH);
  }

  if (pulseLinear || pulseRotation) {
    delayMicroseconds(stepDelay_us);
  }

  if (pulseLinear) {
    digitalWrite(linear_stepPin, LOW);
    linearPulsesSent++;
  }

  if (pulseRotation) {
    digitalWrite(rotation_stepPin, LOW);
    rotationPulsesSent++;
  }

  if (pulseLinear || pulseRotation) {
    delayMicroseconds(stepDelay_us);
  }
}

// Turn the encoder count into distance and stop once the scope has moved far enough
void readLinearFeedback() {

  if (linearDone) {
    return;
  }

  long rawLinearCount = getLinearEncoderCount();

  signedLinearCount = linearEncoderSign * rawLinearCount;

  measuredLinearMotorDeg = (signedLinearCount * degreesPerRevolution) / linearEncoderCPR;

  measuredLinearUreteroMM = (signedLinearCount * linearMMPerRev) / linearEncoderCPR;

  if (fabsf(measuredLinearUreteroMM) >= targetLinearMagnitudeMM - linearStopMarginMM) {
    linearDone = true;
  }
}

// Use the encoder reading and gear ratio to find how far the scope has turned
void readRotationFeedback() {

  if (rotationDone) {
    return;
  }

  long rawRotationCount = getRotationEncoderCount();

  signedRotationCount = rotationEncoderSign * rawRotationCount;

  measuredRotationMotorDeg = (signedRotationCount * degreesPerRevolution) / rotationEncoderCPR;

  measuredUreteroRotationDeg = measuredRotationMotorDeg / rotationGearRatio;

  if (fabsf(measuredRotationMotorDeg) >= targetRotationMotorMagnitudeDeg - rotationStopMarginMotorDeg) {
    rotationDone = true;
  }
}

// Leave time between servo readings so there are not too many serial messages
void readFlexionFeedbackIfReady() {

  if (flexionDone) {
    return;
  }

  if (millis() - lastFlexionReadTime < printInterval_ms) {
    return;
  }

  currentFlexionStep = servo.ReadPos(servo_ID);

  if (currentFlexionStep == -1) {
    Serial.println("ST3020 Position Read Failed");
    flexionDone = true;
    return;
  }

  if (lastMovingFlexionStep == -1 || currentFlexionStep != lastMovingFlexionStep) {
    lastMovingFlexionStep = currentFlexionStep;
    lastFlexionChangeTime = millis();
  }

  measuredFlexionMotorDeg =
    (currentFlexionStep * degreesPerRevolution / (float)servoMaximumStep) - halfRevolutionDegrees;

  // Estimate the tip bend using the calibration picked for this move
  measuredTipDeg = measuredFlexionMotorDeg * selectedFlexionTipGain;

  if (abs(currentFlexionStep - targetFlexionSteps) <= flexionStopMarginSteps) {
    flexionDone = true;
  }

  // Stop waiting if the servo reading stays the same before reaching the target
  if (!flexionDone && millis() - lastFlexionChangeTime > flexionNoChangeTimeout_ms) {
    Serial.println("Flexion stopped moving before reaching the exact target");
    Serial.println("Ending movement so the Serial Monitor does not print forever");
    flexionDone = true;
  }

  lastFlexionReadTime = millis();
}

// Space out the status messages so printing does not hold up the step pulses
void printMovementStatusIfReady() {

  if (millis() - lastPrintTime < printInterval_ms) {
    return;
  }

  Serial.print("Linear Abs mm, Rel mm, Count: ");
  Serial.print(currentLinearPositionMM + measuredLinearUreteroMM, 3);
  Serial.print(", ");
  Serial.print(measuredLinearUreteroMM, 3);
  Serial.print(", ");
  Serial.print(signedLinearCount);

  Serial.print(" | Rotation Motor Deg, Ureteroscope Deg, Count: ");
  Serial.print(measuredRotationMotorDeg, 2);
  Serial.print(", ");
  Serial.print(measuredUreteroRotationDeg, 2);
  Serial.print(", ");
  Serial.print(signedRotationCount);

  Serial.print(" | Rotation Map Deg: ");
  Serial.print(normalizeRotationMapDeg(currentRotationMapDeg + measuredUreteroRotationDeg), 2);

  Serial.print(" | Flexion Step, Motor Deg, Tip Deg: ");
  Serial.print(currentFlexionStep);
  Serial.print(", ");
  Serial.print(measuredFlexionMotorDeg, 2);
  Serial.print(", ");
  Serial.println(measuredTipDeg, 2);

  lastPrintTime = millis();
}

// Save how far the scope actually moved, using the encoder reading
void updateAbsoluteLinearPosition() {

  float previousLinearPositionMM = currentLinearPositionMM;
  currentLinearPositionMM = currentLinearPositionMM + measuredLinearUreteroMM;

  if (currentLinearPositionMM < linearMinPositionMM && currentLinearPositionMM > linearMinPositionMM - linearRangeToleranceMM) {
    currentLinearPositionMM = linearMinPositionMM;
  }

  if (currentLinearPositionMM > linearMaxPositionMM && currentLinearPositionMM < linearMaxPositionMM + linearRangeToleranceMM) {
    currentLinearPositionMM = linearMaxPositionMM;
  }

  Serial.print("Linear absolute position updated from ");
  Serial.print(previousLinearPositionMM, 3);
  Serial.print(" mm to ");
  Serial.print(currentLinearPositionMM, 3);
  Serial.println(" mm");
}

// Add how far the scope turned and bring full turns back around to zero
void updateAbsoluteRotationPosition() {

  float previousRotationMapDeg = currentRotationMapDeg;
  currentRotationMapDeg = normalizeRotationMapDeg(currentRotationMapDeg + measuredUreteroRotationDeg);

  Serial.print("Rotation map updated from ");
  Serial.print(previousRotationMapDeg, 2);
  Serial.print(" deg to ");
  Serial.print(currentRotationMapDeg, 2);
  Serial.println(" deg");
}

void printFinalSummary() {
  if (movementEmergencyStop) {
    Serial.println("\n3-DOF Movement Stopped by Emergency Serial Command");
  } else {
    Serial.println("\n3-DOF Movement Complete");
  }

  Serial.print("Final Linear Absolute Position: ");
  Serial.print(currentLinearPositionMM, 3);
  Serial.println(" mm");

  Serial.print("Final Linear Relative Motion, Motor Deg, Count: ");
  Serial.print(measuredLinearUreteroMM, 3);
  Serial.print(", ");
  Serial.print(measuredLinearMotorDeg, 2);
  Serial.print(", ");
  Serial.println(signedLinearCount);

  Serial.print("Final Rotation Motor Deg, Ureteroscope Deg, Count: ");
  Serial.print(measuredRotationMotorDeg, 2);
  Serial.print(", ");
  Serial.print(measuredUreteroRotationDeg, 2);
  Serial.print(", ");
  Serial.println(signedRotationCount);

  Serial.print("Final Rotation Map Position: ");
  Serial.print(currentRotationMapDeg, 2);
  Serial.println(" deg");

  Serial.print("Final Flexion Step, Motor Deg, Tip Deg: ");
  Serial.print(currentFlexionStep);
  Serial.print(", ");
  Serial.print(measuredFlexionMotorDeg, 2);
  Serial.print(", ");
  Serial.println(measuredTipDeg, 2);
}

// Keep the saved angle within one full turn in either direction
// This only wraps the number; it does not stop the scope from turning farther
float normalizeRotationMapDeg(float angleDeg) {

  while (angleDeg >= degreesPerRevolution) {
    angleDeg = angleDeg - degreesPerRevolution;
  }

  while (angleDeg <= -degreesPerRevolution) {
    angleDeg = angleDeg + degreesPerRevolution;
  }

  if (fabsf(angleDeg) <= rotationMapZeroToleranceDeg) {
    angleDeg = 0.0;
  }

  return angleDeg;
}

// Turn off both stepper drivers and release the servo if it is connected
void disableAllMotors() {

  digitalWrite(linear_enaPin, enaPin_Disable);
  digitalWrite(rotation_enaPin, enaPin_Disable);
  digitalWrite(linear_stepPin, LOW);
  digitalWrite(rotation_stepPin, LOW);

  if (servoConnected) {
    servo.EnableTorque(servo_ID, 0);
  }
}
