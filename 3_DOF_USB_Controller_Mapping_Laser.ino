/*
GLiDAR USB Controller Mapping
Description: Set up a USB controller by checking its sticks and buttons
             Use it to move the scope in/out, turn it, and bend the tip
             Read motor positions, go back to saved home positions, and send laser commands
Components: Teensy, USB gamepad, two steppers, TB6600 drivers, encoders,
            ST3020 servo and Dornier laser interface
*/

#include <USBHost_t36.h>
#include <SCServo.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

SMS_STS servo;

// Pins

// Motor, driver, and encoder for moving the scope in and out
const int linear_enaPin = 2;   // Driver enable
const int linear_dirPin = 3;   // Direction
const int linear_stepPin = 4;  // Step pulses
const int linear_encA = 17;    // Encoder A
const int linear_encB = 16;    // Encoder B

// Motor, driver, and encoder for turning the scope
const int rotation_enaPin = 5;   // Driver enable
const int rotation_dirPin = 6;   // Direction
const int rotation_stepPin = 7;  // Step pulses
const int rotation_encA = 20;    // Encoder A
const int rotation_encB = 21;    // Encoder B

// Pins for the Dornier laser commands
const int relay_laser = 8;   // Ready/standby relay
const int fire1_laser = 9;   // Fire channel 1
const int fire2_laser = 10;  // Fire channel 2

// Wiring settings

// With this wiring, LOW turns the TB6600 drivers on
const int enaPin_Enable = LOW;

const int enaPin_Disable = HIGH;

const int linear_dirForward = HIGH;
const int linear_dirBackward = LOW;

const int rotation_dirClockwise = LOW;
const int rotation_dirCounter = HIGH;

// LOW closes the Ready/Standby relay and HIGH opens it
const int LASER_RELAY_ACTIVE = LOW;
const int LASER_RELAY_RELEASED = HIGH;

// HIGH turns the fire outputs on and LOW turns them off
const int LASER_FIRE_ACTIVE = HIGH;
const int LASER_FIRE_RELEASED = LOW;

// Use the same relay pulse for Ready and Standby
const unsigned long laserControlPulse_ms = 3000;

// Settings for moving the scope in and out

const float linearMMPerRev = 4.0f;
const float linearPulsesPerRev = 1600.0f;
const float linearEncoderCPR = 1000.0f;
const float linearMinPositionMM = 0.0f;
const float linearMaxPositionMM = 177.0f;
const float linearRangeToleranceMM = 0.01f;
const int linearEncoderSign = +1;

// Settings for turning the scope

const float rotationEncoderCPR = 1000.0f;
const float rotationPulsesPerRev = 1600.0f;
const float rotationGearRatio = 2.5f;
const int rotationEncoderSign = -1;
const float rotationMapZeroToleranceDeg = 0.5f;

const float rotationMinPositionDeg = -180.0f;
const float rotationMaxPositionDeg = 180.0f;
const float rotationRangeToleranceDeg = 0.5f;

// The tip bends differently in each angle range, so each range gets its own ratio

const float flexionTwistFactor0To90 = 6.0f;
const float flexionTwistFactor90To180 = 6.5f;
const float flexionTwistFactor180To270 = 7.2f;

// Use the gear ratio to work out how much the servo twists the cable
const float flexionGearRatio = 3.5f;

// Changing the twist factors also updates these tip-bending ratios
const float flexionTipGain0To90 = flexionTwistFactor0To90 / flexionGearRatio;
const float flexionTipGain90To180 = flexionTwistFactor90To180 / flexionGearRatio;
const float flexionTipGain180To270 = flexionTwistFactor180To270 / flexionGearRatio;

// Gradually switch ratios near 90 and 180 degrees so bending changes smoothly
// The change happens over 75-105 and 165-195 degrees
const float flexionTransitionBlendWidthDeg = 30.0f;

// Slow down the bend target near either angle limit
const float flexionSoftLimitZoneDeg = 10.0f;

// ST3020 servo settings

const long servo_baud = 1000000;
const int servo_ID = 1;
const int servo_speed = 500;
const int servo_acceleration = 100;
const int servoMaximumStep = 4095;
const int flexionNeutralStep = 2048;
const float degreesPerRevolution = 360.0f;
const float halfRevolutionDegrees = 180.0f;

// Slow down at larger bends

const int flexionSlowdownStartTipDeg = 100;
const float flexionSpeedReductionPerTipDeg = 2.5f;
const int flexionMinimumSpeed = 50;

// Change servo speed gradually; a larger value lets it change faster
const float flexionServoSpeedRampUnitsPerSec = 300.0f;

// Bend angles that trigger warnings

const int flexionSmallWarningTipDeg = 150;
const int flexionModerateWarningTipDeg = 200;
const int flexionMajorWarningTipDeg = 250;

// Limits for bending the tip

const float flexionMaxTipDeg = 270.0f;
const float flexionMinTipDeg = -270.0f;
const float flexionMaxMotorDeg = 180.0f;
const float flexionMinMotorDeg = -180.0f;
const int flexionStopMarginSteps = 1;
const float flexionRangeToleranceMotorDeg = degreesPerRevolution / (float)servoMaximumStep;
const float flexionRangeToleranceTipDeg =
  flexionRangeToleranceMotorDeg * flexionTipGain180To270;

// Speed settings

// Fastest in/out speed in mm/s
const float maximumLinearSpeedMMPerSec = 2.0f;

// Fastest turning speed in degrees/s
const float maximumRotationSpeedDegPerSec = 30.0f;

// Fastest change in the tip-angle target in degrees/s
const float maximumFlexionTargetSpeedDegPerSec = 35.0f;

// Keep each motor from speeding up or slowing down too suddenly
const float linearAccelerationMMPerSec2 = 6.0f;
const float linearDecelerationMMPerSec2 = 10.0f;
const float rotationAccelerationDegPerSec2 = 90.0f;
const float rotationDecelerationDegPerSec2 = 140.0f;
const float flexionAccelerationDegPerSec2 = 120.0f;
const float flexionDecelerationDegPerSec2 = 180.0f;

// Count tiny speeds as stopped
const float stoppedLinearSpeedToleranceMMPerSec = 0.005f;
const float stoppedRotationSpeedToleranceDegPerSec = 0.05f;
const float stoppedFlexionSpeedToleranceDegPerSec = 0.05f;

// Moving toward a target position

// Use the distance from the target to choose a speed
// The speed limits still apply, and speed changes stay gradual
const float linearPositionControllerGain = 8.0f;
const float rotationPositionControllerGain = 5.0f;

// Stop when the measured position is close enough to the target
const float linearPositionTargetToleranceMM = 0.01f;
const float rotationPositionTargetToleranceDeg = 0.25f;

// When idle, snap targets close to home or a limit to that exact value
const float absoluteTargetSnapToleranceMM = 0.02f;
const float absoluteTargetSnapToleranceDeg = 0.25f;

// Check for a motor that should be moving but has no encoder movement
const float linearStallMinimumSpeedMMPerSec = 0.15f;
const float rotationStallMinimumSpeedDegPerSec = 3.0f;
const unsigned long encoderStallTimeout_ms = 750;

// Timing uses milliseconds for updates and microseconds for step pulses
const unsigned long encoderUpdateInterval_ms = 10;
const unsigned long servoCommandInterval_ms = 40;
const unsigned long servoReadInterval_ms = 80;
const unsigned long statusPrintInterval_ms = 500;
const unsigned long startupSerialSettle_ms = 1200;
const unsigned long configurationPrintPause_ms = 200;
const unsigned long buttonSetupReleaseStable_ms = 500;
const unsigned int stepPulseHighTime_us = 8;
const unsigned int directionSetupTime_us = 10;

// Set up the USB connection for the controller

USBHost usbHost;
USBHub usbHub(usbHost);
USBHIDParser hidParser(usbHost);
JoystickController joystick(usbHost);

// Checking the joystick controls

// Give time to get ready, then record the stick movement
const unsigned long calibrationPreparation_ms = 1500;
const unsigned long calibrationCapture_ms = 3000;

// Ignore tiny stick movements near the center so drift does not move the scope
const float joystickDeadZoneFraction = 0.12f;

// Leave room for controllers that send lots of axis values
const int maximumJoystickAxes = 64;

// Only learn an axis if its reading changes by at least this much
const int32_t minimumCalibrationSpan = 20;

// Keep a small first stick movement from being counted as full speed
const int32_t minimumLiveAxisHalfRange = 100;

// Flip the sign if a stick moves the scope the wrong way
const int linearJoystickDirectionSign = +1;
const int rotationJoystickDirectionSign = -1;
const int flexionJoystickDirectionSign = +1;

// Check the stick axes at startup; set false to use the saved settings below
const bool useStartupAxisLearning = true;

// Check the buttons and triggers at startup, leaving out the Guide button
const bool useStartupButtonLearning = true;

// Use these axis numbers when startup stick setup is turned off
const int defaultLinearXAxisIndex = 0;
const int defaultFlexionXAxisIndex = 2;
const int defaultRotationYAxisIndex = 5;

// Some controllers send trigger values like joystick axes
// A trigger reading must change this much to count during setup
const int32_t minimumButtonAxisChange = 80;
const float analogButtonPressFraction = 0.55f;
const float buttonSetupJoystickCenteredFraction = 0.18f;

// Use these button masks when startup button setup is off
// Check them with the diagnostics sketch if a button does the wrong thing
const uint32_t defaultHomeButtonMask = 0x00000002;         // B
const uint32_t defaultEnableReadyButtonMask = 0x00000010;  // Left bumper
const uint32_t defaultFireButtonMask = 0x00000020;         // Right bumper
const uint32_t defaultStopButtonMask = 0x00000004;         // Bottom face button

// Save the released reading for triggers that send a range of values
// The axis number, direction, and range can change with the controller
const bool defaultStandbyUsesAxis = false;            // This controller sends LT as a button
const int defaultStandbyAxisIndex = 3;
const int defaultStandbyAxisDirection = +1;
const int32_t defaultStandbyAxisTravelEstimate = 255;
const uint32_t defaultStandbyButtonMask = 0x00000040; // LT

const bool defaultPauseUsesAxis = false;              // This controller sends RT as a button
const int defaultPauseAxisIndex = 4;
const int defaultPauseAxisDirection = +1;
const int32_t defaultPauseAxisTravelEstimate = 255;
const uint32_t defaultPauseButtonMask = 0x00000080;   // RT

// Ignore diagonal right-stick input so turning and bending stay separate
const float rightStickAllowedCrossAxisMotion = 0.18f;

// Saved home positions

// Home brings the in/out and turning motors back to their startup zeros
// It also brings the tip back to its saved straight position
const float homeLinearPositionMM = linearMinPositionMM;
const float homeRotationUreteroscopeDeg = 0.0f;
const float homeFlexionTipDeg = 0.0f;

// Count Home as finished when each position is this close
// Use the encoder count to find the in/out position
const float homeLinearToleranceMM = 0.50f;
const float homeRotationToleranceDeg = 1.0f;
const float homeFlexionToleranceDeg = 1.0f;

// During Home, reset the in/out zero if movement stops inside the home zone
const unsigned long linearAutoHomeStallTimeout_ms = 500;

struct LearnedJoystickAxis {
  const char *shortName;
  const char *calibrationInstruction;
  int directionSign;
  int axisIndex;
  int32_t minimum;
  int32_t center;
  int32_t maximum;
  int32_t raw;
  float normalized;
  bool rangeInitialized;
};

LearnedJoystickAxis linearAxis = {
  "Left X",
  "Move only the LEFT joystick fully LEFT and RIGHT",
  linearJoystickDirectionSign,
  -1,
  0,
  0,
  0,
  0,
  0.0f,
  false
};

LearnedJoystickAxis rotationAxis = {
  "Right Y",
  "Move only the RIGHT joystick fully UP and DOWN",
  rotationJoystickDirectionSign,
  -1,
  0,
  0,
  0,
  0,
  0.0f,
  false
};

LearnedJoystickAxis flexionAxis = {
  "Right X",
  "Move only the RIGHT joystick fully LEFT and RIGHT",
  flexionJoystickDirectionSign,
  -1,
  0,
  0,
  0,
  0,
  0.0f,
  false
};

LearnedJoystickAxis *axisSequence[] = {
  &linearAxis,
  &flexionAxis,
  &rotationAxis
};

const int axisSequenceCount = 3;

enum JoystickState {
  WAITING_FOR_CONTROLLER,
  PREPARING_AXIS_CALIBRATION,
  CAPTURING_AXIS_CALIBRATION,
  SETTING_CONTROLLER_BUTTONS,
  JOYSTICK_READY
};

JoystickState joystickState = WAITING_FOR_CONTROLLER;
bool controllerWasConnected = false;
bool controlsReady = false;
int calibrationAxisNumber = 0;
LearnedJoystickAxis *axisBeingCalibrated = nullptr;

bool homeReturnActive = false;

struct LearnedControllerButton {
  const char *shortName;
  const char *setupInstruction;
  char command;
  uint32_t mask;
  bool usesAxis;
  int axisIndex;
  int32_t axisReleased;
  int32_t axisPressed;
  bool wasPressed;
};

LearnedControllerButton homeButton = {
  "Home [h]",
  "Press the B button, the right button in the A/B/X/Y group",
  'h',
  0,
  false,
  -1,
  0,
  0,
  false
};

LearnedControllerButton enableReadyButton = {
  "Enable/Ready [e/r]",
  "Press the left bumper/shoulder",
  'a',
  0,
  false,
  -1,
  0,
  0,
  false
};

LearnedControllerButton standbyButton = {
  "Standby [s]",
  "Press the left trigger",
  's',
  0,
  false,
  -1,
  0,
  0,
  false
};

LearnedControllerButton fireButton = {
  "Fire [f]",
  "Press the right bumper/shoulder",
  'f',
  0,
  false,
  -1,
  0,
  0,
  false
};

LearnedControllerButton pauseButton = {
  "Pause [p]",
  "Press the right trigger",
  'p',
  0,
  false,
  -1,
  0,
  0,
  false
};

LearnedControllerButton stopButton = {
  "Stop/Exit [x]",
  "Press the bottom face button currently used for Stop/Exit",
  'x',
  0,
  false,
  -1,
  0,
  0,
  false
};

LearnedControllerButton *buttonSequence[] = {
  &homeButton,
  &enableReadyButton,
  &standbyButton,
  &fireButton,
  &pauseButton,
  &stopButton
};

const int buttonSequenceCount = 6;
int buttonSetupIndex = 0;
uint32_t buttonSetupBaselineButtons = 0;
int32_t buttonSetupAxisBaseline[maximumJoystickAxes];
bool buttonSetupAxisBaselineSeen[maximumJoystickAxes];
bool buttonSetupWaitingForRelease = false;
unsigned long buttonSetupReleaseQuietStart_ms = 0;

int32_t calibrationMinimum[maximumJoystickAxes];
int32_t calibrationMaximum[maximumJoystickAxes];
bool calibrationAxisSeen[maximumJoystickAxes];
unsigned long joystickStateStartTime = 0;

// Keep track of movement

// Movement speeds requested by the sticks
float requestedLinearTargetRateMMPerSec = 0.0f;
float requestedRotationTargetRateDegPerSec = 0.0f;
float requestedFlexionTargetRateDegPerSec = 0.0f;

// The in/out stick sets speed directly, while turning follows a target angle
// The bending stick changes the target tip angle over time
float targetLinearSpeedMMPerSec = 0.0f;
float currentLinearSpeedMMPerSec = 0.0f;
float targetRotationSpeedDegPerSec = 0.0f;
float currentRotationSpeedDegPerSec = 0.0f;
float targetFlexionSpeedDegPerSec = 0.0f;
float currentFlexionSpeedDegPerSec = 0.0f;
unsigned long lastVelocityUpdate_us = 0;
unsigned long lastAbsoluteTargetUpdate_us = 0;

bool linearStepPulseIsHigh = false;
int linearActiveDirectionSign = 0;
unsigned long linearStepPulseHighStart_us = 0;
unsigned long linearLastStepRise_us = 0;
unsigned long linearDirectionChanged_us = 0;
unsigned long linearTotalStepPulses = 0;

bool rotationStepPulseIsHigh = false;
int rotationActiveDirectionSign = 0;
unsigned long rotationStepPulseHighStart_us = 0;
unsigned long rotationLastStepRise_us = 0;
unsigned long rotationDirectionChanged_us = 0;
unsigned long rotationTotalStepPulses = 0;

bool systemFaultLatched = false;

// Keep track of laser commands here; the laser does not send its status back

bool laserEnabled = false;
bool laserReady = false;
bool laserFiring = false;

// Encoder counts and motor positions

volatile long linearEncoderCount = 0;
volatile long rotationEncoderCount = 0;

long lastProcessedLinearSignedCount = 0;
long lastProcessedRotationSignedCount = 0;

float currentLinearPositionMM = 0.0f;
float currentLinearMotorRevolutions = 0.0f;
long currentLinearMotorStepEstimate = 0;

float targetLinearPositionMM = linearMinPositionMM;
float targetLinearMotorRevolutions = 0.0f;
long targetLinearMotorStepEstimate = 0;

float currentRotationMotorDeg = 0.0f;
float currentRotationUreteroscopeDeg = 0.0f;
float currentRotationMapDeg = 0.0f;

float targetRotationMotorDeg = 0.0f;
float targetRotationUreteroscopeDeg = 0.0f;
float targetRotationMapDeg = 0.0f;
long currentRotationMotorStepEstimate = 0;
long targetRotationMotorStepEstimate = 0;

unsigned long lastEncoderUpdateTime_ms = 0;
unsigned long lastLinearEncoderMovementTime_ms = 0;
unsigned long lastRotationEncoderMovementTime_ms = 0;

// Target tip angle and readings from the servo

float targetTipDeg = 0.0f;
float targetFlexionMotorDeg = 0.0f;
float targetFlexionPhysicalDeg = halfRevolutionDegrees;
int targetFlexionStep = flexionNeutralStep;
int lastCommandedFlexionStep = flexionNeutralStep;
float lastCommandedTipDeg = 0.0f;
int currentFlexionStep = -1;
float measuredFlexionPhysicalDeg = halfRevolutionDegrees;
float measuredFlexionMotorDeg = 0.0f;
float measuredTipDeg = 0.0f;
float selectedFlexionTwistFactor = flexionTwistFactor0To90;
float selectedFlexionTipGain = flexionTipGain0To90;
unsigned long lastFlexionTargetUpdate_us = 0;
unsigned long lastServoCommandTime_ms = 0;
unsigned long lastServoReadTime_ms = 0;
unsigned long lastServoWarningTime_ms = 0;
unsigned long lastServoSpeedRampTime_ms = 0;

// Gradually change the speed value sent to the ST3020
float smoothedFlexionServoSpeed = servo_speed;

// How often to print updates

unsigned long lastStatusPrintTime_ms = 0;

// List the functions used below

void setupStepperPins();
void setupEncoders();
void setupServo();
void setupUsb();
void setupLaserOutputs();
void printStartupMessage();
void printLaserCommandHelp();
void pauseAfterConfigurationPrint();
void handleSerialCommands();
void updateControllerConnection();
void resetControllerButtonAssignments();
void assignDefaultControllerButtons();
void configureDefaultControllerButton(LearnedControllerButton *button, uint32_t defaultMask);
void configureDefaultControllerTriggerAxis(LearnedControllerButton *button, int defaultAxisIndex, int defaultDirection, int32_t defaultTravelEstimate);
int32_t readCurrentAxisOrZero(int axisIndex);
void captureButtonSetupAxisBaseline();
bool findMovedButtonAxis(int *movedAxisIndex, int32_t *releasedValue, int32_t *pressedValue);
bool axisAlreadyAssignedToControllerButton(int candidateAxisIndex);
bool axisIsUsedForMotion(int candidateAxisIndex);
bool controllerButtonIsPressed(LearnedControllerButton *button, uint32_t currentButtons);
bool buttonSetupCurrentPromptAcceptsAxis();
bool buttonSetupReleaseComplete(uint32_t currentButtons);
bool buttonSetupAllLearnedControlsReleased(uint32_t currentButtons);
bool buttonSetupMotionAxesCentered();
bool buttonSetupJoystickAxisCentered(LearnedJoystickAxis *axis);
void printCurrentButtonPrompt();
void startControllerButtonSetup();
void processControllerButtonSetup();
void finishControllerButtonSetup();
void printButtonAssignments();
bool buttonMaskAlreadyAssigned(uint32_t candidateMask);
uint32_t lowestPressedButtonMask(uint32_t buttons);
void updateControllerButtonRuntime();
void runControllerCommand(char command, const char *sourceName);
bool handleLaserCommand(char command);
bool handleEnableReadyButton(const char *sourceName);
void driveLaserOutputsToReleasedState();
void lockLaserSoftwareState();
void sendLaserControlPulse(unsigned long durationMs);
void pauseLaserFiring();
void stopLaserAndRelock(bool sendReadyClearPulse);
void holdCurrentMotionTargets();
void startHomeReturn();
void updateHomeReturnTargets();
bool homeReturnIsFinished();
bool linearIsInsideHomeTolerance();
bool linearIsBeingCommandedTowardHome();
void redefineLinearHomePosition(const char *reason);
void checkLinearAutoHomeDuringHomeReturn();
void updateAbsoluteLinearAndRotationTargets();
void updateAbsoluteTargetDerivedNumbers();
void updateTargetSpeedsFromAbsolutePositionErrors();
float calculatePositionControllerSpeed(float error, float tolerance, float gain, float maximumSpeed);
float clampFloat(float value, float minimumValue, float maximumValue);
void snapAbsoluteTargetsNearLimits();
long estimateLinearMotorStepsFromMM(float positionMM);
long estimateRotationMotorStepsFromDeg(float motorDeg);
void assignDefaultJoystickAxes();
void configureJoystickAxis(LearnedJoystickAxis *axis, int defaultAxisIndex);
void startControllerCalibrationSequence();
void startNextAxisCalibration();
void beginAxisCalibrationCapture();
void clearCalibrationRanges();
void processJoystickPacket();
void recordJoystickCalibrationData();
void finishCurrentAxisCalibration();
void updateAllNormalizedJoystickValues();
void updateNormalizedJoystickValue(LearnedJoystickAxis *axis);
void updateLiveAxisRange(LearnedJoystickAxis *axis);
float applyJoystickResponseCurve(float value);
float filterHorizontalAxisOnly(float rawHorizontalValue, float rawVerticalValue, float allowedCrossAxisMotion);
void filterRightStickExclusiveAxes(float rawHorizontalValue, float rawVerticalValue, float *filteredHorizontalValue, float *filteredVerticalValue);
void updateTargetsFromJoysticks();
void updateSmoothVelocities();
float moveValueToward(float currentValue, float targetValue, float maximumChange);
float chooseRampRate(float currentValue, float targetValue, float acceleration, float deceleration);
void updateLinearStepperPulseOutput();
void updateRotationStepperPulseOutput();
bool linearCommandWouldPushPastLimit(float speedMMPerSec);
void stopLinearImmediately();
void stopRotationImmediately();
void stopAllMotion();
void updateEncoderFeedback();
void checkRuntimeEncoderStalls();
void latchSystemFault(const char *message);
void updateFlexionTargetAngle();
float selectFlexionTwistFactor(float tipDeg);
float selectFlexionTipGain(float tipDeg);
float smoothStep01(float value);
float blendFloat(float firstValue, float secondValue, float blendAmount);
float applyFlexionSoftLimitToSpeed(float requestedSpeed);
bool calculateFlexionStepFromTip(float tipDeg, int *targetStep, float *motorDeg);
int calculateFlexionServoSpeed(float tipDeg);
int updateSmoothedFlexionServoSpeed(int desiredServoSpeed);
void sendFlexionCommandIfReady();
void readFlexionPositionIfReady();
void printFlexionOverstretchWarning(float tipDeg);
void printStatusIfReady();
float normalizeRotationMapDeg(float angleDeg);
void linearEncoderISR();
void rotationEncoderISR();
long getLinearEncoderCount();
long getRotationEncoderCount();
void resetLinearEncoder();
void resetRotationEncoder();

// Setup

// Set up the hardware and starting positions
void setup() {
  Serial.begin(115200);
  delay(startupSerialSettle_ms);

  setupStepperPins();
  setupEncoders();
  setupServo();
  setupLaserOutputs();
  setupUsb();

  // Set the in/out and turning positions to zero, and use the saved tip center
  currentLinearPositionMM = linearMinPositionMM;
  targetLinearPositionMM = linearMinPositionMM;
  currentLinearMotorRevolutions = 0.0f;
  targetLinearMotorRevolutions = 0.0f;
  currentLinearMotorStepEstimate = 0;
  targetLinearMotorStepEstimate = 0;
  currentRotationMotorDeg = 0.0f;
  currentRotationUreteroscopeDeg = 0.0f;
  currentRotationMapDeg = 0.0f;
  targetRotationMotorDeg = 0.0f;
  targetRotationUreteroscopeDeg = 0.0f;
  targetRotationMapDeg = 0.0f;
  currentRotationMotorStepEstimate = 0;
  targetRotationMotorStepEstimate = 0;
  targetTipDeg = 0.0f;
  targetFlexionMotorDeg = 0.0f;
  targetFlexionPhysicalDeg = halfRevolutionDegrees;
  targetFlexionStep = flexionNeutralStep;
  lastCommandedTipDeg = 0.0f;
  lastCommandedFlexionStep = flexionNeutralStep;

  lastVelocityUpdate_us = micros();
  lastFlexionTargetUpdate_us = micros();
  lastLinearEncoderMovementTime_ms = millis();
  lastRotationEncoderMovementTime_ms = millis();
  lastServoSpeedRampTime_ms = millis();
  smoothedFlexionServoSpeed = servo_speed;
  lastAbsoluteTargetUpdate_us = micros();
  updateAbsoluteTargetDerivedNumbers();

  printStartupMessage();
}

// Turn on the stepper drivers and start both STEP pins LOW
void setupStepperPins() {
  pinMode(linear_stepPin, OUTPUT);
  pinMode(linear_dirPin, OUTPUT);
  pinMode(linear_enaPin, OUTPUT);
  pinMode(rotation_stepPin, OUTPUT);
  pinMode(rotation_dirPin, OUTPUT);
  pinMode(rotation_enaPin, OUTPUT);

  digitalWrite(linear_stepPin, LOW);
  digitalWrite(linear_dirPin, linear_dirForward);
  digitalWrite(linear_enaPin, enaPin_Enable);
  digitalWrite(rotation_stepPin, LOW);
  digitalWrite(rotation_dirPin, rotation_dirClockwise);
  digitalWrite(rotation_enaPin, enaPin_Enable);
}

// Count pulses from encoder channel A and use channel B to check the direction
void setupEncoders() {
  pinMode(linear_encA, INPUT_PULLUP);
  pinMode(linear_encB, INPUT_PULLUP);
  pinMode(rotation_encA, INPUT_PULLUP);
  pinMode(rotation_encB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(linear_encA), linearEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rotation_encA), rotationEncoderISR, RISING);

  resetLinearEncoder();
  resetRotationEncoder();
}

// Talk to the ST3020 servo through Serial1
void setupServo() {
  Serial1.begin(servo_baud);
  servo.pSerial = &Serial1;
}

// Start the USB controller connection and clear old setup readings
void setupUsb() {
  usbHost.begin();
  clearCalibrationRanges();
}

// Start the laser pins at their off levels before setting them as outputs
void setupLaserOutputs() {
  driveLaserOutputsToReleasedState();

  pinMode(relay_laser, OUTPUT);
  pinMode(fire1_laser, OUTPUT);
  pinMode(fire2_laser, OUTPUT);

  driveLaserOutputsToReleasedState();
  lockLaserSoftwareState();
}

// Print instructions for setting up the sticks and buttons
void printStartupMessage() {
  Serial.println("\n============================================================");
  Serial.println("        3 DOF CONTROLLER DIAGNOSTICS + LASER");
  Serial.println("============================================================");
  Serial.println("Before setup:");
  Serial.println("  Turn on or enable the controller.");
  Serial.println("  Keep sticks, buttons, and triggers released.");
  Serial.println();
  Serial.println("Joystick order:");
  Serial.println("  1. Left joystick left/right");
  Serial.println("  2. Right joystick left/right");
  Serial.println("  3. Right joystick up/down");
  Serial.println();
  Serial.println("Button order:");
  Serial.println("  Home, Enable/Ready, Standby, Fire, Pause, Stop/Exit");
  Serial.println();
  Serial.println("Laser relay power:");
  Serial.println("  Relay modules powered from external 4.0 V supply");
  Serial.println("  Teensy GND must share reference with external supply DC-");
  Serial.println("  Ready relay=8  Fire1=9  Fire2=10");
  Serial.println("============================================================");
  Serial.println("Waiting for controller");
  pauseAfterConfigurationPrint();
}

// Wait a little so the setup messages are easier to read
void pauseAfterConfigurationPrint() {
  Serial.flush();
  delay(configurationPrintPause_ms);
}

// Main loop

// Read the controls, update movement, check the motors, and print updates
void loop() {
  // Keep calling Task so new controller data can come in
  usbHost.Task();

  updateControllerConnection();
  handleSerialCommands();

  if (joystick && joystick.available()) {
    processJoystickPacket();
    joystick.joystickDataClear();
  }

  // Time each setup step so the USB connection can keep updating
  if (joystickState == PREPARING_AXIS_CALIBRATION &&
      millis() - joystickStateStartTime >= calibrationPreparation_ms) {
    beginAxisCalibrationCapture();
  }

  if (joystickState == CAPTURING_AXIS_CALIBRATION &&
      millis() - joystickStateStartTime >= calibrationCapture_ms) {
    finishCurrentAxisCalibration();
  }

  // Work out the targets before sending movement commands
  updateEncoderFeedback();
  updateTargetsFromJoysticks();
  updateAbsoluteLinearAndRotationTargets();
  updateTargetSpeedsFromAbsolutePositionErrors();
  updateSmoothVelocities();
  updateFlexionTargetAngle();
  updateLinearStepperPulseOutput();
  updateRotationStepperPulseOutput();
  checkRuntimeEncoderStalls();
  sendFlexionCommandIfReady();
  readFlexionPositionIfReady();
  printStatusIfReady();
}

// Connecting and setting up the controller

// Let Serial Monitor send Home and laser commands too
void handleSerialCommands() {
  while (Serial.available() > 0) {
    char command = (char)Serial.read();

    // Skip spaces and the Enter key
    if (command == '\n' || command == '\r' || command == ' ' || command == '\t') {
      continue;
    }

    if (command >= 'A' && command <= 'Z') {
      command = (char)(command + ('a' - 'A'));
    }

    runControllerCommand(command, "Serial Monitor");
  }
}

// Start setup when the controller connects; stop step commands if it disconnects
// Also turn off the laser outputs when it disconnects
void updateControllerConnection() {
  bool controllerIsConnected = (bool)joystick;

  if (controllerIsConnected && !controllerWasConnected) {
    controllerWasConnected = true;

    if (useStartupAxisLearning) {
      startControllerCalibrationSequence();
    } else {
      assignDefaultJoystickAxes();
    }
  }

  if (!controllerIsConnected && controllerWasConnected) {
    controllerWasConnected = false;
    joystickState = WAITING_FOR_CONTROLLER;
    controlsReady = false;
    homeReturnActive = false;
    resetControllerButtonAssignments();
    stopAllMotion();
    stopLaserAndRelock(true);
    Serial.println();
    Serial.println("CONTROLLER DISCONNECTED");
    Serial.println("All motion stopped, laser outputs released, and waiting for reconnection");
    Serial.println();
    pauseAfterConfigurationPrint();
  }
}

// Use the saved axes if startup stick setup is off
void assignDefaultJoystickAxes() {
  controlsReady = false;
  resetControllerButtonAssignments();
  stopAllMotion();

  configureJoystickAxis(&linearAxis, defaultLinearXAxisIndex);
  configureJoystickAxis(&flexionAxis, defaultFlexionXAxisIndex);
  configureJoystickAxis(&rotationAxis, defaultRotationYAxisIndex);

  axisBeingCalibrated = nullptr;

  Serial.println("\nController detected and responding");
  Serial.println("Default axis mapping is active");
  Serial.println("Keep sticks centered for the first moment so live centers are captured correctly");

  if (useStartupButtonLearning) {
    startControllerButtonSetup();
  } else {
    assignDefaultControllerButtons();
  }
}

// Clear the old axis readings and fill in the range as new readings come in
void configureJoystickAxis(LearnedJoystickAxis *axis, int defaultAxisIndex) {
  axis->axisIndex = defaultAxisIndex;
  axis->minimum = 0;
  axis->center = 0;
  axis->maximum = 0;
  axis->raw = 0;
  axis->normalized = 0.0f;
  axis->rangeInitialized = false;
}

// Ask for one stick movement at a time to find its USB axis number
void startControllerCalibrationSequence() {
  controlsReady = false;
  calibrationAxisNumber = 0;
  linearAxis.axisIndex = -1;
  rotationAxis.axisIndex = -1;
  flexionAxis.axisIndex = -1;
  homeReturnActive = false;
  resetControllerButtonAssignments();
  stopAllMotion();

  Serial.println();
  Serial.println("CONTROLLER CONNECTED");
  Serial.println("Keep all controls released until each prompt asks you to move or press.");
  Serial.println();
  pauseAfterConfigurationPrint();
  startNextAxisCalibration();
}

// Set up one joystick axis at a time
void startNextAxisCalibration() {
  if (calibrationAxisNumber >= axisSequenceCount) {
    axisBeingCalibrated = nullptr;
    stopAllMotion();

    Serial.println();
    Serial.println("JOYSTICK DIAGNOSTICS COMPLETE");
    Serial.println("All joystick axes were learned.");
    Serial.println();
    pauseAfterConfigurationPrint();

    if (useStartupButtonLearning) {
      startControllerButtonSetup();
    } else {
      assignDefaultControllerButtons();
    }
    return;
  }

  axisBeingCalibrated = axisSequence[calibrationAxisNumber];
  joystickState = PREPARING_AXIS_CALIBRATION;
  joystickStateStartTime = millis();
  clearCalibrationRanges();
  stopAllMotion();

  Serial.println();
  Serial.println("JOYSTICK SETUP");
  Serial.print("Axis ");
  Serial.print(calibrationAxisNumber + 1);
  Serial.print(" of ");
  Serial.print(axisSequenceCount);
  Serial.print(": ");
  Serial.println(axisBeingCalibrated->shortName);
  Serial.println(axisBeingCalibrated->calibrationInstruction);
  Serial.println("Keep every other control still.");
  Serial.println("Recording starts in a moment.");
  Serial.println();
  pauseAfterConfigurationPrint();
  joystickStateStartTime = millis();
}

// Clear the old button and trigger choices before setup
void resetControllerButtonAssignments() {
  for (int index = 0; index < buttonSequenceCount; index++) {
    buttonSequence[index]->mask = 0;
    buttonSequence[index]->usesAxis = false;
    buttonSequence[index]->axisIndex = -1;
    buttonSequence[index]->axisReleased = 0;
    buttonSequence[index]->axisPressed = 0;
    buttonSequence[index]->wasPressed = false;
  }

  buttonSetupIndex = 0;
  buttonSetupBaselineButtons = 0;
  buttonSetupWaitingForRelease = false;
  buttonSetupReleaseQuietStart_ms = 0;

  for (int axis = 0; axis < maximumJoystickAxes; axis++) {
    buttonSetupAxisBaseline[axis] = 0;
    buttonSetupAxisBaselineSeen[axis] = false;
  }
}

// Use the saved buttons if startup button setup is off
void assignDefaultControllerButtons() {
  controlsReady = false;
  homeReturnActive = false;
  resetControllerButtonAssignments();

  configureDefaultControllerButton(&homeButton, defaultHomeButtonMask);
  configureDefaultControllerButton(&enableReadyButton, defaultEnableReadyButtonMask);
  configureDefaultControllerButton(&fireButton, defaultFireButtonMask);
  configureDefaultControllerButton(&stopButton, defaultStopButtonMask);

  if (defaultStandbyUsesAxis) {
    configureDefaultControllerTriggerAxis(
      &standbyButton,
      defaultStandbyAxisIndex,
      defaultStandbyAxisDirection,
      defaultStandbyAxisTravelEstimate
    );
  } else {
    configureDefaultControllerButton(&standbyButton, defaultStandbyButtonMask);
  }

  if (defaultPauseUsesAxis) {
    configureDefaultControllerTriggerAxis(
      &pauseButton,
      defaultPauseAxisIndex,
      defaultPauseAxisDirection,
      defaultPauseAxisTravelEstimate
    );
  } else {
    configureDefaultControllerButton(&pauseButton, defaultPauseButtonMask);
  }

  joystickState = JOYSTICK_READY;
  controlsReady = true;
  stopAllMotion();
  lastStatusPrintTime_ms = millis();

  Serial.println();
  Serial.println("DEFAULT BUTTON MAPPING ASSIGNED");
  Serial.println("No controller button setup prompts will run");
  Serial.println("Assumed controls are:");
  Serial.println("  B face button -> Home command");
  Serial.println("  Left bumper / shoulder -> Enable first press, Ready second press");
  Serial.println("  Left trigger -> Standby command");
  Serial.println("  Right bumper / shoulder -> Fire command");
  Serial.println("  Right trigger -> Pause command");
  Serial.println("  Bottom face button -> Stop / Exit command");
  printButtonAssignments();
  Serial.println("3 DOF control is active");
  printLaserCommandHelp();
  pauseAfterConfigurationPrint();
}

// Save the bit USBHost_t36 uses for this button
void configureDefaultControllerButton(LearnedControllerButton *button, uint32_t defaultMask) {
  button->mask = defaultMask;
  button->usesAxis = false;
  button->axisIndex = -1;
  button->axisReleased = 0;
  button->axisPressed = 0;
  button->wasPressed = false;
}

// Save the trigger reading when released and estimate how far it can change
void configureDefaultControllerTriggerAxis(LearnedControllerButton *button, int defaultAxisIndex, int defaultDirection, int32_t defaultTravelEstimate) {
  int32_t releasedValue = readCurrentAxisOrZero(defaultAxisIndex);
  int32_t travelEstimate = labs(defaultTravelEstimate);

  if (travelEstimate < minimumButtonAxisChange) {
    travelEstimate = minimumButtonAxisChange;
  }

  button->mask = 0;
  button->usesAxis = true;
  button->axisIndex = defaultAxisIndex;
  button->axisReleased = releasedValue;
  button->axisPressed =
    releasedValue + (defaultDirection >= 0 ? travelEstimate : -travelEstimate);
  button->wasPressed = false;
}

// Return zero if that USB axis is missing
int32_t readCurrentAxisOrZero(int axisIndex) {
  if (axisIndex < 0 ||
      axisIndex >= maximumJoystickAxes ||
      (joystick.axisMask() & (uint64_t(1) << axisIndex)) == 0) {
    return 0;
  }

  return joystick.getAxis(axisIndex);
}

// Save the released readings so trigger presses can be spotted
void captureButtonSetupAxisBaseline() {
  uint64_t axisMask = joystick.axisMask();

  for (int axis = 0; axis < maximumJoystickAxes; axis++) {
    if ((axisMask & (uint64_t(1) << axis)) == 0) {
      buttonSetupAxisBaselineSeen[axis] = false;
      buttonSetupAxisBaseline[axis] = 0;
      continue;
    }

    buttonSetupAxisBaselineSeen[axis] = true;
    buttonSetupAxisBaseline[axis] = joystick.getAxis(axis);
  }
}

// Find which trigger axis changed most, leaving out the axes used to move the scope
bool findMovedButtonAxis(int *movedAxisIndex, int32_t *releasedValue, int32_t *pressedValue) {
  uint64_t axisMask = joystick.axisMask();
  int bestAxis = -1;
  int32_t bestChange = 0;
  int32_t bestReleased = 0;
  int32_t bestPressed = 0;

  for (int axis = 0; axis < maximumJoystickAxes; axis++) {
    if ((axisMask & (uint64_t(1) << axis)) == 0 || !buttonSetupAxisBaselineSeen[axis]) {
      continue;
    }

    if (axisIsUsedForMotion(axis)) {
      continue;
    }

    int32_t currentValue = joystick.getAxis(axis);
    int32_t change = labs(currentValue - buttonSetupAxisBaseline[axis]);

    if (change > bestChange) {
      bestAxis = axis;
      bestChange = change;
      bestReleased = buttonSetupAxisBaseline[axis];
      bestPressed = currentValue;
    }
  }

  if (bestAxis < 0 || bestChange < minimumButtonAxisChange) {
    return false;
  }

  *movedAxisIndex = bestAxis;
  *releasedValue = bestReleased;
  *pressedValue = bestPressed;
  return true;
}

// Do not use the same trigger axis for two commands
bool axisAlreadyAssignedToControllerButton(int candidateAxisIndex) {
  for (int index = 0; index < buttonSequenceCount; index++) {
    if (buttonSequence[index]->usesAxis &&
        buttonSequence[index]->axisIndex == candidateAxisIndex) {
      return true;
    }
  }

  return false;
}

// Keep the movement axes separate from the triggers
bool axisIsUsedForMotion(int candidateAxisIndex) {
  return candidateAxisIndex == linearAxis.axisIndex ||
         candidateAxisIndex == rotationAxis.axisIndex ||
         candidateAxisIndex == flexionAxis.axisIndex;
}

// Only Standby and Pause can use a trigger axis
bool buttonSetupCurrentPromptAcceptsAxis() {
  if (buttonSetupIndex < 0 || buttonSetupIndex >= buttonSequenceCount) {
    return false;
  }

  char command = buttonSequence[buttonSetupIndex]->command;
  return command == 's' || command == 'p';
}

bool buttonSetupReleaseComplete(uint32_t currentButtons) {
  return buttonSetupAllLearnedControlsReleased(currentButtons);
}

// Wait until the buttons and sticks are released before the next setup message
bool buttonSetupAllLearnedControlsReleased(uint32_t currentButtons) {
  if (currentButtons != 0) {
    return false;
  }

  if (!buttonSetupMotionAxesCentered()) {
    return false;
  }

  for (int index = 0; index < buttonSetupIndex; index++) {
    if (controllerButtonIsPressed(buttonSequence[index], currentButtons)) {
      return false;
    }
  }

  return true;
}

bool buttonSetupMotionAxesCentered() {
  return buttonSetupJoystickAxisCentered(&linearAxis) &&
         buttonSetupJoystickAxisCentered(&flexionAxis) &&
         buttonSetupJoystickAxisCentered(&rotationAxis);
}

// Make sure this stick axis is near center while setting up buttons
bool buttonSetupJoystickAxisCentered(LearnedJoystickAxis *axis) {
  if (!axis->rangeInitialized ||
      axis->axisIndex < 0 ||
      axis->axisIndex >= maximumJoystickAxes ||
      (joystick.axisMask() & (uint64_t(1) << axis->axisIndex)) == 0) {
    return true;
  }

  int32_t rawValue = joystick.getAxis(axis->axisIndex);
  int32_t positiveHalfSpan = labs(axis->maximum - axis->center);
  int32_t negativeHalfSpan = labs(axis->center - axis->minimum);
  int32_t halfSpan = positiveHalfSpan > negativeHalfSpan ? positiveHalfSpan : negativeHalfSpan;

  if (halfSpan < minimumLiveAxisHalfRange) {
    halfSpan = minimumLiveAxisHalfRange;
  }

  int32_t allowedCenterOffset =
    (int32_t)((float)halfSpan * buttonSetupJoystickCenteredFraction);

  if (allowedCenterOffset < minimumButtonAxisChange / 4) {
    allowedCenterOffset = minimumButtonAxisChange / 4;
  }

  return labs(rawValue - axis->center) <= allowedCenterOffset;
}

void printCurrentButtonPrompt() {
  LearnedControllerButton *button = buttonSequence[buttonSetupIndex];

  Serial.println();
  Serial.println("BUTTON SETUP");
  Serial.print("Button ");
  Serial.print(buttonSetupIndex + 1);
  Serial.print(" of ");
  Serial.print(buttonSequenceCount);
  Serial.print(": ");
  Serial.println(button->shortName);
  Serial.println(button->setupInstruction);

  if (buttonSetupCurrentPromptAcceptsAxis()) {
    Serial.println("If this is an analog trigger, press only that trigger.");
  }

  Serial.println("Press once, then release.");
  Serial.println();
  pauseAfterConfigurationPrint();
}

// Stop movement commands before setting up the buttons
void startControllerButtonSetup() {
  stopAllMotion();
  controlsReady = false;
  homeReturnActive = false;
  buttonSetupIndex = 0;
  buttonSetupBaselineButtons = joystick.getButtons();
  buttonSetupWaitingForRelease = true;
  buttonSetupReleaseQuietStart_ms = 0;
  captureButtonSetupAxisBaseline();
  joystickState = SETTING_CONTROLLER_BUTTONS;

  Serial.println();
  Serial.println("CONTROLLER BUTTON SETUP");
  Serial.println("Release every button and trigger.");
  Serial.println("The first button prompt appears after controls are quiet.");
  Serial.println();
  pauseAfterConfigurationPrint();
}

// Save one button press and wait for release before moving to the next button
void processControllerButtonSetup() {
  uint32_t currentButtons = joystick.getButtons();

  if (buttonSetupWaitingForRelease) {
    if (buttonSetupReleaseComplete(currentButtons)) {
      if (buttonSetupReleaseQuietStart_ms == 0) {
        buttonSetupReleaseQuietStart_ms = millis();
        return;
      }

      if (millis() - buttonSetupReleaseQuietStart_ms >= buttonSetupReleaseStable_ms) {
        buttonSetupWaitingForRelease = false;
        buttonSetupReleaseQuietStart_ms = 0;
        buttonSetupBaselineButtons = currentButtons;
        captureButtonSetupAxisBaseline();
        Serial.println();
        Serial.println("Released. Ready for the next assignment.");
        printCurrentButtonPrompt();
      }
    } else {
      buttonSetupReleaseQuietStart_ms = 0;
    }

    return;
  }

  bool promptAcceptsAxis = buttonSetupCurrentPromptAcceptsAxis();
  int movedAxisIndex = -1;
  int32_t movedAxisReleasedValue = 0;
  int32_t movedAxisPressedValue = 0;
  bool analogAxisMoved = false;

  if (promptAcceptsAxis) {
    analogAxisMoved =
      findMovedButtonAxis(&movedAxisIndex, &movedAxisReleasedValue, &movedAxisPressedValue);
  }

  // Update the resting readings while the buttons and triggers are released
  if (currentButtons == 0 && !analogAxisMoved) {
    buttonSetupBaselineButtons = 0;
    captureButtonSetupAxisBaseline();
    return;
  }

  uint32_t newlyPressedButtons =
    currentButtons & ~buttonSetupBaselineButtons;

  if (newlyPressedButtons == 0 && !analogAxisMoved) {
    return;
  }

  LearnedControllerButton *button = buttonSequence[buttonSetupIndex];

  if (newlyPressedButtons != 0) {
    uint32_t selectedMask = lowestPressedButtonMask(newlyPressedButtons);

    if (buttonMaskAlreadyAssigned(selectedMask)) {
      Serial.println();
      Serial.println("Already assigned. Release all controls and try this prompt again.");
      Serial.println();
      buttonSetupWaitingForRelease = true;
      buttonSetupReleaseQuietStart_ms = 0;
      return;
    }

    button->mask = selectedMask;
    button->usesAxis = false;
    button->axisIndex = -1;
    button->axisReleased = 0;
    button->axisPressed = 0;
  } else {
    if (axisIsUsedForMotion(movedAxisIndex)) {
      Serial.println();
      Serial.println("That axis is used for joystick movement. Release all controls and try again.");
      Serial.println();
      buttonSetupWaitingForRelease = true;
      buttonSetupReleaseQuietStart_ms = 0;
      return;
    }

    if (axisAlreadyAssignedToControllerButton(movedAxisIndex)) {
      Serial.println();
      Serial.println("That trigger axis is already assigned. Release all controls and try again.");
      Serial.println();
      buttonSetupWaitingForRelease = true;
      buttonSetupReleaseQuietStart_ms = 0;
      return;
    }

    button->mask = 0;
    button->usesAxis = true;
    button->axisIndex = movedAxisIndex;
    button->axisReleased = movedAxisReleasedValue;
    button->axisPressed = movedAxisPressedValue;
  }

  button->wasPressed = true;
  stopAllMotion();

  Serial.println();
  Serial.print("ASSIGNED: ");
  Serial.print(button->shortName);
  if (button->usesAxis) {
    Serial.print(" as analog axis ");
    Serial.print(button->axisIndex);
    Serial.print(" released ");
    Serial.print(button->axisReleased);
    Serial.print(" pressed ");
    Serial.println(button->axisPressed);
  } else {
    Serial.print(" assigned with mask 0x");
    Serial.println(button->mask, HEX);
  }

  buttonSetupIndex++;

  if (buttonSetupIndex >= buttonSequenceCount) {
    finishControllerButtonSetup();
    return;
  }

  buttonSetupWaitingForRelease = true;
  buttonSetupReleaseQuietStart_ms = 0;
  Serial.println("Release all controls. The next prompt will appear automatically.");
  Serial.println();
  pauseAfterConfigurationPrint();
}

// Finish setup and let the controller start moving the scope
void finishControllerButtonSetup() {
  joystickState = JOYSTICK_READY;
  controlsReady = true;
  stopAllMotion();
  lastStatusPrintTime_ms = millis();

  Serial.println();
  Serial.println("CONTROLLER BUTTON SETUP COMPLETE");
  printButtonAssignments();
  Serial.println("Controller diagnostics complete. 3 DOF control is active.");
  Serial.println("Press the B face button for Home.");
  Serial.println("Serial Monitor commands are h, e, r, f, p, s, and x.");
  printLaserCommandHelp();
  Serial.println();
  pauseAfterConfigurationPrint();
}

void printButtonAssignments() {
  Serial.println("Assigned controller controls:");
  for (int index = 0; index < buttonSequenceCount; index++) {
    Serial.print("  ");
    Serial.print(buttonSequence[index]->shortName);
    if (buttonSequence[index]->usesAxis) {
      Serial.print(" = axis ");
      Serial.print(buttonSequence[index]->axisIndex);
      Serial.print(" released ");
      Serial.print(buttonSequence[index]->axisReleased);
      Serial.print(" pressed ");
      Serial.println(buttonSequence[index]->axisPressed);
    } else {
      Serial.print(" = 0x");
      Serial.println(buttonSequence[index]->mask, HEX);
    }
  }
}

bool buttonMaskAlreadyAssigned(uint32_t candidateMask) {
  for (int index = 0; index < buttonSequenceCount; index++) {
    if (buttonSequence[index]->mask == candidateMask) {
      return true;
    }
  }

  return false;
}

// Check button bits for presses and trigger readings for how far they moved
bool controllerButtonIsPressed(LearnedControllerButton *button, uint32_t currentButtons) {
  if (!button->usesAxis) {
    return button->mask != 0 && (currentButtons & button->mask) != 0;
  }

  if (button->axisIndex < 0 ||
      button->axisIndex >= maximumJoystickAxes ||
      (joystick.axisMask() & (uint64_t(1) << button->axisIndex)) == 0) {
    return false;
  }

  int32_t currentValue = joystick.getAxis(button->axisIndex);
  int32_t fullTravel = labs(button->axisPressed - button->axisReleased);

  if (fullTravel < minimumButtonAxisChange) {
    fullTravel = minimumButtonAxisChange;
  }

  int32_t pressedDirection =
    button->axisPressed >= button->axisReleased ? +1 : -1;
  int32_t currentTravel =
    (currentValue - button->axisReleased) * pressedDirection;
  int32_t pressThreshold =
    (int32_t)((float)fullTravel * analogButtonPressFraction);

  if (pressThreshold < minimumButtonAxisChange) {
    pressThreshold = minimumButtonAxisChange;
  }

  return currentTravel >= pressThreshold;
}

// Pick one button if several are pressed during setup
uint32_t lowestPressedButtonMask(uint32_t buttons) {
  return buttons & (~buttons + 1);
}

// Start saving stick movement for this setup step
void beginAxisCalibrationCapture() {
  joystickState = CAPTURING_AXIS_CALIBRATION;
  joystickStateStartTime = millis();
  clearCalibrationRanges();

  Serial.println();
  Serial.print("RECORDING: ");
  Serial.println(axisBeingCalibrated->shortName);
  Serial.println();
  pauseAfterConfigurationPrint();
  joystickStateStartTime = millis();
}

// Clear the saved axis ranges before each setup step
void clearCalibrationRanges() {
  for (int axis = 0; axis < maximumJoystickAxes; axis++) {
    calibrationMinimum[axis] = INT32_MAX;
    calibrationMaximum[axis] = INT32_MIN;
    calibrationAxisSeen[axis] = false;
  }
}

// Find the stick axes and scale their readings

// Use controller data for setup first, then use it to move the scope
void processJoystickPacket() {
  if (joystickState == CAPTURING_AXIS_CALIBRATION) {
    recordJoystickCalibrationData();
    return;
  }

  if (joystickState == SETTING_CONTROLLER_BUTTONS) {
    processControllerButtonSetup();
    return;
  }

  if (joystickState == JOYSTICK_READY) {
    updateAllNormalizedJoystickValues();
    updateControllerButtonRuntime();
  }
}

// Save the smallest and largest readings from each axis
void recordJoystickCalibrationData() {
  uint64_t axisMask = joystick.axisMask();

  for (int axis = 0; axis < maximumJoystickAxes; axis++) {
    if ((axisMask & (uint64_t(1) << axis)) == 0) {
      continue;
    }

    int32_t value = joystick.getAxis(axis);
    calibrationAxisSeen[axis] = true;

    if (value < calibrationMinimum[axis]) {
      calibrationMinimum[axis] = value;
    }

    if (value > calibrationMaximum[axis]) {
      calibrationMaximum[axis] = value;
    }
  }
}

// Pick the axis that changed most and save its range and center
void finishCurrentAxisCalibration() {
  int bestAxis = -1;
  int32_t bestSpan = 0;

  for (int axis = 0; axis < maximumJoystickAxes; axis++) {
    if (!calibrationAxisSeen[axis]) {
      continue;
    }

    int32_t span = calibrationMaximum[axis] - calibrationMinimum[axis];

    if (span > bestSpan) {
      bestSpan = span;
      bestAxis = axis;
    }
  }

  if (bestAxis < 0 || bestSpan < minimumCalibrationSpan) {
    Serial.println();
    Serial.println("AXIS LEARNING FAILED");
    Serial.println("No axis moved far enough, so this same joystick direction will repeat.");
    Serial.println();
    pauseAfterConfigurationPrint();
    startNextAxisCalibration();
    return;
  }

  axisBeingCalibrated->axisIndex = bestAxis;
  axisBeingCalibrated->minimum = calibrationMinimum[bestAxis];
  axisBeingCalibrated->maximum = calibrationMaximum[bestAxis];
  axisBeingCalibrated->center =
    (int32_t)(((int64_t)axisBeingCalibrated->minimum + (int64_t)axisBeingCalibrated->maximum) / 2);
  axisBeingCalibrated->raw = axisBeingCalibrated->center;
  axisBeingCalibrated->normalized = 0.0f;
  axisBeingCalibrated->rangeInitialized = true;

  Serial.println();
  Serial.print("LEARNED: ");
  Serial.print(axisBeingCalibrated->shortName);
  Serial.print(" is USB axis ");
  Serial.print(bestAxis);
  Serial.print(" | range ");
  Serial.print(axisBeingCalibrated->minimum);
  Serial.print(" to ");
  Serial.print(axisBeingCalibrated->maximum);
  Serial.print(" | center ");
  Serial.println(axisBeingCalibrated->center);
  pauseAfterConfigurationPrint();

  calibrationAxisNumber++;
  startNextAxisCalibration();
}

void updateAllNormalizedJoystickValues() {
  updateNormalizedJoystickValue(&linearAxis);
  updateNormalizedJoystickValue(&rotationAxis);
  updateNormalizedJoystickValue(&flexionAxis);
}

// Scale the readings from -1 to +1, ignore center drift, and set the direction
void updateNormalizedJoystickValue(LearnedJoystickAxis *axis) {
  if (axis->axisIndex < 0) {
    axis->normalized = 0.0f;
    return;
  }

  if (axis->axisIndex >= maximumJoystickAxes ||
      (joystick.axisMask() & (uint64_t(1) << axis->axisIndex)) == 0) {
    axis->normalized = 0.0f;
    return;
  }

  axis->raw = joystick.getAxis(axis->axisIndex);
  updateLiveAxisRange(axis);

  int32_t positiveHalfSpan = axis->maximum - axis->center;
  int32_t negativeHalfSpan = axis->center - axis->minimum;

  if (positiveHalfSpan < minimumLiveAxisHalfRange) {
    positiveHalfSpan = minimumLiveAxisHalfRange;
  }

  if (negativeHalfSpan < minimumLiveAxisHalfRange) {
    negativeHalfSpan = minimumLiveAxisHalfRange;
  }

  int32_t halfSpan =
    positiveHalfSpan > negativeHalfSpan ? positiveHalfSpan : negativeHalfSpan;

  int32_t deadZone = (int32_t)((float)halfSpan * joystickDeadZoneFraction);
  int32_t delta = axis->raw - axis->center;

  if (labs(delta) <= deadZone) {
    axis->normalized = 0.0f;
    return;
  }

  if (delta > 0) {
    int32_t usableRange = positiveHalfSpan - deadZone;
    if (usableRange <= 0) {
      axis->normalized = 0.0f;
      return;
    }
    axis->normalized = (float)(delta - deadZone) / (float)usableRange;
  } else {
    int32_t usableRange = negativeHalfSpan - deadZone;
    if (usableRange <= 0) {
      axis->normalized = 0.0f;
      return;
    }
    axis->normalized = (float)(delta + deadZone) / (float)usableRange;
  }

  if (axis->normalized > 1.0f) axis->normalized = 1.0f;
  if (axis->normalized < -1.0f) axis->normalized = -1.0f;

  axis->normalized *= axis->directionSign;
}

// Update the range if the stick moves past an earlier reading
void updateLiveAxisRange(LearnedJoystickAxis *axis) {
  if (!axis->rangeInitialized) {
    axis->minimum = axis->raw;
    axis->center = axis->raw;
    axis->maximum = axis->raw;
    axis->rangeInitialized = true;
    return;
  }

  if (axis->raw < axis->minimum) {
    axis->minimum = axis->raw;
  }

  if (axis->raw > axis->maximum) {
    axis->maximum = axis->raw;
  }
}

// Turning stick readings into movement

// Make small stick movements gentle while still letting full movement reach full speed
float applyJoystickResponseCurve(float value) {
  float cubicValue = value * value * value;
  return 0.35f * value + 0.65f * cubicValue;
}

// Only allow left/right input when the up/down reading is near center
float filterHorizontalAxisOnly(float rawHorizontalValue, float rawVerticalValue, float allowedCrossAxisMotion) {
  float horizontalMagnitude = fabsf(rawHorizontalValue);
  float verticalMagnitude = fabsf(rawVerticalValue);

  if (horizontalMagnitude <= 0.0f) {
    return 0.0f;
  }

  if (verticalMagnitude <= allowedCrossAxisMotion) {
    return rawHorizontalValue;
  }

  return 0.0f;
}

// Use the right stick to bend or turn the scope, and ignore diagonal input
void filterRightStickExclusiveAxes(float rawHorizontalValue, float rawVerticalValue, float *filteredHorizontalValue, float *filteredVerticalValue) {
  float horizontalMagnitude = fabsf(rawHorizontalValue);
  float verticalMagnitude = fabsf(rawVerticalValue);

  *filteredHorizontalValue = 0.0f;
  *filteredVerticalValue = 0.0f;

  if (horizontalMagnitude <= 0.0f && verticalMagnitude <= 0.0f) {
    return;
  }

  // Mostly left/right: bend the tip
  if (horizontalMagnitude > 0.0f &&
      verticalMagnitude <= rightStickAllowedCrossAxisMotion) {
    *filteredHorizontalValue = rawHorizontalValue;
    return;
  }

  // Mostly up/down: turn the scope
  if (verticalMagnitude > 0.0f &&
      horizontalMagnitude <= rightStickAllowedCrossAxisMotion) {
    *filteredVerticalValue = rawVerticalValue;
    return;
  }

}

// Run a button command once when pressed so holding it does not repeat it
void updateControllerButtonRuntime() {
  uint32_t currentButtons = joystick.getButtons();

  for (int index = 0; index < buttonSequenceCount; index++) {
    LearnedControllerButton *button = buttonSequence[index];

    if (!button->usesAxis && button->mask == 0) {
      continue;
    }

    bool buttonIsPressed = controllerButtonIsPressed(button, currentButtons);

    if (buttonIsPressed && !button->wasPressed) {
      runControllerCommand(button->command, button->shortName);
    }

    button->wasPressed = buttonIsPressed;
  }
}

// Send USB buttons and Serial Monitor input to the same command functions
void runControllerCommand(char command, const char *sourceName) {
  if (command >= 'A' && command <= 'Z') {
    command = (char)(command + ('a' - 'A'));
  }

  if (command == 'a') {
    handleEnableReadyButton(sourceName);
    return;
  }

  Serial.println();
  Serial.print("COMMAND ISSUED: ");
  Serial.print(command);
  Serial.print(" from ");
  Serial.println(sourceName);
  Serial.println();

  if (command == 'h') {
    if (laserFiring) {
      Serial.println("HOME blocked while laser fire outputs are active. Press Pause [p] or Stop [x] first.");
      return;
    }

    startHomeReturn();
    return;
  }

  if (handleLaserCommand(command)) {
    return;
  }

  if (laserFiring) {
    Serial.println("Unknown command while firing. Emergency stop triggered.");
    stopLaserAndRelock(true);
    Serial.println("All laser outputs are OFF. Laser commands are locked.");
    return;
  }

  Serial.println("Unknown command.");
  Serial.println("Valid commands are h/e/r/f/p/s/x.");
  printLaserCommandHelp();
}

// Set every laser output to its off level
void driveLaserOutputsToReleasedState() {
  digitalWrite(fire1_laser, LASER_FIRE_RELEASED);
  digitalWrite(fire2_laser, LASER_FIRE_RELEASED);
  digitalWrite(relay_laser, LASER_RELAY_RELEASED);
  laserFiring = false;
}

// Require Enable and Ready again before allowing another Fire command
void lockLaserSoftwareState() {
  laserEnabled = false;
  laserReady = false;
  laserFiring = false;
}

// Stop new movement commands before pulsing the Ready/Standby relay
void sendLaserControlPulse(unsigned long durationMs) {
  if (!laserEnabled) {
    Serial.println("Laser pulse blocked: Enable [e] first.");
    return;
  }

  holdCurrentMotionTargets();
  stopAllMotion();

  Serial.print("Laser relay/contact pulse: ");
  Serial.print(durationMs);
  Serial.println(" ms");

  digitalWrite(relay_laser, LASER_RELAY_ACTIVE);
  delay(durationMs);
  digitalWrite(relay_laser, LASER_RELAY_RELEASED);

  Serial.println("Laser relay/contact pulse finished.");
}

// Turn off the fire outputs and keep the saved Enable and Ready states
void pauseLaserFiring() {
  digitalWrite(fire1_laser, LASER_FIRE_RELEASED);
  digitalWrite(fire2_laser, LASER_FIRE_RELEASED);
  laserFiring = false;
  holdCurrentMotionTargets();
  stopAllMotion();
}

// Turn off the fire outputs, clear Ready if needed, and reset the laser states
void stopLaserAndRelock(bool sendReadyClearPulse) {
  pauseLaserFiring();

  if (sendReadyClearPulse && laserEnabled && laserReady) {
    sendLaserControlPulse(laserControlPulse_ms);
  }

  driveLaserOutputsToReleasedState();
  lockLaserSoftwareState();
}

// Set the movement targets to the latest measured positions
void holdCurrentMotionTargets() {
  homeReturnActive = false;
  requestedLinearTargetRateMMPerSec = 0.0f;
  requestedRotationTargetRateDegPerSec = 0.0f;
  requestedFlexionTargetRateDegPerSec = 0.0f;
  targetLinearPositionMM = currentLinearPositionMM;
  targetRotationUreteroscopeDeg = currentRotationUreteroscopeDeg;

  if (currentFlexionStep >= 0) {
    targetTipDeg = measuredTipDeg;
  }

  updateAbsoluteTargetDerivedNumbers();
}

void printLaserCommandHelp() {
  Serial.println("LASER: Left bumper=Enable/Ready | Right bumper=Fire | Left trigger=Standby | Right trigger=Pause | Bottom face=Stop");
}

// The first left-bumper press sends Enable and the next one sends Ready
bool handleEnableReadyButton(const char *sourceName) {
  Serial.println();
  Serial.print("COMMAND ISSUED: Enable / Ready from ");
  Serial.println(sourceName);
  Serial.println();

  Serial.println("---------------- ENABLE / READY BUTTON ----------------");
  Serial.print("Source: ");
  Serial.println(sourceName);
  Serial.print("Before: ");
  Serial.print(laserEnabled ? "ENABLED" : "LOCKED");
  Serial.print(" | ");
  Serial.print(laserReady ? "READY" : "NOT READY");
  Serial.print(" | ");
  Serial.println(laserFiring ? "FIRING" : "NOT FIRING");

  if (laserFiring) {
    Serial.println("Executing: EMERGENCY STOP");
    Serial.println("Reason: Enable/Ready is not allowed while fire outputs are ON.");
    stopLaserAndRelock(true);
    Serial.println("All laser outputs are OFF. Laser commands are locked.");
    return true;
  }

  if (!laserEnabled) {
    Serial.println("Executing: ENABLE [e]");
    Serial.println("Why: laser commands were locked, so this left bumper press unlocks them.");
    return handleLaserCommand('e');
  }

  if (!laserReady) {
    Serial.println("Executing: READY [r]");
    Serial.println("Why: laser is enabled but not ready, so this left bumper press sends the ready pulse.");
    return handleLaserCommand('r');
  }

  Serial.println("Executing: nothing");
  Serial.println("Why: laser is already enabled and ready.");
  Serial.println("Next press Fire [f], Standby [s], Pause [p], or Stop [x].");
  return true;
}

// Only allow Fire after the code has gone through Enable and Ready
bool handleLaserCommand(char command) {
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
    Serial.println("Laser commands are unlocked. Next press Ready [r] before Fire [f].");
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
      Serial.println("Ready blocked: Enable [e] first.");
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
      Serial.println("Fire blocked: Enable [e], then Ready [r], then Fire [f].");
      printLaserCommandHelp();
      return true;
    }

    if (laserFiring) {
      Serial.println("Fire [f] ignored: laser is already firing.");
      return true;
    }

    holdCurrentMotionTargets();
    stopAllMotion();

    Serial.println("Laser command: Fire [f]");
    Serial.println("WARNING: fire outputs are turning ON. Be careful for unintended firing.");
    digitalWrite(fire1_laser, LASER_FIRE_ACTIVE);
    digitalWrite(fire2_laser, LASER_FIRE_ACTIVE);
    laserFiring = true;
    Serial.println("Fire outputs are ON. Press Pause [p], Standby [s], or Stop [x] to turn them OFF.");
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
      Serial.println("Standby blocked: Enable [e] first.");
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
    Serial.println("Standby finished. Ready has been cleared; press Ready [r] before Fire [f].");
    return true;
  }

  if (command == 'x') {
    Serial.println("Laser command: Stop/Exit [x]");
    holdCurrentMotionTargets();
    stopAllMotion();
    stopLaserAndRelock(true);
    Serial.println("All laser outputs are OFF. Laser commands are locked.");
    return true;
  }

  return false;
}

// Bring all three movements back to their saved home positions
void startHomeReturn() {
  homeReturnActive = true;
  stopAllMotion();

  targetLinearPositionMM = homeLinearPositionMM;
  targetRotationUreteroscopeDeg = homeRotationUreteroscopeDeg;
  targetTipDeg = homeFlexionTipDeg;
  updateAbsoluteTargetDerivedNumbers();

  Serial.println("\nHOME button pressed");
  Serial.println("Absolute targets set to Linear 0 mm Rotation 0 deg Flexion 0 deg");
  Serial.println("Returning linear rotation and flexion to startup positions");
}

// Keep the home targets the same until all three positions are close enough
void updateHomeReturnTargets() {
  // Ignore the sticks while going home
  targetLinearPositionMM = homeLinearPositionMM;
  targetRotationUreteroscopeDeg = homeRotationUreteroscopeDeg;
  targetTipDeg = homeFlexionTipDeg;
  targetFlexionSpeedDegPerSec = 0.0f;
  requestedLinearTargetRateMMPerSec = 0.0f;
  requestedRotationTargetRateDegPerSec = 0.0f;
  requestedFlexionTargetRateDegPerSec = 0.0f;
  updateAbsoluteTargetDerivedNumbers();

  // If the carriage is already in the home zone, reset zero without moving farther
  if (linearIsInsideHomeTolerance() &&
      fabsf(currentLinearPositionMM - homeLinearPositionMM) > linearPositionTargetToleranceMM) {
    redefineLinearHomePosition("Linear is inside HOME tolerance during HOME return");
  }

  if (homeReturnIsFinished()) {
    homeReturnActive = false;
    stopAllMotion();

    currentLinearPositionMM = homeLinearPositionMM;
    targetLinearPositionMM = homeLinearPositionMM;
    currentLinearMotorRevolutions = 0.0f;
    targetLinearMotorRevolutions = 0.0f;
    currentLinearMotorStepEstimate = 0;
    targetLinearMotorStepEstimate = 0;
    currentRotationMotorDeg = 0.0f;
    currentRotationUreteroscopeDeg = 0.0f;
    currentRotationMapDeg = homeRotationUreteroscopeDeg;
    targetRotationMotorDeg = 0.0f;
    targetRotationUreteroscopeDeg = homeRotationUreteroscopeDeg;
    targetRotationMapDeg = homeRotationUreteroscopeDeg;
    currentRotationMotorStepEstimate = 0;
    targetRotationMotorStepEstimate = 0;
    targetTipDeg = homeFlexionTipDeg;
    targetFlexionMotorDeg = 0.0f;
    targetFlexionPhysicalDeg = halfRevolutionDegrees;
    targetFlexionStep = flexionNeutralStep;
    resetLinearEncoder();
    resetRotationEncoder();

    Serial.println("HOME complete");
    Serial.println("Absolute position counters reset to the startup home state");
  }
}

bool homeReturnIsFinished() {
  float flexionHomeErrorDeg =
    currentFlexionStep >= 0
      ? measuredTipDeg - homeFlexionTipDeg
      : targetTipDeg - homeFlexionTipDeg;

  return linearIsInsideHomeTolerance() &&
         fabsf(currentRotationUreteroscopeDeg - homeRotationUreteroscopeDeg) <= homeRotationToleranceDeg &&
         fabsf(flexionHomeErrorDeg) <= homeFlexionToleranceDeg;
}

// Use the encoder position to check if the carriage is home
bool linearIsInsideHomeTolerance() {

  return fabsf(currentLinearPositionMM - homeLinearPositionMM) <= homeLinearToleranceMM;
}

// Check if Home is trying to move the carriage toward zero
bool linearIsBeingCommandedTowardHome() {
  bool targetIsHome =
    fabsf(targetLinearPositionMM - homeLinearPositionMM) <= homeLinearToleranceMM;

  bool movingTowardLowerLimit =
    targetLinearSpeedMMPerSec < -stoppedLinearSpeedToleranceMMPerSec ||
    currentLinearSpeedMMPerSec < -stoppedLinearSpeedToleranceMMPerSec;

  return homeReturnActive && targetIsHome && movingTowardLowerLimit;
}

// Reset the in/out zero when the carriage is already in the home zone
void redefineLinearHomePosition(const char *reason) {
  stopLinearImmediately();

  currentLinearPositionMM = homeLinearPositionMM;
  targetLinearPositionMM = homeLinearPositionMM;
  currentLinearMotorRevolutions = 0.0f;
  targetLinearMotorRevolutions = 0.0f;
  currentLinearMotorStepEstimate = 0;
  targetLinearMotorStepEstimate = 0;
  requestedLinearTargetRateMMPerSec = 0.0f;
  resetLinearEncoder();
  lastLinearEncoderMovementTime_ms = millis();
  updateAbsoluteTargetDerivedNumbers();

  Serial.print("\nLinear home reference reset: ");
  Serial.println(reason);
  Serial.println("Current linear position is now defined as 0.00 mm");
}

// During Home, reset the in/out zero if the encoder stops changing in the home zone
void checkLinearAutoHomeDuringHomeReturn() {
  if (!linearIsBeingCommandedTowardHome()) {
    return;
  }

  if (!linearIsInsideHomeTolerance()) {
    return;
  }

  if (millis() - lastLinearEncoderMovementTime_ms < linearAutoHomeStallTimeout_ms) {
    return;
  }

  redefineLinearHomePosition("Linear stalled or stopped inside HOME tolerance");
}

// Turn stick readings into speeds, and block them during a fault or laser firing
void updateTargetsFromJoysticks() {
  if (!controlsReady || systemFaultLatched || laserFiring) {
    requestedLinearTargetRateMMPerSec = 0.0f;
    requestedRotationTargetRateDegPerSec = 0.0f;
    requestedFlexionTargetRateDegPerSec = 0.0f;
    targetLinearSpeedMMPerSec = 0.0f;
    targetRotationSpeedDegPerSec = 0.0f;
    targetFlexionSpeedDegPerSec = 0.0f;
    return;
  }

  if (homeReturnActive) {
    updateHomeReturnTargets();
    return;
  }

  float linearRequest =
    applyJoystickResponseCurve(linearAxis.normalized) * maximumLinearSpeedMMPerSec;

  // The in/out stick normally sets motor speed directly
  if ((currentLinearPositionMM <= linearMinPositionMM + linearRangeToleranceMM && linearRequest < 0.0f) ||
      (currentLinearPositionMM >= linearMaxPositionMM - linearRangeToleranceMM && linearRequest > 0.0f) ||
      linearCommandWouldPushPastLimit(linearRequest)) {
    linearRequest = 0.0f;
    stopLinearImmediately();
  }

  requestedLinearTargetRateMMPerSec = linearRequest;
  targetLinearSpeedMMPerSec = linearRequest;

  float filteredRightStickHorizontal = 0.0f;
  float filteredRightStickVertical = 0.0f;

  filterRightStickExclusiveAxes(
    flexionAxis.normalized,
    rotationAxis.normalized,
    &filteredRightStickHorizontal,
    &filteredRightStickVertical
  );

  float rotationRequest =
    applyJoystickResponseCurve(filteredRightStickVertical) * maximumRotationSpeedDegPerSec;

  // Stop turning commands that would move past either measured angle limit
  if (currentRotationUreteroscopeDeg <= rotationMinPositionDeg + rotationRangeToleranceDeg &&
      rotationRequest < 0.0f) {
    rotationRequest = 0.0f;
    targetRotationUreteroscopeDeg = clampFloat(
      currentRotationUreteroscopeDeg,
      rotationMinPositionDeg,
      rotationMaxPositionDeg
    );
  }

  if (currentRotationUreteroscopeDeg >= rotationMaxPositionDeg - rotationRangeToleranceDeg &&
      rotationRequest > 0.0f) {
    rotationRequest = 0.0f;
    targetRotationUreteroscopeDeg = clampFloat(
      currentRotationUreteroscopeDeg,
      rotationMinPositionDeg,
      rotationMaxPositionDeg
    );
  }

  requestedRotationTargetRateDegPerSec = rotationRequest;

  float flexionRequest =
    applyJoystickResponseCurve(filteredRightStickHorizontal) * maximumFlexionTargetSpeedDegPerSec;

  if (targetTipDeg <= flexionMinTipDeg + flexionRangeToleranceTipDeg &&
      flexionRequest < 0.0f) {
    flexionRequest = 0.0f;
  }

  if (targetTipDeg >= flexionMaxTipDeg - flexionRangeToleranceTipDeg &&
      flexionRequest > 0.0f) {
    flexionRequest = 0.0f;
  }

  requestedFlexionTargetRateDegPerSec = applyFlexionSoftLimitToSpeed(flexionRequest);
  targetFlexionSpeedDegPerSec = requestedFlexionTargetRateDegPerSec;
}

// Target positions

// Change the target angle using stick speed and the time since the last update
// For normal in/out movement, keep the target at the measured position
void updateAbsoluteLinearAndRotationTargets() {
  unsigned long now_us = micros();
  unsigned long elapsed_us = now_us - lastAbsoluteTargetUpdate_us;
  lastAbsoluteTargetUpdate_us = now_us;

  // Limit the time step so a slow update does not cause a big jump
  if (elapsed_us > 50000) {
    elapsed_us = 50000;
  }

  float elapsedSeconds = (float)elapsed_us / 1000000.0f;

  if (!controlsReady || systemFaultLatched || laserFiring) {
    return;
  }

  // Keep the home targets the same while going home
  if (!homeReturnActive) {
    // Keep the printed in/out target matched to the measured position
    targetLinearPositionMM = currentLinearPositionMM;

    // Keep track of all the turning so Home can return to the starting angle
    targetRotationUreteroscopeDeg += requestedRotationTargetRateDegPerSec * elapsedSeconds;

    // Keep the target angle inside the VR rotation limits
    targetRotationUreteroscopeDeg = clampFloat(
      targetRotationUreteroscopeDeg,
      rotationMinPositionDeg,
      rotationMaxPositionDeg
    );
  }

  snapAbsoluteTargetsNearLimits();
  updateAbsoluteTargetDerivedNumbers();
}

// Work out motor steps and angles from the targets for commands and updates
void updateAbsoluteTargetDerivedNumbers() {

  targetLinearMotorRevolutions = targetLinearPositionMM / linearMMPerRev;
  targetLinearMotorStepEstimate = estimateLinearMotorStepsFromMM(targetLinearPositionMM);

  targetRotationMotorDeg = targetRotationUreteroscopeDeg * rotationGearRatio;
  targetRotationMapDeg = normalizeRotationMapDeg(targetRotationUreteroscopeDeg);
  targetRotationMotorStepEstimate = estimateRotationMotorStepsFromDeg(targetRotationMotorDeg);

  int calculatedStep = flexionNeutralStep;
  float calculatedMotorDeg = 0.0f;

  if (calculateFlexionStepFromTip(targetTipDeg, &calculatedStep, &calculatedMotorDeg)) {
    targetFlexionMotorDeg = calculatedMotorDeg;
    targetFlexionPhysicalDeg = targetFlexionMotorDeg + halfRevolutionDegrees;
    targetFlexionStep = calculatedStep;
  }
}

// Turning follows a target position; in/out movement only does this during Home
void updateTargetSpeedsFromAbsolutePositionErrors() {
  if (!controlsReady || systemFaultLatched || laserFiring) {
    targetLinearSpeedMMPerSec = 0.0f;
    targetRotationSpeedDegPerSec = 0.0f;
    return;
  }

  if (homeReturnActive) {
    targetLinearPositionMM = clampFloat(
      targetLinearPositionMM,
      linearMinPositionMM,
      linearMaxPositionMM
    );

    if (linearIsInsideHomeTolerance() &&
        targetLinearPositionMM <= homeLinearPositionMM + homeLinearToleranceMM &&
        fabsf(currentLinearPositionMM - homeLinearPositionMM) > linearPositionTargetToleranceMM) {
      redefineLinearHomePosition("Linear current and target are inside HOME tolerance");
    }

    float linearErrorMM = targetLinearPositionMM - currentLinearPositionMM;
    targetLinearSpeedMMPerSec = calculatePositionControllerSpeed(
      linearErrorMM,
      linearPositionTargetToleranceMM,
      linearPositionControllerGain,
      maximumLinearSpeedMMPerSec
    );
  }

  if (linearCommandWouldPushPastLimit(targetLinearSpeedMMPerSec)) {
    targetLinearSpeedMMPerSec = 0.0f;
    stopLinearImmediately();
  }

  // Use the total turning angle to find how far the scope is from its target
  float rotationErrorDeg = targetRotationUreteroscopeDeg - currentRotationUreteroscopeDeg;
  targetRotationSpeedDegPerSec = calculatePositionControllerSpeed(
    rotationErrorDeg,
    rotationPositionTargetToleranceDeg,
    rotationPositionControllerGain,
    maximumRotationSpeedDegPerSec
  );
}

// Use the distance from the target to choose speed, and stop when close enough
float calculatePositionControllerSpeed(float error, float tolerance, float gain, float maximumSpeed) {
  if (fabsf(error) <= tolerance) {
    return 0.0f;
  }

  float speed = error * gain;

  if (speed > maximumSpeed) {
    speed = maximumSpeed;
  }

  if (speed < -maximumSpeed) {
    speed = -maximumSpeed;
  }

  return speed;
}

float clampFloat(float value, float minimumValue, float maximumValue) {
  if (value < minimumValue) {
    return minimumValue;
  }

  if (value > maximumValue) {
    return maximumValue;
  }

  return value;
}

// Snap tiny leftover targets to the exact home position or limit
void snapAbsoluteTargetsNearLimits() {
  bool linearTargetIsBeingMoved =
    fabsf(requestedLinearTargetRateMMPerSec) >= stoppedLinearSpeedToleranceMMPerSec;

  bool rotationTargetIsBeingMoved =
    fabsf(requestedRotationTargetRateDegPerSec) >= stoppedRotationSpeedToleranceDegPerSec;

  bool flexionTargetIsBeingMoved =
    fabsf(requestedFlexionTargetRateDegPerSec) >= stoppedFlexionSpeedToleranceDegPerSec;

  // Only snap targets while idle so small stick movements can still move away from zero
  if (homeReturnActive || !linearTargetIsBeingMoved) {
    if (fabsf(targetLinearPositionMM - linearMinPositionMM) <= absoluteTargetSnapToleranceMM) {
      targetLinearPositionMM = linearMinPositionMM;
    }

    if (fabsf(targetLinearPositionMM - linearMaxPositionMM) <= absoluteTargetSnapToleranceMM) {
      targetLinearPositionMM = linearMaxPositionMM;
    }
  }

  if (homeReturnActive || !rotationTargetIsBeingMoved) {
    if (fabsf(targetRotationUreteroscopeDeg - homeRotationUreteroscopeDeg) <= absoluteTargetSnapToleranceDeg) {
      targetRotationUreteroscopeDeg = homeRotationUreteroscopeDeg;
    }
  }

  if (homeReturnActive || !flexionTargetIsBeingMoved) {
    if (fabsf(targetTipDeg - homeFlexionTipDeg) <= flexionRangeToleranceTipDeg) {
      targetTipDeg = homeFlexionTipDeg;
    }
  }
}

// Estimate motor steps from the carriage movement in mm
long estimateLinearMotorStepsFromMM(float positionMM) {
  float motorRevolutions = positionMM / linearMMPerRev;
  return lroundf(motorRevolutions * linearPulsesPerRev);
}

// Estimate motor steps from the motor angle in degrees
long estimateRotationMotorStepsFromDeg(float motorDeg) {
  return lroundf((motorDeg / degreesPerRevolution) * rotationPulsesPerRev);
}

// Gradual speed changes

// Change speed gradually so starting, stopping, and reversing feel smoother
void updateSmoothVelocities() {
  unsigned long now_us = micros();
  unsigned long elapsed_us = now_us - lastVelocityUpdate_us;
  lastVelocityUpdate_us = now_us;

  if (elapsed_us > 50000) {
    elapsed_us = 50000;
  }

  float elapsedSeconds = (float)elapsed_us / 1000000.0f;

  currentLinearSpeedMMPerSec = moveValueToward(
    currentLinearSpeedMMPerSec,
    targetLinearSpeedMMPerSec,
    chooseRampRate(
      currentLinearSpeedMMPerSec,
      targetLinearSpeedMMPerSec,
      linearAccelerationMMPerSec2,
      linearDecelerationMMPerSec2
    ) * elapsedSeconds
  );

  currentRotationSpeedDegPerSec = moveValueToward(
    currentRotationSpeedDegPerSec,
    targetRotationSpeedDegPerSec,
    chooseRampRate(
      currentRotationSpeedDegPerSec,
      targetRotationSpeedDegPerSec,
      rotationAccelerationDegPerSec2,
      rotationDecelerationDegPerSec2
    ) * elapsedSeconds
  );

  currentFlexionSpeedDegPerSec = moveValueToward(
    currentFlexionSpeedDegPerSec,
    targetFlexionSpeedDegPerSec,
    chooseRampRate(
      currentFlexionSpeedDegPerSec,
      targetFlexionSpeedDegPerSec,
      flexionAccelerationDegPerSec2,
      flexionDecelerationDegPerSec2
    ) * elapsedSeconds
  );

  if (fabsf(currentLinearSpeedMMPerSec) < stoppedLinearSpeedToleranceMMPerSec &&
      fabsf(targetLinearSpeedMMPerSec) < stoppedLinearSpeedToleranceMMPerSec) {
    currentLinearSpeedMMPerSec = 0.0f;
  }

  if (fabsf(currentRotationSpeedDegPerSec) < stoppedRotationSpeedToleranceDegPerSec &&
      fabsf(targetRotationSpeedDegPerSec) < stoppedRotationSpeedToleranceDegPerSec) {
    currentRotationSpeedDegPerSec = 0.0f;
  }

  if (fabsf(currentFlexionSpeedDegPerSec) < stoppedFlexionSpeedToleranceDegPerSec &&
      fabsf(targetFlexionSpeedDegPerSec) < stoppedFlexionSpeedToleranceDegPerSec) {
    currentFlexionSpeedDegPerSec = 0.0f;
  }
}

// Move closer to the target without going past it
float moveValueToward(float currentValue, float targetValue, float maximumChange) {
  if (currentValue < targetValue) {
    currentValue += maximumChange;
    if (currentValue > targetValue) {
      currentValue = targetValue;
    }
  } else if (currentValue > targetValue) {
    currentValue -= maximumChange;
    if (currentValue < targetValue) {
      currentValue = targetValue;
    }
  }

  return currentValue;
}

// Use the slowing-down rate when stopping or changing direction
float chooseRampRate(float currentValue, float targetValue, float acceleration, float deceleration) {
  bool sameDirection =
    (targetValue >= 0.0f && currentValue >= 0.0f) ||
    (targetValue <= 0.0f && currentValue <= 0.0f);

  bool increasingMagnitude = fabsf(targetValue) > fabsf(currentValue);

  return sameDirection && increasingMagnitude ? acceleration : deceleration;
}

// Send stepper pulses while the rest of the code keeps running

// Send the in/out STEP pulses without pausing the loop
void updateLinearStepperPulseOutput() {
  unsigned long now_us = micros();

  // Set the STEP pin LOW once it has been HIGH long enough
  if (linearStepPulseIsHigh && now_us - linearStepPulseHighStart_us >= stepPulseHighTime_us) {
    digitalWrite(linear_stepPin, LOW);
    linearStepPulseIsHigh = false;
  }

  if (systemFaultLatched ||
      fabsf(currentLinearSpeedMMPerSec) < stoppedLinearSpeedToleranceMMPerSec) {
    return;
  }

  if (linearCommandWouldPushPastLimit(currentLinearSpeedMMPerSec)) {
    stopLinearImmediately();
    return;
  }

  int requestedDirectionSign = currentLinearSpeedMMPerSec > 0.0f ? +1 : -1;

  // Keep STEP LOW while changing direction, then wait before the next pulse
  if (requestedDirectionSign != linearActiveDirectionSign && !linearStepPulseIsHigh) {
    digitalWrite(
      linear_dirPin,
      requestedDirectionSign > 0 ? linear_dirForward : linear_dirBackward
    );
    linearActiveDirectionSign = requestedDirectionSign;
    linearDirectionChanged_us = now_us;
    linearLastStepRise_us = now_us;
    return;
  }

  if (now_us - linearDirectionChanged_us < directionSetupTime_us) {
    return;
  }

  // Work out how many pulses per second give the requested carriage speed
  float pulsesPerSecond =
    (fabsf(currentLinearSpeedMMPerSec) / linearMMPerRev) * linearPulsesPerRev;

  if (pulsesPerSecond < 1.0f) {
    return;
  }

  unsigned long stepInterval_us =
    (unsigned long)(1000000.0f / pulsesPerSecond);

  if (!linearStepPulseIsHigh && now_us - linearLastStepRise_us >= stepInterval_us) {
    digitalWrite(linear_stepPin, HIGH);
    linearStepPulseIsHigh = true;
    linearStepPulseHighStart_us = now_us;
    linearLastStepRise_us = now_us;
    linearTotalStepPulses++;
  }
}

// Use the gear ratio to turn the requested scope speed into motor STEP pulses
void updateRotationStepperPulseOutput() {
  unsigned long now_us = micros();

  // Set the STEP pin LOW once it has been HIGH long enough
  if (rotationStepPulseIsHigh && now_us - rotationStepPulseHighStart_us >= stepPulseHighTime_us) {
    digitalWrite(rotation_stepPin, LOW);
    rotationStepPulseIsHigh = false;
  }

  if (systemFaultLatched ||
      fabsf(currentRotationSpeedDegPerSec) < stoppedRotationSpeedToleranceDegPerSec) {
    return;
  }

  // Stop step commands that would turn past either limit
  if (currentRotationUreteroscopeDeg <= rotationMinPositionDeg + rotationRangeToleranceDeg &&
      currentRotationSpeedDegPerSec < 0.0f) {
    targetRotationUreteroscopeDeg = clampFloat(
      currentRotationUreteroscopeDeg,
      rotationMinPositionDeg,
      rotationMaxPositionDeg
    );
    updateAbsoluteTargetDerivedNumbers();
    stopRotationImmediately();
    return;
  }

  if (currentRotationUreteroscopeDeg >= rotationMaxPositionDeg - rotationRangeToleranceDeg &&
      currentRotationSpeedDegPerSec > 0.0f) {
    targetRotationUreteroscopeDeg = clampFloat(
      currentRotationUreteroscopeDeg,
      rotationMinPositionDeg,
      rotationMaxPositionDeg
    );
    updateAbsoluteTargetDerivedNumbers();
    stopRotationImmediately();
    return;
  }

  int requestedDirectionSign = currentRotationSpeedDegPerSec > 0.0f ? +1 : -1;

  // Keep STEP LOW while changing direction, then wait before the next pulse
  if (requestedDirectionSign != rotationActiveDirectionSign && !rotationStepPulseIsHigh) {
    digitalWrite(
      rotation_dirPin,
      requestedDirectionSign > 0 ? rotation_dirClockwise : rotation_dirCounter
    );
    rotationActiveDirectionSign = requestedDirectionSign;
    rotationDirectionChanged_us = now_us;
    rotationLastStepRise_us = now_us;
    return;
  }

  if (now_us - rotationDirectionChanged_us < directionSetupTime_us) {
    return;
  }

  // Use the gear ratio to work out motor speed from scope turning speed
  float motorSpeedDegPerSec =
    fabsf(currentRotationSpeedDegPerSec) * rotationGearRatio;

  float pulsesPerSecond =
    (motorSpeedDegPerSec / degreesPerRevolution) * rotationPulsesPerRev;

  if (pulsesPerSecond < 1.0f) {
    return;
  }

  unsigned long stepInterval_us =
    (unsigned long)(1000000.0f / pulsesPerSecond);

  if (!rotationStepPulseIsHigh && now_us - rotationLastStepRise_us >= stepInterval_us) {
    digitalWrite(rotation_stepPin, HIGH);
    rotationStepPulseIsHigh = true;
    rotationStepPulseHighStart_us = now_us;
    rotationLastStepRise_us = now_us;
    rotationTotalStepPulses++;
  }
}

bool linearCommandWouldPushPastLimit(float speedMMPerSec) {
  // Stop backward movement at the measured lower limit
  if (speedMMPerSec < 0.0f &&
      currentLinearPositionMM <= linearMinPositionMM + linearRangeToleranceMM) {
    return true;
  }

  // Stop forward movement at the measured upper limit
  if (speedMMPerSec > 0.0f &&
      currentLinearPositionMM >= linearMaxPositionMM - linearRangeToleranceMM) {
    return true;
  }

  return false;
}

// Stop the in/out pulses and reset their saved speed and direction
void stopLinearImmediately() {
  targetLinearSpeedMMPerSec = 0.0f;
  currentLinearSpeedMMPerSec = 0.0f;
  digitalWrite(linear_stepPin, LOW);
  linearStepPulseIsHigh = false;
  linearActiveDirectionSign = 0;
}

// Stop the turning pulses and reset their saved speed and direction
void stopRotationImmediately() {
  targetRotationSpeedDegPerSec = 0.0f;
  currentRotationSpeedDegPerSec = 0.0f;
  digitalWrite(rotation_stepPin, LOW);
  rotationStepPulseIsHigh = false;
  rotationActiveDirectionSign = 0;
}

// Clear the stepper speeds, stick readings, and tip target speed
// A move already sent to the servo can still continue
void stopAllMotion() {
  stopLinearImmediately();
  stopRotationImmediately();
  requestedLinearTargetRateMMPerSec = 0.0f;
  requestedRotationTargetRateDegPerSec = 0.0f;
  requestedFlexionTargetRateDegPerSec = 0.0f;
  targetFlexionSpeedDegPerSec = 0.0f;
  currentFlexionSpeedDegPerSec = 0.0f;
  linearAxis.normalized = 0.0f;
  rotationAxis.normalized = 0.0f;
  flexionAxis.normalized = 0.0f;
}

// Read the motor encoders

// Use encoder counts to update how far the scope has moved and turned
void updateEncoderFeedback() {
  if (millis() - lastEncoderUpdateTime_ms < encoderUpdateInterval_ms) {
    return;
  }

  lastEncoderUpdateTime_ms = millis();

  long linearRawCount = getLinearEncoderCount();
  long linearSignedCount = linearEncoderSign * linearRawCount;
  long linearCountChange = linearSignedCount - lastProcessedLinearSignedCount;
  lastProcessedLinearSignedCount = linearSignedCount;

  if (linearCountChange != 0) {
    lastLinearEncoderMovementTime_ms = millis();

    float motorRevolutions =
      (float)linearCountChange / linearEncoderCPR;

    currentLinearPositionMM += motorRevolutions * linearMMPerRev;

    if (currentLinearPositionMM < linearMinPositionMM) {
      currentLinearPositionMM = linearMinPositionMM;
    }

    if (currentLinearPositionMM > linearMaxPositionMM) {
      currentLinearPositionMM = linearMaxPositionMM;
    }
  }

  // Work out motor values from the carriage position for printed updates
  currentLinearMotorRevolutions = currentLinearPositionMM / linearMMPerRev;
  currentLinearMotorStepEstimate = estimateLinearMotorStepsFromMM(currentLinearPositionMM);

  long rotationRawCount = getRotationEncoderCount();
  long rotationSignedCount = rotationEncoderSign * rotationRawCount;
  long rotationCountChange = rotationSignedCount - lastProcessedRotationSignedCount;
  lastProcessedRotationSignedCount = rotationSignedCount;

  if (rotationCountChange != 0) {
    lastRotationEncoderMovementTime_ms = millis();

    float motorAngleChangeDeg =
      ((float)rotationCountChange * degreesPerRevolution) / rotationEncoderCPR;

    float ureteroscopeAngleChangeDeg =
      motorAngleChangeDeg / rotationGearRatio;

    currentRotationMotorDeg += motorAngleChangeDeg;
    currentRotationUreteroscopeDeg += ureteroscopeAngleChangeDeg;
    currentRotationMapDeg = normalizeRotationMapDeg(currentRotationUreteroscopeDeg);
  }

  // Estimate motor steps from the total turning angle
  currentRotationMotorStepEstimate = estimateRotationMotorStepsFromDeg(currentRotationMotorDeg);

  if (currentRotationSpeedDegPerSec == 0.0f &&
      targetRotationSpeedDegPerSec == 0.0f &&
      fabsf(currentRotationUreteroscopeDeg) <= rotationMapZeroToleranceDeg &&
      fabsf(targetRotationUreteroscopeDeg) <= rotationMapZeroToleranceDeg) {
    currentRotationMotorDeg = 0.0f;
    currentRotationUreteroscopeDeg = 0.0f;
    currentRotationMapDeg = 0.0f;
    currentRotationMotorStepEstimate = 0;
  }
}

// Save a motor fault if the motor should be moving but its encoder count stays still
void checkRuntimeEncoderStalls() {
  if (systemFaultLatched) {
    return;
  }

  checkLinearAutoHomeDuringHomeReturn();

  if (fabsf(currentLinearSpeedMMPerSec) < linearStallMinimumSpeedMMPerSec) {
    lastLinearEncoderMovementTime_ms = millis();
  } else if (millis() - lastLinearEncoderMovementTime_ms > encoderStallTimeout_ms) {
    latchSystemFault("FAULT linear encoder movement was not detected");
    return;
  }

  if (fabsf(currentRotationSpeedDegPerSec) < rotationStallMinimumSpeedDegPerSec) {
    lastRotationEncoderMovementTime_ms = millis();
  } else if (millis() - lastRotationEncoderMovementTime_ms > encoderStallTimeout_ms) {
    latchSystemFault("FAULT rotation encoder movement was not detected");
    return;
  }
}

// A fault turns off the steppers; restart the Teensy to clear it
// This function leaves the laser outputs and any already-sent servo move unchanged
void latchSystemFault(const char *message) {
  systemFaultLatched = true;
  homeReturnActive = false;
  stopAllMotion();
  digitalWrite(linear_enaPin, enaPin_Disable);
  digitalWrite(rotation_enaPin, enaPin_Disable);
  Serial.println(message);
  Serial.println("Both stepper drivers disabled");
  Serial.println("Restart Teensy after correcting the problem");
}

// Bending the tip

// Change the tip-angle target using the smoothed speed and keep it inside the limits
void updateFlexionTargetAngle() {
  unsigned long now_us = micros();
  unsigned long elapsed_us = now_us - lastFlexionTargetUpdate_us;
  lastFlexionTargetUpdate_us = now_us;

  if (elapsed_us > 50000) {
    elapsed_us = 50000;
  }

  float elapsedSeconds = (float)elapsed_us / 1000000.0f;

  if (!homeReturnActive && !laserFiring) {
    // Change the target while the stick is held and keep it when the stick is released
    targetTipDeg += currentFlexionSpeedDegPerSec * elapsedSeconds;
  }

  if (targetTipDeg < flexionMinTipDeg) {
    targetTipDeg = flexionMinTipDeg;
  }

  if (targetTipDeg > flexionMaxTipDeg) {
    targetTipDeg = flexionMaxTipDeg;
  }

  snapAbsoluteTargetsNearLimits();
  updateAbsoluteTargetDerivedNumbers();
}

// Gradually change bending ratios near 90 and 180 degrees
float selectFlexionTwistFactor(float tipDeg) {
  float absoluteTipDeg = fabsf(tipDeg);
  float halfBlendWidth = flexionTransitionBlendWidthDeg * 0.5f;

  if (absoluteTipDeg < 90.0f - halfBlendWidth) {
    return flexionTwistFactor0To90;
  }

  if (absoluteTipDeg <= 90.0f + halfBlendWidth) {
    float blendAmount =
      smoothStep01((absoluteTipDeg - (90.0f - halfBlendWidth)) / flexionTransitionBlendWidthDeg);
    return blendFloat(flexionTwistFactor0To90, flexionTwistFactor90To180, blendAmount);
  }

  if (absoluteTipDeg < 180.0f - halfBlendWidth) {
    return flexionTwistFactor90To180;
  }

  if (absoluteTipDeg <= 180.0f + halfBlendWidth) {
    float blendAmount =
      smoothStep01((absoluteTipDeg - (180.0f - halfBlendWidth)) / flexionTransitionBlendWidthDeg);
    return blendFloat(flexionTwistFactor90To180, flexionTwistFactor180To270, blendAmount);
  }

  return flexionTwistFactor180To270;
}

// Use the ratio for this range to estimate the tip angle from the motor angle
float selectFlexionTipGain(float tipDeg) {
  float absoluteTipDeg = fabsf(tipDeg);
  float halfBlendWidth = flexionTransitionBlendWidthDeg * 0.5f;

  if (absoluteTipDeg < 90.0f - halfBlendWidth) {
    return flexionTipGain0To90;
  }

  if (absoluteTipDeg <= 90.0f + halfBlendWidth) {
    float blendAmount =
      smoothStep01((absoluteTipDeg - (90.0f - halfBlendWidth)) / flexionTransitionBlendWidthDeg);
    return blendFloat(flexionTipGain0To90, flexionTipGain90To180, blendAmount);
  }

  if (absoluteTipDeg < 180.0f - halfBlendWidth) {
    return flexionTipGain90To180;
  }

  if (absoluteTipDeg <= 180.0f + halfBlendWidth) {
    float blendAmount =
      smoothStep01((absoluteTipDeg - (180.0f - halfBlendWidth)) / flexionTransitionBlendWidthDeg);
    return blendFloat(flexionTipGain90To180, flexionTipGain180To270, blendAmount);
  }

  return flexionTipGain180To270;
}

// Smooth the change from one bending ratio to the next
float smoothStep01(float value) {
  if (value <= 0.0f) {
    return 0.0f;
  }

  if (value >= 1.0f) {
    return 1.0f;
  }

  return value * value * (3.0f - 2.0f * value);
}

float blendFloat(float firstValue, float secondValue, float blendAmount) {
  return firstValue + (secondValue - firstValue) * blendAmount;
}

// Slow the target change as the tip gets close to either angle limit
float applyFlexionSoftLimitToSpeed(float requestedSpeed) {
  if (requestedSpeed == 0.0f) {
    return 0.0f;
  }

  float distanceToLimit =
    requestedSpeed > 0.0f
      ? flexionMaxTipDeg - targetTipDeg
      : targetTipDeg - flexionMinTipDeg;

  if (distanceToLimit <= 0.0f) {
    return 0.0f;
  }

  if (distanceToLimit >= flexionSoftLimitZoneDeg) {
    return requestedSpeed;
  }

  float blendAmount = smoothStep01(distanceToLimit / flexionSoftLimitZoneDeg);
  return requestedSpeed * blendAmount;
}

// Work out the servo step for the tip angle and check both sets of limits
bool calculateFlexionStepFromTip(float tipDeg, int *targetStep, float *motorDeg) {
  selectedFlexionTwistFactor = selectFlexionTwistFactor(tipDeg);
  selectedFlexionTipGain = selectFlexionTipGain(tipDeg);

  float calculatedMotorDeg = tipDeg / selectedFlexionTipGain;

  if (tipDeg < flexionMinTipDeg || tipDeg > flexionMaxTipDeg ||
      calculatedMotorDeg < flexionMinMotorDeg || calculatedMotorDeg > flexionMaxMotorDeg) {
    return false;
  }

  float physicalMotorDeg = calculatedMotorDeg + halfRevolutionDegrees;
  int calculatedStep = lroundf((physicalMotorDeg * servoMaximumStep) / degreesPerRevolution);

  if (calculatedStep < 0) {
    calculatedStep = 0;
  }

  if (calculatedStep > servoMaximumStep) {
    calculatedStep = servoMaximumStep;
  }

  *targetStep = calculatedStep;
  *motorDeg = calculatedMotorDeg;
  return true;
}

// Slow the servo down as the tip bends farther from straight
int calculateFlexionServoSpeed(float tipDeg) {
  float absoluteTargetTipDeg = fabsf(tipDeg);

  if (absoluteTargetTipDeg <= flexionSlowdownStartTipDeg) {
    return servo_speed;
  }

  float degreesBeyondSlowdown =
    absoluteTargetTipDeg - flexionSlowdownStartTipDeg;

  int speedReduction =
    lroundf(degreesBeyondSlowdown * flexionSpeedReductionPerTipDeg);

  int calculatedSpeed = servo_speed - speedReduction;

  if (calculatedSpeed < flexionMinimumSpeed) {
    calculatedSpeed = flexionMinimumSpeed;
  }

  return calculatedSpeed;
}

// Change the speed value sent to the servo gradually
int updateSmoothedFlexionServoSpeed(int desiredServoSpeed) {
  unsigned long now_ms = millis();
  unsigned long elapsed_ms = now_ms - lastServoSpeedRampTime_ms;
  lastServoSpeedRampTime_ms = now_ms;

  // Limit the time step so a pause does not cause a big speed jump
  if (elapsed_ms > 100) {
    elapsed_ms = 100;
  }

  float elapsedSeconds = (float)elapsed_ms / 1000.0f;
  float maximumSpeedChange =
    flexionServoSpeedRampUnitsPerSec * elapsedSeconds;

  smoothedFlexionServoSpeed = moveValueToward(
    smoothedFlexionServoSpeed,
    (float)desiredServoSpeed,
    maximumSpeedChange
  );

  if (smoothedFlexionServoSpeed < flexionMinimumSpeed) {
    smoothedFlexionServoSpeed = flexionMinimumSpeed;
  }

  if (smoothedFlexionServoSpeed > servo_speed) {
    smoothedFlexionServoSpeed = servo_speed;
  }

  return lroundf(smoothedFlexionServoSpeed);
}

// Send the servo its target step, speed, and acceleration at regular intervals
void sendFlexionCommandIfReady() {
  if (!controlsReady || systemFaultLatched || laserFiring) {
    return;
  }

  if (millis() - lastServoCommandTime_ms < servoCommandInterval_ms) {
    return;
  }

  lastServoCommandTime_ms = millis();

  if (!calculateFlexionStepFromTip(targetTipDeg, &targetFlexionStep, &targetFlexionMotorDeg)) {
    targetFlexionSpeedDegPerSec = 0.0f;
    currentFlexionSpeedDegPerSec = 0.0f;
    Serial.println("Flexion target is outside the allowed ST3020 range");
    return;
  }

  targetFlexionPhysicalDeg = targetFlexionMotorDeg + halfRevolutionDegrees;

  if (abs(targetFlexionStep - lastCommandedFlexionStep) < flexionStopMarginSteps &&
      fabsf(currentFlexionSpeedDegPerSec) < stoppedFlexionSpeedToleranceDegPerSec) {
    return;
  }

  int desiredSpeed = calculateFlexionServoSpeed(targetTipDeg);
  int selectedSpeed = updateSmoothedFlexionServoSpeed(desiredSpeed);
  int commandResult =
    servo.WritePosEx(servo_ID, targetFlexionStep, selectedSpeed, servo_acceleration);

  if (commandResult != 1 && millis() - lastServoWarningTime_ms > 1000) {
    lastServoWarningTime_ms = millis();
    Serial.println("Warning ST3020 command was not acknowledged");
  }

  printFlexionOverstretchWarning(targetTipDeg);

  lastCommandedFlexionStep = targetFlexionStep;
  lastCommandedTipDeg = targetTipDeg;
}

// Read the servo position and use the bending ratios to estimate the tip angle
void readFlexionPositionIfReady() {
  if (!controlsReady) {
    return;
  }

  if (millis() - lastServoReadTime_ms < servoReadInterval_ms) {
    return;
  }

  lastServoReadTime_ms = millis();

  int readStep = servo.ReadPos(servo_ID);

  if (readStep == -1) {
    return;
  }

  currentFlexionStep = readStep;

  measuredFlexionPhysicalDeg =
    ((float)currentFlexionStep * degreesPerRevolution) / (float)servoMaximumStep;

  measuredFlexionMotorDeg =
    measuredFlexionPhysicalDeg - halfRevolutionDegrees;

  measuredTipDeg =
    measuredFlexionMotorDeg * selectFlexionTipGain(targetTipDeg);
}

// Only print a bend warning when its level changes
void printFlexionOverstretchWarning(float tipDeg) {
  static int lastWarningLevel = -1;

  float absoluteTargetTipDeg = fabsf(tipDeg);
  int warningLevel = 0;

  if (absoluteTargetTipDeg >= flexionMaxTipDeg - flexionRangeToleranceTipDeg) {
    warningLevel = 4;
  } else if (absoluteTargetTipDeg >= flexionMajorWarningTipDeg) {
    warningLevel = 3;
  } else if (absoluteTargetTipDeg >= flexionModerateWarningTipDeg) {
    warningLevel = 2;
  } else if (absoluteTargetTipDeg >= flexionSmallWarningTipDeg) {
    warningLevel = 1;
  }

  if (warningLevel == lastWarningLevel) {
    return;
  }

  lastWarningLevel = warningLevel;

  if (warningLevel == 4) {
    Serial.println("WARNING reaching maximum allowed flexion");
  } else if (warningLevel == 3) {
    Serial.println("WARNING very high flexion target");
  } else if (warningLevel == 2) {
    Serial.println("WARNING high flexion target may overstretch the ureteroscope");
  } else if (warningLevel == 1) {
    Serial.println("Notice flexion is above the first caution threshold");
  }
}

// Print updates

// Print updates regularly, but wait during setup and while a STEP pin is HIGH
void printStatusIfReady() {
  if (!controlsReady || joystickState != JOYSTICK_READY) {
    return;
  }

  if (millis() - lastStatusPrintTime_ms < statusPrintInterval_ms) {
    return;
  }

  if (linearStepPulseIsHigh || rotationStepPulseIsHigh) {
    return;
  }

  lastStatusPrintTime_ms = millis();

  Serial.print(homeReturnActive ? "HOMING | " : "");
  Serial.print("Linear ");
  Serial.print(currentLinearPositionMM, 2);
  Serial.print(" mm target ");
  Serial.print(targetLinearPositionMM, 2);
  Serial.print(" speed ");
  Serial.print(currentLinearSpeedMMPerSec, 2);
  Serial.print(" mm/s");

  Serial.print(" | Rotation ");
  Serial.print(currentRotationUreteroscopeDeg, 1);
  Serial.print(" deg target ");
  Serial.print(targetRotationUreteroscopeDeg, 1);
  Serial.print(" speed ");
  Serial.print(currentRotationSpeedDegPerSec, 1);
  Serial.print(" deg/s range ");
  Serial.print(rotationMinPositionDeg, 0);
  Serial.print(" to ");
  Serial.print(rotationMaxPositionDeg, 0);
  Serial.print(" deg");

  Serial.print(" | Flexion target ");
  Serial.print(targetTipDeg, 1);
  Serial.print(" deg measured ");
  Serial.print(measuredTipDeg, 1);
  Serial.print(" deg");

  Serial.print(" | Laser ");
  Serial.print(laserEnabled ? "ENABLED" : "LOCKED");
  Serial.print(" ");
  Serial.print(laserReady ? "READY" : "NOT READY");
  Serial.print(" ");
  Serial.print(laserFiring ? "FIRING" : "NOT FIRING");

  if (systemFaultLatched) {
    Serial.print(" | FAULT");
  }

  Serial.println();
}

// Encoder helper functions

// Wrap the displayed angle after a full turn and keep the total angle for Home
float normalizeRotationMapDeg(float angleDeg) {
  while (angleDeg >= degreesPerRevolution) {
    angleDeg -= degreesPerRevolution;
  }

  while (angleDeg <= -degreesPerRevolution) {
    angleDeg += degreesPerRevolution;
  }

  return angleDeg;
}

// Count an in/out encoder pulse and use channel B to check its direction
void linearEncoderISR() {
  if (digitalRead(linear_encB) == HIGH) {
    linearEncoderCount++;
  } else {
    linearEncoderCount--;
  }
}

// Count a turning encoder pulse and use channel B to check its direction
void rotationEncoderISR() {
  if (digitalRead(rotation_encB) == HIGH) {
    rotationEncoderCount++;
  } else {
    rotationEncoderCount--;
  }
}

// Briefly pause interrupts so the in/out count cannot change while being copied
long getLinearEncoderCount() {
  noInterrupts();
  long count = linearEncoderCount;
  interrupts();
  return count;
}

// Briefly pause interrupts so the turning count cannot change while being copied
long getRotationEncoderCount() {
  noInterrupts();
  long count = rotationEncoderCount;
  interrupts();
  return count;
}

// Reset the in/out count and the last saved count together
void resetLinearEncoder() {
  noInterrupts();
  linearEncoderCount = 0;
  interrupts();
  lastProcessedLinearSignedCount = 0;
}

// Reset the turning count and the last saved count together
void resetRotationEncoder() {
  noInterrupts();
  rotationEncoderCount = 0;
  interrupts();
  lastProcessedRotationSignedCount = 0;
}
