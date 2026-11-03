/*
GLiDAR USB Controller with Preset Controls
Description: Use a game controller to move the scope in/out, turn it, and bend the tip
             The controls are already set, so the joystick setup can be skipped
             The buttons also return the scope home and send laser commands
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

// Pins used for each part

// The NEMA 14 motor moves the scope in and out
const int linear_enaPin = 2;   // Linear driver enable
const int linear_dirPin = 3;   // Linear direction
const int linear_stepPin = 4;  // Linear step pulses
const int linear_encA = 17;    // Linear encoder A
const int linear_encB = 16;    // Linear encoder B

// The NEMA 17 motor turns the scope
const int rotation_enaPin = 5;   // Rotation driver enable
const int rotation_dirPin = 6;   // Rotation direction
const int rotation_stepPin = 7;  // Rotation step pulses
const int rotation_encA = 20;    // Rotation encoder A
const int rotation_encB = 21;    // Rotation encoder B

// Pins that send commands to the Dornier laser
const int relay_laser = 8;   // Ready/Standby relay
const int fire1_laser = 9;   // Fire output 1
const int fire2_laser = 10;  // Fire output 2

// HIGH and LOW settings for this wiring

// LOW turns the stepper drivers on
const int enaPin_Enable = LOW;

// HIGH turns the stepper drivers off
const int enaPin_Disable = HIGH;

// Which signal moves the scope forward or backward
const int linear_dirForward = HIGH;
const int linear_dirBackward = LOW;

// Which signal turns the scope in each direction
const int rotation_dirClockwise = LOW;
const int rotation_dirCounter = HIGH;

// LOW closes the Ready/Standby relay contact; HIGH opens it
const int LASER_RELAY_ACTIVE = LOW;
const int LASER_RELAY_RELEASED = HIGH;

// HIGH turns the Fire outputs on; LOW turns them off
const int LASER_FIRE_ACTIVE = HIGH;
const int LASER_FIRE_RELEASED = LOW;

// Ready and Standby both use this length of relay pulse
const unsigned long laserControlPulse_ms = 3000;

// One motor turn moves the scope 4 mm; the allowed travel is 0-177 mm

const float linearMMPerRev = 4.0f;
const float linearPulsesPerRev = 1600.0f;
const float linearEncoderCPR = 1000.0f;
const float linearMinPositionMM = 0.0f;
const float linearMaxPositionMM = 177.0f;
const float linearRangeToleranceMM = 0.01f;
const int linearEncoderSign = +1;

// Use the encoder counts and 2.5:1 gear ratio to track scope rotation

const float rotationEncoderCPR = 1000.0f;
const float rotationPulsesPerRev = 1600.0f;
const float rotationGearRatio = 2.5f;
const int rotationEncoderSign = -1;
const float rotationMapZeroToleranceDeg = 0.5f;

const float rotationMinPositionDeg = -180.0f;
const float rotationMaxPositionDeg = 180.0f;
const float rotationRangeToleranceDeg = 0.5f;

// The tip bends differently in the 0-90, 90-180, and 180-270 degree ranges
// Use the same bending factors in either direction

const float flexionTwistFactor0To90 = 6.0f;
const float flexionTwistFactor90To180 = 6.5f;
const float flexionTwistFactor180To270 = 7.2f;

// Gear ratio between the servo and the cable that bends the tip
const float flexionGearRatio = 3.5f;

// Use these factors to estimate the tip angle from the servo angle
const float flexionTipGain0To90 = flexionTwistFactor0To90 / flexionGearRatio;
const float flexionTipGain90To180 = flexionTwistFactor90To180 / flexionGearRatio;
const float flexionTipGain180To270 = flexionTwistFactor180To270 / flexionGearRatio;

// Change between the bending factors gradually over 30 degrees
// The changes happen around 90 and 180 degrees: 75-105 and 165-195
const float flexionTransitionBlendWidthDeg = 30.0f;

// Move the tip-angle target more slowly near its limits
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

// Slow the servo down when the tip is bent farther

const int flexionSlowdownStartTipDeg = 100;
const float flexionSpeedReductionPerTipDeg = 2.5f;
const int flexionMinimumSpeed = 50;

// Keep the servo speed setting from changing too suddenly
// A higher value lets the speed change more quickly
const float flexionServoSpeedRampUnitsPerSec = 300.0f;

// Tip angles that should print a warning

const int flexionSmallWarningTipDeg = 150;
const int flexionModerateWarningTipDeg = 200;
const int flexionMajorWarningTipDeg = 250;

// How far the tip and servo are allowed to turn

const float flexionMaxTipDeg = 270.0f;
const float flexionMinTipDeg = -270.0f;
const float flexionMaxMotorDeg = 180.0f;
const float flexionMinMotorDeg = -180.0f;
const int flexionStopMarginSteps = 1;
const float flexionRangeToleranceMotorDeg = degreesPerRevolution / (float)servoMaximumStep;
const float flexionRangeToleranceTipDeg =
  flexionRangeToleranceMotorDeg * flexionTipGain180To270;

// How fast each part can move and change speed

// Fastest in/out movement, in mm/s
const float maximumLinearSpeedMMPerSec = 2.0f;

// Fastest scope rotation, in degrees/s
const float maximumRotationSpeedDegPerSec = 30.0f;

// Fastest change to the tip-angle target, in degrees/s
const float maximumFlexionTargetSpeedDegPerSec = 35.0f;

// Build up speed and slow down gradually
const float linearAccelerationMMPerSec2 = 6.0f;
const float linearDecelerationMMPerSec2 = 10.0f;
const float rotationAccelerationDegPerSec2 = 90.0f;
const float rotationDecelerationDegPerSec2 = 140.0f;
const float flexionAccelerationDegPerSec2 = 120.0f;
const float flexionDecelerationDegPerSec2 = 180.0f;

// Count very small speeds as zero
const float stoppedLinearSpeedToleranceMMPerSec = 0.005f;
const float stoppedRotationSpeedToleranceDegPerSec = 0.05f;
const float stoppedFlexionSpeedToleranceDegPerSec = 0.05f;

// Settings for moving to a target position

// Use the distance left to decide how fast to move
// For example, 0.20 mm left with a gain of 8 asks for 1.6 mm/s
const float linearPositionControllerGain = 8.0f;
const float rotationPositionControllerGain = 5.0f;

// Stop once the position is close enough to the target
// Allow a little more rotation error because the gears have some play
const float linearPositionTargetToleranceMM = 0.01f;
const float rotationPositionTargetToleranceDeg = 0.25f;

// Round a stopped target to home or a limit when it is already very close
// This keeps a tiny leftover error from making HOME run forever
const float absoluteTargetSnapToleranceMM = 0.02f;
const float absoluteTargetSnapToleranceDeg = 0.25f;

// Check for a motor that is told to move but has no encoder movement
const float linearStallMinimumSpeedMMPerSec = 0.15f;
const float rotationStallMinimumSpeedDegPerSec = 3.0f;
const unsigned long encoderStallTimeout_ms = 750;

// How often to read feedback, send commands, print status, and pulse the motors
const unsigned long encoderUpdateInterval_ms = 10;
const unsigned long servoCommandInterval_ms = 40;
const unsigned long servoReadInterval_ms = 80;
const unsigned long statusPrintInterval_ms = 1000;
const unsigned long commandStatusQuietAfter_ms = 1500;
const unsigned int stepPulseHighTime_us = 8;
const unsigned int directionSetupTime_us = 10;

// Set up USB support for the controller

USBHost usbHost;
USBHub usbHub(usbHost);
USBHIDParser hidParser(usbHost);
JoystickController joystick(usbHost);

// Controller setup and button assignments

// Time allowed for learning joystick axes if that option is turned on
const unsigned long calibrationPreparation_ms = 1500;
const unsigned long calibrationCapture_ms = 3000;

// Ignore small joystick changes near the center so drift does not move the scope
const float joystickDeadZoneFraction = 0.12f;

// Leave room for up to 64 axes in the controller reports
const int maximumJoystickAxes = 64;

// An axis must change by at least this much before setup accepts it
const int32_t minimumCalibrationSpan = 20;

// Keep the first small joystick movement from being treated as full speed
const int32_t minimumLiveAxisHalfRange = 100;

// Change a sign if a joystick moves the scope the wrong way
const int linearJoystickDirectionSign = +1;
const int linearGuardJoystickDirectionSign = -1;
const int rotationJoystickDirectionSign = -1;
const int flexionJoystickDirectionSign = +1;

// Set this to true to learn the joystick axes when the program starts
const bool useStartupAxisLearning = false;

// Set this to true to choose the command buttons when the program starts
const bool useStartupButtonLearning = false;

// Saved joystick axis numbers for this controller
// Use the diagnostic sketch to check these if a stick does the wrong thing
const int defaultLinearXAxisIndex = 0;
const int defaultLinearYAxisGuardIndex = 1;
const int defaultFlexionXAxisIndex = 2;
const int defaultRotationYAxisIndex = 5;

// Ignore in/out commands if the left stick is moved too far up or down
const float leftStickAllowedCrossAxisMotion = 0.12f;

// Some triggers send a changing value instead of a simple pressed/released button
const int32_t minimumButtonAxisChange = 80;
const float analogButtonPressFraction = 0.55f;

// Saved button bits for this controller
// Use the diagnostic sketch to find these values for a different controller
const uint32_t defaultHomeButtonMask = 0x00000002;         // B
const uint32_t defaultEnableReadyButtonMask = 0x00000010;  // Left bumper
const uint32_t defaultFireButtonMask = 0x00000020;         // Right bumper
const uint32_t defaultStopButtonMask = 0x00000004;         // Bottom face button on this controller

// Settings for triggers that send a changing value
// Save the released value at startup and use the sign to tell which way a press changes it
const bool defaultStandbyUsesAxis = false;            // LT acts like a button on this controller
const int defaultStandbyAxisIndex = 3;
const int defaultStandbyAxisDirection = +1;
const int32_t defaultStandbyAxisTravelEstimate = 255;
const uint32_t defaultStandbyButtonMask = 0x00000040; // LT

const bool defaultPauseUsesAxis = false;              // RT acts like a button on this controller
const int defaultPauseAxisIndex = 4;
const int defaultPauseAxisDirection = +1;
const int32_t defaultPauseAxisTravelEstimate = 255;
const uint32_t defaultPauseButtonMask = 0x00000080;   // RT

// The right stick can turn the scope or bend the tip, one at a time
// Ignore diagonal movement if both directions go past this amount
const float rightStickAllowedCrossAxisMotion = 0.18f;

// Settings for returning to the saved home positions

// HOME brings in/out movement and rotation back to their starting positions
// The tip goes back to the straight angle set here
const float homeLinearPositionMM = linearMinPositionMM;
const float homeRotationUreteroscopeDeg = 0.0f;
const float homeFlexionTipDeg = 0.0f;

// HOME is finished when each position is this close to its home target
// Use the encoder to track the in/out position
const float homeLinearToleranceMM = 0.50f;
const float homeRotationToleranceDeg = 1.0f;
const float homeFlexionToleranceDeg = 1.0f;

// During HOME, reset to 0 mm if the encoder stops changing inside the home zone
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

LearnedJoystickAxis linearGuardAxis = {
  "Left Y Guard",
  "Move only the LEFT joystick fully UP and DOWN",
  linearGuardJoystickDirectionSign,
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
  &linearGuardAxis,
  &flexionAxis,
  &rotationAxis
};

const int axisSequenceCount = 4;

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

int32_t calibrationMinimum[maximumJoystickAxes];
int32_t calibrationMaximum[maximumJoystickAxes];
bool calibrationAxisSeen[maximumJoystickAxes];
unsigned long joystickStateStartTime = 0;

// Values used to keep track of movement

// The joysticks set in/out speed and how quickly the turn and bend targets change
float requestedLinearTargetRateMMPerSec = 0.0f;
float requestedRotationTargetRateDegPerSec = 0.0f;
float requestedFlexionTargetRateDegPerSec = 0.0f;

// Speeds asked for by the controls and the speeds after smoothing
// In/out movement follows speed directly; rotation tries to reach its target angle
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

// These flags track commands sent to the laser, not what the laser is actually doing

bool laserEnabled = false;
bool laserReady = false;
bool laserFiring = false;

// Save encoder counts and the positions calculated from them

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

// Save the tip targets and the readings sent back by the servo

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

// Change the speed sent to the ST3020 gradually
float smoothedFlexionServoSpeed = servo_speed;

// Keep track of when to print the next status message

unsigned long lastStatusPrintTime_ms = 0;
unsigned long statusPrintMutedUntil_ms = 0;

// List the functions used below

void setupStepperPins();
void setupEncoders();
void setupServo();
void setupUsb();
void setupLaserOutputs();
void printStartupMessage();
void printLaserCommandHelp();
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
void printCurrentButtonPrompt();
void startControllerButtonSetup();
void processControllerButtonSetup();
void finishControllerButtonSetup();
void printButtonAssignments();
bool buttonMaskAlreadyAssigned(uint32_t candidateMask);
uint32_t lowestPressedButtonMask(uint32_t buttons);
void updateControllerButtonRuntime();
void runControllerCommand(char command, const char *sourceName);
const char *commandDisplayName(char command);
void printCommandIssued(const char *commandName, const char *sourceName);
void beginCommandPrintBlock();
void finishCommandPrintBlock();
void muteStatusPrintsFor(unsigned long quietTimeMs);
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

// Set everything up

// Run this setup once when the Teensy starts
void setup() {
  Serial.begin(115200);

  setupStepperPins();
  setupEncoders();
  setupServo();
  setupLaserOutputs();
  setupUsb();

  // Save the starting in/out and rotation positions as home; use the straight angle for the tip
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

// Start both STEP pins LOW, then turn on the stepper drivers
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

// Count each rising edge on encoder A and read B to find the direction
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

// Use Serial1 to talk to the ST3020 servo
void setupServo() {
  Serial1.begin(servo_baud);
  servo.pSerial = &Serial1;
}

// Start reading the USB controller and clear old setup values
void setupUsb() {
  usbHost.begin();
  clearCalibrationRanges();
}

// Keep the laser signals off while changing the pins to output mode
void setupLaserOutputs() {
  driveLaserOutputsToReleasedState();

  pinMode(relay_laser, OUTPUT);
  pinMode(fire1_laser, OUTPUT);
  pinMode(fire2_laser, OUTPUT);

  driveLaserOutputsToReleasedState();
  lockLaserSoftwareState();
}

// Print the controls and wiring notes to help with testing
void printStartupMessage() {
  Serial.println("\n============================================================");
  Serial.println("              3 DOF CONTROLLER + LASER");
  Serial.println("============================================================");
  Serial.println("");
  Serial.println("MOTION");
  Serial.println("  Left joystick X  -> Linear speed");
  Serial.println("  Right joystick X -> Flexion target");
  Serial.println("  Right joystick Y -> Rotation target");
  Serial.println("  B button         -> Home");
  Serial.println("");
  Serial.println("AXIS NUMBERS");
  Serial.print("  Left X=");
  Serial.print(defaultLinearXAxisIndex);
  Serial.print("  Left Y guard=");
  Serial.print(defaultLinearYAxisGuardIndex);
  Serial.print("  Right X=");
  Serial.print(defaultFlexionXAxisIndex);
  Serial.print("  Right Y=");
  Serial.println(defaultRotationYAxisIndex);
  Serial.println("");
  Serial.println("LASER BUTTONS");
  Serial.println("  Left bumper / shoulder, first press  -> Enable command");
  Serial.println("  Left bumper / shoulder, second press -> Ready command");
  Serial.println("  Right bumper / shoulder              -> Fire command");
  Serial.println("  Left trigger                         -> Standby command");
  Serial.println("  Right trigger                        -> Pause command");
  Serial.println("  Bottom face button                   -> Stop / Exit command");
  Serial.println("");
  Serial.println("LASER PINS");
  Serial.println("  Ready relay=8  Fire1=9  Fire2=10");
  Serial.println("  Relay modules powered from external 4.0 V supply");
  Serial.println("  Teensy GND must share reference with external supply DC-");
  Serial.println("");
  Serial.println("STATUS");
  Serial.println("  Linear uses the same direct speed control as the working movement-only sketch.");
  Serial.print("  Status print interval: ");
  Serial.print(statusPrintInterval_ms);
  Serial.println(" ms");
  Serial.println("");
  printLaserCommandHelp();
  Serial.println("============================================================");
  Serial.println("Waiting for controller");
}

// Keep the program running

// Read the controls, update movement, check feedback, and print status
void loop() {
  // Keep checking USB so new controller readings can come in
  usbHost.Task();

  updateControllerConnection();
  handleSerialCommands();

  // Only use each new controller report once
  if (joystick && joystick.available()) {
    processJoystickPacket();
    joystick.joystickDataClear();
  }

  // Use timers for setup so the rest of the loop can keep running
  if (joystickState == PREPARING_AXIS_CALIBRATION &&
      millis() - joystickStateStartTime >= calibrationPreparation_ms) {
    beginAxisCalibrationCapture();
  }

  if (joystickState == CAPTURING_AXIS_CALIBRATION &&
      millis() - joystickStateStartTime >= calibrationCapture_ms) {
    finishCurrentAxisCalibration();
  }

  // Read feedback and work out the targets before telling the motors to move
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

// Connect and set up the controller

// Let the Serial Monitor send HOME and laser commands too
void handleSerialCommands() {
  while (Serial.available() > 0) {
    char command = (char)Serial.read();

    // Skip spaces and Enter characters
    if (command == '\n' || command == '\r' || command == ' ' || command == '\t') {
      continue;
    }

    // Make uppercase command letters work too
    if (command >= 'A' && command <= 'Z') {
      command = (char)(command + ('a' - 'A'));
    }

    runControllerCommand(command, "Serial Monitor");
  }
}

// Load the controls when the controller connects
// If it disconnects, stop asking for movement and turn off the laser outputs
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
  }
}

// Use the saved axis numbers to skip the joystick setup
void assignDefaultJoystickAxes() {
  controlsReady = false;
  resetControllerButtonAssignments();
  stopAllMotion();

  configureJoystickAxis(&linearAxis, defaultLinearXAxisIndex);
  configureJoystickAxis(&linearGuardAxis, defaultLinearYAxisGuardIndex);
  configureJoystickAxis(&flexionAxis, defaultFlexionXAxisIndex);
  configureJoystickAxis(&rotationAxis, defaultRotationYAxisIndex);

  axisBeingCalibrated = nullptr;

  Serial.println();
  Serial.println("CONTROLLER CONNECTED");
  Serial.println("Keep sticks centered briefly");
  Serial.println();

  if (useStartupButtonLearning) {
    startControllerButtonSetup();
  } else {
    assignDefaultControllerButtons();
  }
}

// Save the axis number and forget its old smallest and largest readings
void configureJoystickAxis(LearnedJoystickAxis *axis, int defaultAxisIndex) {
  axis->axisIndex = defaultAxisIndex;
  axis->minimum = 0;
  axis->center = 0;
  axis->maximum = 0;
  axis->raw = 0;
  axis->normalized = 0.0f;
  axis->rangeInitialized = false;
}

// Start learning the joystick axes if that option is turned on
void startControllerCalibrationSequence() {
  controlsReady = false;
  calibrationAxisNumber = 0;
  linearAxis.axisIndex = -1;
  linearGuardAxis.axisIndex = -1;
  rotationAxis.axisIndex = -1;
  flexionAxis.axisIndex = -1;
  homeReturnActive = false;
  resetControllerButtonAssignments();
  stopAllMotion();

  Serial.println();
  Serial.println("CONTROLLER CONNECTED");
  Serial.println("Axis learning will run now");
  Serial.println();
  startNextAxisCalibration();
}

// Ask for one joystick direction at a time
void startNextAxisCalibration() {
  if (calibrationAxisNumber >= axisSequenceCount) {
    axisBeingCalibrated = nullptr;
    stopAllMotion();

    Serial.println("\nAll joystick axes learned");

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

  Serial.print("\nAxis ");
  Serial.print(calibrationAxisNumber + 1);
  Serial.print(" of ");
  Serial.print(axisSequenceCount);
  Serial.print("  ");
  Serial.println(axisBeingCalibrated->shortName);
  Serial.println(axisBeingCalibrated->calibrationInstruction);
  Serial.println("Keep the other sticks and triggers still");
}

// Clear old button and trigger choices before setup or reconnecting
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

  for (int axis = 0; axis < maximumJoystickAxes; axis++) {
    buttonSetupAxisBaseline[axis] = 0;
    buttonSetupAxisBaselineSeen[axis] = false;
  }
}

// Load the saved button and trigger choices
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

  Serial.println("Default buttons/triggers assigned");
  Serial.println("Controls active");
}

// Save the bit that tells the USB report which button this is
void configureDefaultControllerButton(LearnedControllerButton *button, uint32_t defaultMask) {
  button->mask = defaultMask;
  button->usesAxis = false;
  button->axisIndex = -1;
  button->axisReleased = 0;
  button->axisPressed = 0;
  button->wasPressed = false;
}

// Set up a trigger that sends a changing axis value
// Read its released value and estimate how far it can change
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

// Use zero if this USB axis is missing
int32_t readCurrentAxisOrZero(int axisIndex) {
  if (axisIndex < 0 ||
      axisIndex >= maximumJoystickAxes ||
      (joystick.axisMask() & (uint64_t(1) << axisIndex)) == 0) {
    return 0;
  }

  return joystick.getAxis(axisIndex);
}

// Save released values so setup can tell which trigger changed
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

// Find which axis changed the most while setting up a button
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

// Do not let two commands use the same trigger axis
bool axisAlreadyAssignedToControllerButton(int candidateAxisIndex) {
  for (int index = 0; index < buttonSequenceCount; index++) {
    if (buttonSequence[index]->usesAxis &&
        buttonSequence[index]->axisIndex == candidateAxisIndex) {
      return true;
    }
  }

  return false;
}

// Do not use a movement axis for a command trigger too
bool axisIsUsedForMotion(int candidateAxisIndex) {
  return candidateAxisIndex == linearAxis.axisIndex ||
         candidateAxisIndex == linearGuardAxis.axisIndex ||
         candidateAxisIndex == rotationAxis.axisIndex ||
         candidateAxisIndex == flexionAxis.axisIndex;
}

// Print which button or trigger to press next
void printCurrentButtonPrompt() {
  LearnedControllerButton *button = buttonSequence[buttonSetupIndex];

  Serial.print("\nButton ");
  Serial.print(buttonSetupIndex + 1);
  Serial.print(" of ");
  Serial.print(buttonSequenceCount);
  Serial.print("  ");
  Serial.println(button->shortName);
  Serial.println("Release all controller buttons");
  Serial.println(button->setupInstruction);
}

// Do not allow movement or controller commands while choosing the buttons
void startControllerButtonSetup() {
  stopAllMotion();
  controlsReady = false;
  homeReturnActive = false;
  buttonSetupIndex = 0;
  buttonSetupBaselineButtons = joystick.getButtons();
  buttonSetupWaitingForRelease = false;
  captureButtonSetupAxisBaseline();
  joystickState = SETTING_CONTROLLER_BUTTONS;

  Serial.println("\n---------------- CONTROLLER BUTTON SETUP ----------------");
  Serial.println("Each listed physical button must be pressed once");
  Serial.println("Movement and laser buttons remain locked until setup is complete");
  printCurrentButtonPrompt();
}

// Save one button or trigger press, then wait until it is released
void processControllerButtonSetup() {
  uint32_t currentButtons = joystick.getButtons();
  int movedAxisIndex = -1;
  int32_t movedAxisReleasedValue = 0;
  int32_t movedAxisPressedValue = 0;
  bool analogAxisMoved =
    findMovedButtonAxis(&movedAxisIndex, &movedAxisReleasedValue, &movedAxisPressedValue);

  if (buttonSetupWaitingForRelease) {
    if (currentButtons == 0 && !analogAxisMoved) {
      buttonSetupWaitingForRelease = false;
      buttonSetupBaselineButtons = 0;
      captureButtonSetupAxisBaseline();
      printCurrentButtonPrompt();
    }

    return;
  }

  // Update the resting values when all buttons and triggers are released
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
      Serial.println("That button is already assigned");
      Serial.println("Please release it and press the requested different physical button");
      buttonSetupWaitingForRelease = true;
      return;
    }

    button->mask = selectedMask;
    button->usesAxis = false;
    button->axisIndex = -1;
    button->axisReleased = 0;
    button->axisPressed = 0;
  } else {
    if (axisIsUsedForMotion(movedAxisIndex)) {
      Serial.println("That axis is already used for joystick movement");
      Serial.println("Please release it and press only the requested trigger/button");
      buttonSetupWaitingForRelease = true;
      return;
    }

    if (axisAlreadyAssignedToControllerButton(movedAxisIndex)) {
      Serial.println("That analog trigger axis is already assigned");
      Serial.println("Please release it and press the requested different trigger/button");
      buttonSetupWaitingForRelease = true;
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

  Serial.print(button->shortName);
  if (button->usesAxis) {
    Serial.print(" assigned as analog axis ");
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
  Serial.println("Release all buttons/triggers to continue to the next assignment");
}

// Finish choosing the buttons and turn on the normal controls
void finishControllerButtonSetup() {
  joystickState = JOYSTICK_READY;
  controlsReady = true;
  stopAllMotion();

  Serial.println("\nController button setup complete");
  printButtonAssignments();
  Serial.println("Full 3 DOF standardized-axis control is active");
  Serial.println("Press B for HOME or use Serial Monitor h/e/r/f/p/s/x");
  printLaserCommandHelp();
}

// Print the button and trigger choices to help find problems
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

// Check if this button has already been used for another command
bool buttonMaskAlreadyAssigned(uint32_t candidateMask) {
  for (int index = 0; index < buttonSequenceCount; index++) {
    if (buttonSequence[index]->mask == candidateMask) {
      return true;
    }
  }

  return false;
}

// Read button bits and check how far each analog trigger has moved
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

// Choose one button if several are pressed during setup
uint32_t lowestPressedButtonMask(uint32_t buttons) {
  return buttons & (~buttons + 1);
}

// Start saving readings for this joystick direction
void beginAxisCalibrationCapture() {
  joystickState = CAPTURING_AXIS_CALIBRATION;
  joystickStateStartTime = millis();
  clearCalibrationRanges();

  Serial.print("CALIBRATING ");
  Serial.println(axisBeingCalibrated->shortName);
  Serial.println(axisBeingCalibrated->calibrationInstruction);
}

// Clear the smallest and largest readings saved during joystick setup
void clearCalibrationRanges() {
  for (int axis = 0; axis < maximumJoystickAxes; axis++) {
    calibrationMinimum[axis] = INT32_MAX;
    calibrationMaximum[axis] = INT32_MIN;
    calibrationAxisSeen[axis] = false;
  }
}

// Set up the joystick axes and scale their readings

// Use each controller report for either setup or normal controls
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

// Save each axis range so setup can find which one moved the most
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

// Choose the axis that moved the most and save its range and center
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
    Serial.println("Axis learning failed because no axis moved far enough");
    Serial.println("Repeating this joystick direction");
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

  Serial.print(axisBeingCalibrated->shortName);
  Serial.print(" learned as USB axis ");
  Serial.print(bestAxis);
  Serial.print(" range ");
  Serial.print(axisBeingCalibrated->minimum);
  Serial.print(" to ");
  Serial.print(axisBeingCalibrated->maximum);
  Serial.print(" center ");
  Serial.println(axisBeingCalibrated->center);

  calibrationAxisNumber++;
  startNextAxisCalibration();
}

// Scale each movement axis to a value from -1 to +1
void updateAllNormalizedJoystickValues() {
  updateNormalizedJoystickValue(&linearAxis);
  updateNormalizedJoystickValue(&linearGuardAxis);
  updateNormalizedJoystickValue(&rotationAxis);
  updateNormalizedJoystickValue(&flexionAxis);
}

// Scale the raw axis value to -1 through +1 and ignore the small center area
// The sign fixes joystick directions that are backward
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

// Update the saved range as the stick moves, even when using the preset axes
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

// Turn joystick movement into movement requests

// Make small stick movements gentle but still allow full speed at the ends
float applyJoystickResponseCurve(float value) {
  float cubicValue = value * value * value;
  return 0.35f * value + 0.65f * cubicValue;
}

// Only allow in/out movement when the left stick is close to its vertical center
float filterHorizontalAxisOnly(float rawHorizontalValue, float rawVerticalValue, float allowedCrossAxisMotion) {
  float horizontalMagnitude = fabsf(rawHorizontalValue);
  float verticalMagnitude = fabsf(rawVerticalValue);

  if (horizontalMagnitude <= 0.0f) {
    return 0.0f;
  }

  if (verticalMagnitude <= allowedCrossAxisMotion) {
    return rawHorizontalValue;
  }

  // Ignore the left stick if it is moved diagonally or up/down
  return 0.0f;
}

// Use the right stick to either bend the tip or turn the scope
// Ignore diagonal input when both directions move too far from center
void filterRightStickExclusiveAxes(float rawHorizontalValue, float rawVerticalValue, float *filteredHorizontalValue, float *filteredVerticalValue) {
  float horizontalMagnitude = fabsf(rawHorizontalValue);
  float verticalMagnitude = fabsf(rawVerticalValue);

  // Start by asking for no turning or bending
  *filteredHorizontalValue = 0.0f;
  *filteredVerticalValue = 0.0f;

  if (horizontalMagnitude <= 0.0f && verticalMagnitude <= 0.0f) {
    return;
  }

  // Moving mostly left/right bends the tip
  if (horizontalMagnitude > 0.0f &&
      verticalMagnitude <= rightStickAllowedCrossAxisMotion) {
    *filteredHorizontalValue = rawHorizontalValue;
    return;
  }

  // Moving mostly up/down turns the scope
  if (verticalMagnitude > 0.0f &&
      horizontalMagnitude <= rightStickAllowedCrossAxisMotion) {
    *filteredVerticalValue = rawVerticalValue;
    return;
  }

  // Keep both requests at zero if the stick is moved too far diagonally
}

// Run the command once when pressed, not over and over while the button is held
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

// Use the same command code for the Serial Monitor and controller
void runControllerCommand(char command, const char *sourceName) {
  if (command >= 'A' && command <= 'Z') {
    command = (char)(command + ('a' - 'A'));
  }

  beginCommandPrintBlock();

  if (command == 'a') {
    handleEnableReadyButton(sourceName);
    finishCommandPrintBlock();
    return;
  }

  printCommandIssued(commandDisplayName(command), sourceName);

  if (command == 'h') {
    if (laserFiring) {
      Serial.println("HOME blocked while laser fire outputs are active. Press Pause [p] or Stop [x] first.");
      finishCommandPrintBlock();
      return;
    }

    startHomeReturn();
    finishCommandPrintBlock();
    return;
  }

  if (handleLaserCommand(command)) {
    finishCommandPrintBlock();
    return;
  }

  if (laserFiring) {
    Serial.println("Unknown command while firing. Emergency stop triggered.");
    stopLaserAndRelock(true);
    Serial.println("All laser outputs are OFF. Laser commands are locked.");
    finishCommandPrintBlock();
    return;
  }

  Serial.println("Unknown command.");
  Serial.println("Valid commands are h/e/r/f/p/s/x.");
  printLaserCommandHelp();
  finishCommandPrintBlock();
}

// Give each command letter a name that is easier to read
const char *commandDisplayName(char command) {
  if (command == 'h') return "Home [h]";
  if (command == 'e') return "Enable [e]";
  if (command == 'r') return "Ready [r]";
  if (command == 'f') return "Fire [f]";
  if (command == 'p') return "Pause [p]";
  if (command == 's') return "Standby [s]";
  if (command == 'x') return "Stop / Exit [x]";
  return "Unknown command";
}

// Print the command and whether it came from the controller or Serial Monitor
void printCommandIssued(const char *commandName, const char *sourceName) {
  Serial.println();
  Serial.print("COMMAND: ");
  Serial.println(commandName);
  Serial.print("Source: ");
  Serial.println(sourceName);
  Serial.println();
}

// Pause status messages while printing a command
void beginCommandPrintBlock() {
  muteStatusPrintsFor(commandStatusQuietAfter_ms);
}

// Finish printing the command, then wait a little before printing status again
void finishCommandPrintBlock() {
  Serial.println();
  Serial.flush();
  muteStatusPrintsFor(commandStatusQuietAfter_ms);
}

// Keep status messages from making command messages hard to read
void muteStatusPrintsFor(unsigned long quietTimeMs) {
  unsigned long now = millis();
  unsigned long quietUntil = now + quietTimeMs;

  if ((long)(quietUntil - statusPrintMutedUntil_ms) > 0) {
    statusPrintMutedUntil_ms = quietUntil;
  }

  lastStatusPrintTime_ms = now;
}

// Turn all laser output signals off
void driveLaserOutputsToReleasedState() {
  digitalWrite(fire1_laser, LASER_FIRE_RELEASED);
  digitalWrite(fire2_laser, LASER_FIRE_RELEASED);
  digitalWrite(relay_laser, LASER_RELAY_RELEASED);
  laserFiring = false;
}

// Make Enable and Ready happen again before allowing Fire
void lockLaserSoftwareState() {
  laserEnabled = false;
  laserReady = false;
  laserFiring = false;
}

// Stop asking for movement, then pulse the Ready/Standby relay
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

// Turn Fire signals off but keep the saved Enable and Ready flags
void pauseLaserFiring() {
  digitalWrite(fire1_laser, LASER_FIRE_RELEASED);
  digitalWrite(fire2_laser, LASER_FIRE_RELEASED);
  laserFiring = false;
  holdCurrentMotionTargets();
  stopAllMotion();
}

// Turn Fire off, clear Ready if needed, and lock the laser commands again
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

// Print the list of laser buttons
void printLaserCommandHelp() {
  Serial.println("LASER: Left bumper=Enable/Ready | Right bumper=Fire | Left trigger=Standby | Right trigger=Pause | Bottom face=Stop");
}

// The left bumper first enables commands, then sends Ready when pressed again
bool handleEnableReadyButton(const char *sourceName) {
  if (laserFiring) {
    printCommandIssued("Emergency Stop [x]", sourceName);
    Serial.print("Before: ");
    Serial.print(laserEnabled ? "ENABLED" : "LOCKED");
    Serial.print(" | ");
    Serial.print(laserReady ? "READY" : "NOT READY");
    Serial.print(" | ");
    Serial.println(laserFiring ? "FIRING" : "NOT FIRING");
    Serial.println("Executing: EMERGENCY STOP");
    Serial.println("Reason: Enable/Ready is not allowed while fire outputs are ON.");
    stopLaserAndRelock(true);
    Serial.println("All laser outputs are OFF. Laser commands are locked.");
    return true;
  }

  if (!laserEnabled) {
    printCommandIssued("Enable [e]", sourceName);
    Serial.print("Before: ");
    Serial.print(laserEnabled ? "ENABLED" : "LOCKED");
    Serial.print(" | ");
    Serial.print(laserReady ? "READY" : "NOT READY");
    Serial.print(" | ");
    Serial.println(laserFiring ? "FIRING" : "NOT FIRING");
    Serial.println("Executing: ENABLE [e]");
    Serial.println("Why: laser commands were locked, so this left bumper press unlocks them.");
    return handleLaserCommand('e');
  }

  if (!laserReady) {
    printCommandIssued("Ready [r]", sourceName);
    Serial.print("Before: ");
    Serial.print(laserEnabled ? "ENABLED" : "LOCKED");
    Serial.print(" | ");
    Serial.print(laserReady ? "READY" : "NOT READY");
    Serial.print(" | ");
    Serial.println(laserFiring ? "FIRING" : "NOT FIRING");
    Serial.println("Executing: READY [r]");
    Serial.println("Why: laser is enabled but not ready, so this left bumper press sends the ready pulse.");
    return handleLaserCommand('r');
  }

  printCommandIssued("No command", sourceName);
  Serial.print("Before: ");
  Serial.print(laserEnabled ? "ENABLED" : "LOCKED");
  Serial.print(" | ");
  Serial.print(laserReady ? "READY" : "NOT READY");
  Serial.print(" | ");
  Serial.println(laserFiring ? "FIRING" : "NOT FIRING");
  Serial.println("Executing: nothing");
  Serial.println("Why: laser is already enabled and ready.");
  Serial.println("Next press Fire [f], Standby [s], Pause [p], or Stop [x].");
  return true;
}

// Keep track of which laser commands have been sent
// Only allow Fire when the Enable and Ready flags are set
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

    Serial.println("Executing: Enable [e]");
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
    Serial.println("Executing: Ready [r]");
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

    Serial.println("Executing: Fire [f]");
    Serial.println("WARNING: fire outputs are turning ON. Be careful for unintended firing.");
    digitalWrite(fire1_laser, LASER_FIRE_ACTIVE);
    digitalWrite(fire2_laser, LASER_FIRE_ACTIVE);
    laserFiring = true;
    Serial.println("Fire outputs are ON. Press Pause [p], Standby [s], or Stop [x] to turn them OFF.");
    return true;
  }

  if (command == 'p') {
    Serial.println("Executing: Pause [p]");
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

    Serial.println("Executing: Standby [s]");
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
    Serial.println("Executing: Stop / Exit [x]");
    holdCurrentMotionTargets();
    stopAllMotion();
    stopLaserAndRelock(true);
    Serial.println("All laser outputs are OFF. Laser commands are locked.");
    return true;
  }

  return false;
}

// Send all three movements back to their saved home targets
void startHomeReturn() {
  homeReturnActive = true;
  stopAllMotion();

  // Set the target positions and let the position-control code choose the speeds
  targetLinearPositionMM = homeLinearPositionMM;
  targetRotationUreteroscopeDeg = homeRotationUreteroscopeDeg;
  targetTipDeg = homeFlexionTipDeg;
  updateAbsoluteTargetDerivedNumbers();

  Serial.println("Executing: Home [h]");
  Serial.println("Absolute targets set to Linear 0 mm Rotation 0 deg Flexion 0 deg");
  Serial.println("Returning linear rotation and flexion to startup positions");
}

// Keep the home targets the same until all three positions are close enough
// Then set the saved positions back to their starting references
void updateHomeReturnTargets() {
  // Ignore the joysticks while going home
  targetLinearPositionMM = homeLinearPositionMM;
  targetRotationUreteroscopeDeg = homeRotationUreteroscopeDeg;
  targetTipDeg = homeFlexionTipDeg;
  targetFlexionSpeedDegPerSec = 0.0f;
  requestedLinearTargetRateMMPerSec = 0.0f;
  requestedRotationTargetRateDegPerSec = 0.0f;
  requestedFlexionTargetRateDegPerSec = 0.0f;
  updateAbsoluteTargetDerivedNumbers();

  // Set the in/out position to 0 mm once the encoder is inside the home zone
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

// Check if all three positions are close enough to home
bool homeReturnIsFinished() {
  float flexionHomeErrorDeg =
    currentFlexionStep >= 0
      ? measuredTipDeg - homeFlexionTipDeg
      : targetTipDeg - homeFlexionTipDeg;

  return linearIsInsideHomeTolerance() &&
         fabsf(currentRotationUreteroscopeDeg - homeRotationUreteroscopeDeg) <= homeRotationToleranceDeg &&
         fabsf(flexionHomeErrorDeg) <= homeFlexionToleranceDeg;
}

// Use the in/out encoder reading to check the home zone
bool linearIsInsideHomeTolerance() {

  return fabsf(currentLinearPositionMM - homeLinearPositionMM) <= homeLinearToleranceMM;
}

// Check if HOME is moving the carriage toward the lower travel limit
bool linearIsBeingCommandedTowardHome() {
  bool targetIsHome =
    fabsf(targetLinearPositionMM - homeLinearPositionMM) <= homeLinearToleranceMM;

  bool movingTowardLowerLimit =
    targetLinearSpeedMMPerSec < -stoppedLinearSpeedToleranceMMPerSec ||
    currentLinearSpeedMMPerSec < -stoppedLinearSpeedToleranceMMPerSec;

  return homeReturnActive && targetIsHome && movingTowardLowerLimit;
}

// Treat the current in/out position as 0 mm when it is inside the home zone
void redefineLinearHomePosition(const char *reason) {
  stopLinearImmediately();

  // Reset both the saved in/out position and its encoder reference
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

// During HOME, reset in/out to zero if it stops moving inside the home zone
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

// Use the joystick values to ask for movement
// Do not accept new requests during setup, after a fault, or while the Fire flag is set
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

  float filteredLinearInput =
    filterHorizontalAxisOnly(
      linearAxis.normalized,
      linearGuardAxis.normalized,
      leftStickAllowedCrossAxisMotion
    );

  float linearRequest =
    applyJoystickResponseCurve(filteredLinearInput) * maximumLinearSpeedMMPerSec;

  // The left joystick directly sets the in/out speed
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

  // Use the saved angle to stop requests past -180 or +180 degrees
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

// Keep track of the target positions

// Add each bit of joystick movement to the rotation target
// During normal control, show the measured in/out position as its target
void updateAbsoluteLinearAndRotationTargets() {
  unsigned long now_us = micros();
  unsigned long elapsed_us = now_us - lastAbsoluteTargetUpdate_us;
  lastAbsoluteTargetUpdate_us = now_us;

  // Limit the time step so a pause does not make the target jump
  if (elapsed_us > 50000) {
    elapsed_us = 50000;
  }

  float elapsedSeconds = (float)elapsed_us / 1000000.0f;

  if (!controlsReady || systemFaultLatched || laserFiring) {
    return;
  }

  // Only change joystick targets here; leave the home targets alone
  if (!homeReturnActive) {
    // Keep the printed in/out target matched to the encoder reading
    targetLinearPositionMM = currentLinearPositionMM;

    // Keep the total turn target so HOME can turn back to the starting zero
    targetRotationUreteroscopeDeg += requestedRotationTargetRateDegPerSec * elapsedSeconds;

    // Keep the turn target between -180 and +180 degrees
    targetRotationUreteroscopeDeg = clampFloat(
      targetRotationUreteroscopeDeg,
      rotationMinPositionDeg,
      rotationMaxPositionDeg
    );
  }

  snapAbsoluteTargetsNearLimits();
  updateAbsoluteTargetDerivedNumbers();
}

// Work out the motor values from the scope targets for commands and status
void updateAbsoluteTargetDerivedNumbers() {
  // Turn carriage travel in mm into motor turns and estimated steps
  targetLinearMotorRevolutions = targetLinearPositionMM / linearMMPerRev;
  targetLinearMotorStepEstimate = estimateLinearMotorStepsFromMM(targetLinearPositionMM);

  // Include the rotation gear ratio
  targetRotationMotorDeg = targetRotationUreteroscopeDeg * rotationGearRatio;
  targetRotationMapDeg = normalizeRotationMapDeg(targetRotationUreteroscopeDeg);
  targetRotationMotorStepEstimate = estimateRotationMotorStepsFromDeg(targetRotationMotorDeg);

  // Work out the servo position needed for the tip angle
  int calculatedStep = flexionNeutralStep;
  float calculatedMotorDeg = 0.0f;

  if (calculateFlexionStepFromTip(targetTipDeg, &calculatedStep, &calculatedMotorDeg)) {
    targetFlexionMotorDeg = calculatedMotorDeg;
    targetFlexionPhysicalDeg = targetFlexionMotorDeg + halfRevolutionDegrees;
    targetFlexionStep = calculatedStep;
  }
}

// Use the distance from each target to work out the speed
// Rotation follows its angle target; in/out only follows a position target during HOME
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

    // During HOME, move in/out toward the home position
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

  // Turn the scope toward its target angle
  float rotationErrorDeg = targetRotationUreteroscopeDeg - currentRotationUreteroscopeDeg;
  targetRotationSpeedDegPerSec = calculatePositionControllerSpeed(
    rotationErrorDeg,
    rotationPositionTargetToleranceDeg,
    rotationPositionControllerGain,
    maximumRotationSpeedDegPerSec
  );
}

// Use the distance left to pick a speed, or stop if close enough
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

// Keep a value between its minimum and maximum
float clampFloat(float value, float minimumValue, float maximumValue) {
  if (value < minimumValue) {
    return minimumValue;
  }

  if (value > maximumValue) {
    return maximumValue;
  }

  return value;
}

// Clear tiny differences near home and the travel limits
void snapAbsoluteTargetsNearLimits() {
  bool linearTargetIsBeingMoved =
    fabsf(requestedLinearTargetRateMMPerSec) >= stoppedLinearSpeedToleranceMMPerSec;

  bool rotationTargetIsBeingMoved =
    fabsf(requestedRotationTargetRateDegPerSec) >= stoppedRotationSpeedToleranceDegPerSec;

  bool flexionTargetIsBeingMoved =
    fabsf(requestedFlexionTargetRateDegPerSec) >= stoppedFlexionSpeedToleranceDegPerSec;

  // Only round stopped targets so small joystick moves can still leave home
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

// Estimate motor steps from the in/out distance in mm
long estimateLinearMotorStepsFromMM(float positionMM) {
  float motorRevolutions = positionMM / linearMMPerRev;
  return lroundf(motorRevolutions * linearPulsesPerRev);
}

// Estimate motor steps from the motor rotation angle
long estimateRotationMotorStepsFromDeg(float motorDeg) {
  return lroundf((motorDeg / degreesPerRevolution) * rotationPulsesPerRev);
}

// Keep speed changes gradual

// Bring each speed toward the requested speed without speeding up or braking too fast
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

// Move toward the target without going past it
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

// Use the speed-up rate when speeding up, and the braking rate when slowing or reversing
float chooseRampRate(float currentValue, float targetValue, float acceleration, float deceleration) {
  bool sameDirection =
    (targetValue >= 0.0f && currentValue >= 0.0f) ||
    (targetValue <= 0.0f && currentValue <= 0.0f);

  bool increasingMagnitude = fabsf(targetValue) > fabsf(currentValue);

  return sameDirection && increasingMagnitude ? acceleration : deceleration;
}

// Send step pulses while the rest of the loop keeps running

// Send in/out STEP pulses without making the main loop wait
void updateLinearStepperPulseOutput() {
  unsigned long now_us = micros();

  // Bring STEP back LOW once it has stayed HIGH long enough
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

  // Change direction while STEP is LOW, then give the driver time to read it
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

  // Work out how many step pulses per second give the requested in/out speed
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

// Send rotation STEP pulses while the main loop keeps running
void updateRotationStepperPulseOutput() {
  unsigned long now_us = micros();

  // Bring STEP back LOW once it has stayed HIGH long enough
  if (rotationStepPulseIsHigh && now_us - rotationStepPulseHighStart_us >= stepPulseHighTime_us) {
    digitalWrite(rotation_stepPin, LOW);
    rotationStepPulseIsHigh = false;
  }

  if (systemFaultLatched ||
      fabsf(currentRotationSpeedDegPerSec) < stoppedRotationSpeedToleranceDegPerSec) {
    return;
  }

  // Stop turning farther out when the tracked angle reaches a limit
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

  // Change direction while STEP is LOW, then give the driver time to read it
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

  // Use the gear ratio to turn scope rotation speed into motor speed
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

// Check if the in/out movement would go past a saved travel limit
bool linearCommandWouldPushPastLimit(float speedMMPerSec) {
  // Stop backward requests at the measured lower limit
  if (speedMMPerSec < 0.0f &&
      currentLinearPositionMM <= linearMinPositionMM + linearRangeToleranceMM) {
    return true;
  }

  // Stop forward requests at the measured 177 mm limit
  if (speedMMPerSec > 0.0f &&
      currentLinearPositionMM >= linearMaxPositionMM - linearRangeToleranceMM) {
    return true;
  }

  return false;
}

// Stop the in/out step pulses and reset its speed and direction
void stopLinearImmediately() {
  targetLinearSpeedMMPerSec = 0.0f;
  currentLinearSpeedMMPerSec = 0.0f;
  digitalWrite(linear_stepPin, LOW);
  linearStepPulseIsHigh = false;
  linearActiveDirectionSign = 0;
}

// Stop the rotation step pulses and reset its speed and direction
void stopRotationImmediately() {
  targetRotationSpeedDegPerSec = 0.0f;
  currentRotationSpeedDegPerSec = 0.0f;
  digitalWrite(rotation_stepPin, LOW);
  rotationStepPulseIsHigh = false;
  rotationActiveDirectionSign = 0;
}

// Stop stepper movement and clear new tip-angle requests
// A move already sent to the servo can still keep going
void stopAllMotion() {
  stopLinearImmediately();
  stopRotationImmediately();
  requestedLinearTargetRateMMPerSec = 0.0f;
  requestedRotationTargetRateDegPerSec = 0.0f;
  requestedFlexionTargetRateDegPerSec = 0.0f;
  targetFlexionSpeedDegPerSec = 0.0f;
  currentFlexionSpeedDegPerSec = 0.0f;
  linearAxis.normalized = 0.0f;
  linearGuardAxis.normalized = 0.0f;
  rotationAxis.normalized = 0.0f;
  flexionAxis.normalized = 0.0f;
}

// Read movement from the encoders

// Use encoder counts to update positions for the limits, HOME, and status messages
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

  // Work out motor values from the carriage position for the status messages
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

  // Estimate motor steps from the total turn angle
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

// Set a motor fault if the motor is told to move but the encoder does not change
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

// Keep the fault set, clear movement requests, and turn off the stepper drivers
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

// Move the tip

// Use the smoothed speed to change the tip target without going past its limits
void updateFlexionTargetAngle() {
  unsigned long now_us = micros();
  unsigned long elapsed_us = now_us - lastFlexionTargetUpdate_us;
  lastFlexionTargetUpdate_us = now_us;

  if (elapsed_us > 50000) {
    elapsed_us = 50000;
  }

  float elapsedSeconds = (float)elapsed_us / 1000000.0f;

  if (!homeReturnActive && !laserFiring) {
    // When the stick is released, leave the tip target where it is
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

// Change smoothly between the tip factors around 90 and 180 degrees
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

// Choose the factor that estimates the tip angle from the servo angle
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

// Make a smooth change from 0 to 1 when mixing the tip factors
float smoothStep01(float value) {
  if (value <= 0.0f) {
    return 0.0f;
  }

  if (value >= 1.0f) {
    return 1.0f;
  }

  return value * value * (3.0f - 2.0f * value);
}

// Mix two values together
float blendFloat(float firstValue, float secondValue, float blendAmount) {
  return firstValue + (secondValue - firstValue) * blendAmount;
}

// Slow the tip target down as it gets close to a limit
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

// Find the servo step for this tip angle and check both sets of limits
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

// Change the servo speed setting a little at a time
int updateSmoothedFlexionServoSpeed(int desiredServoSpeed) {
  unsigned long now_ms = millis();
  unsigned long elapsed_ms = now_ms - lastServoSpeedRampTime_ms;
  lastServoSpeedRampTime_ms = now_ms;

  // Limit the time step so a pause does not make the speed jump
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

// Send the servo its target, speed, and acceleration when the update timer is ready
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

// Read the servo position and use the bending factors to estimate the tip angle
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

// Only print another tip warning when the warning level changes
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

// Print status when its timer is ready, but wait during commands and HIGH step pulses
void printStatusIfReady() {
  if (!controlsReady) {
    return;
  }

  unsigned long now = millis();

  if ((long)(now - statusPrintMutedUntil_ms) < 0) {
    return;
  }

  if (now - lastStatusPrintTime_ms < statusPrintInterval_ms) {
    return;
  }

  if (linearStepPulseIsHigh || rotationStepPulseIsHigh) {
    return;
  }

  lastStatusPrintTime_ms = now;

  if (homeReturnActive) {
    Serial.print("HOMING | ");
  }

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

// Extra encoder functions

// Keep the displayed angle within one full turn in either direction
// Save the total rotation too so HOME can return to the start
float normalizeRotationMapDeg(float angleDeg) {
  while (angleDeg >= degreesPerRevolution) {
    angleDeg -= degreesPerRevolution;
  }

  while (angleDeg <= -degreesPerRevolution) {
    angleDeg += degreesPerRevolution;
  }

  return angleDeg;
}

// Count an in/out encoder edge and read B to find the direction
void linearEncoderISR() {
  if (digitalRead(linear_encB) == HIGH) {
    linearEncoderCount++;
  } else {
    linearEncoderCount--;
  }
}

// Count a rotation encoder edge and read B to find the direction
void rotationEncoderISR() {
  if (digitalRead(rotation_encB) == HIGH) {
    rotationEncoderCount++;
  } else {
    rotationEncoderCount--;
  }
}

// Pause interrupts for a moment so the encoder count stays the same while it is copied
long getLinearEncoderCount() {
  noInterrupts();
  long count = linearEncoderCount;
  interrupts();
  return count;
}

// Pause interrupts for a moment so the encoder count stays the same while it is copied
long getRotationEncoderCount() {
  noInterrupts();
  long count = rotationEncoderCount;
  interrupts();
  return count;
}

// Reset the in/out encoder count and the last count used by the loop together
void resetLinearEncoder() {
  noInterrupts();
  linearEncoderCount = 0;
  interrupts();
  lastProcessedLinearSignedCount = 0;
}

// Reset the rotation encoder count and the last count used by the loop together
void resetRotationEncoder() {
  noInterrupts();
  rotationEncoderCount = 0;
  interrupts();
  lastProcessedRotationSignedCount = 0;
}
