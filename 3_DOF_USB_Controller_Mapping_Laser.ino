/// Full 3 DOF USB Controller Diagnostics with Home and Laser Buttons
/*
  Overall Purpose:
    Run a full diagnostic controller setup for all three ureteroscope motion axes and the Dornier laser interface
    The diagnostic startup learns joystick axes and asks the user to press each command button or trigger once
    The linear axis moves the carriage with a NEMA 14 stepper motor and encoder feedback
    The rotation axis turns the ureteroscope with a NEMA 17 stepper motor and encoder feedback
    The flexion axis bends the ureteroscope tip with an ST3020 smart servo
    The laser controls send software-managed Enable, Ready, Fire, Pause, Standby, and Stop commands

  Hardware:
    Teensy board with USB host support
    USB game controller
    NEMA 14 stepper motor for the linear carriage
    TB6600 driver for the linear NEMA 14 stepper motor
    Linear quadrature encoder
    NEMA 17 stepper motor for ureteroscope rotation
    TB6600 driver for the rotation NEMA 17 stepper motor
    Rotation quadrature encoder
    ST3020 smart servo for ureteroscope tip flexion
    Dornier laser relay/contact interface
    External motor, servo, and relay power supplies
    Shared ground between the Teensy, motor drivers, encoders, servo system, and relay supply

  Pins:
    Pin 2 connects to the linear TB6600 ENA input
    Pin 3 connects to the linear TB6600 DIR input
    Pin 4 connects to the linear TB6600 STEP input
    Pin 17 connects to linear encoder channel A
    Pin 16 connects to linear encoder channel B
    Pin 5 connects to the rotation TB6600 ENA input
    Pin 6 connects to the rotation TB6600 DIR input
    Pin 7 connects to the rotation TB6600 STEP input
    Pin 20 connects to rotation encoder channel A
    Pin 21 connects to rotation encoder channel B
    Pin 8 connects to the Dornier laser ready/standby relay or contact channel
    Pin 9 connects to Dornier laser fire output channel 1
    Pin 10 connects to Dornier laser fire output channel 2
    Serial1 connects to the ST3020 servo communication bus at 1000000 baud
    The USB controller connects through the Teensy USB host port

  Other:
    Left joystick left and right controls linear motion
    Right joystick up and down controls rotation
    Right joystick left and right controls flexion
    B button returns linear, rotation, and flexion to the startup home positions
    Left bumper first sends Enable [e], then sends Ready [r] on the next press
    Left trigger sends Standby [s]
    Right bumper sends Fire [f]
    Right trigger sends Pause [p]
    Bottom face button sends Stop/Exit [x]
    The Guide button is not learned because it is only used to power or connect the controller
    Rotation is limited from -180 to +180 degrees to match the Isaac Sim VR Teensy code
    The startup position is treated as home for all three axes
    Joystick motion is blocked while the laser fire outputs are active
    This diagnostic version is best when checking a new controller or verifying real button and trigger mappings
*/

#include <USBHost_t36.h>
#include <SCServo.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

SMS_STS servo;

///----------------------------------------------- Pin Assignments -----------------------------------------------

// Linear NEMA 14 and TB6600 pins
const int linear_enaPin = 2;   // Pin 2 sends enable control to the linear TB6600 driver ENA input
const int linear_dirPin = 3;   // Pin 3 sends direction control to the linear TB6600 driver DIR input
const int linear_stepPin = 4;  // Pin 4 sends step pulses to the linear TB6600 driver STEP input
const int linear_encA = 17;    // Pin 17 reads linear encoder channel A
const int linear_encB = 16;    // Pin 16 reads linear encoder channel B

// Rotation NEMA 17 and TB6600 pins
const int rotation_enaPin = 5;   // Pin 5 sends enable control to the rotation TB6600 driver ENA input
const int rotation_dirPin = 6;   // Pin 6 sends direction control to the rotation TB6600 driver DIR input
const int rotation_stepPin = 7;  // Pin 7 sends step pulses to the rotation TB6600 driver STEP input
const int rotation_encA = 20;    // Pin 20 reads rotation encoder channel A
const int rotation_encB = 21;    // Pin 21 reads rotation encoder channel B

// Dornier laser relay/contact pins
const int relay_laser = 8;   // Pin 8 controls the laser ready/standby relay or contact channel
const int fire1_laser = 9;   // Pin 9 controls laser fire output channel 1
const int fire2_laser = 10;  // Pin 10 controls laser fire output channel 2

///--------------------------------------------- Electrical Settings ---------------------------------------------

// This wiring enables the TB6600 drivers when ENA is LOW
const int enaPin_Enable = LOW;

// This wiring disables the TB6600 drivers when ENA is HIGH
const int enaPin_Disable = HIGH;

// Linear direction levels
const int linear_dirForward = HIGH;
const int linear_dirBackward = LOW;

// Rotation direction levels
const int rotation_dirClockwise = LOW;
const int rotation_dirCounter = HIGH;

// Ready/standby relay channel is active-low:
//   LOW  = relay/contact closes
//   HIGH = relay/contact opens
const int LASER_RELAY_ACTIVE = LOW;
const int LASER_RELAY_RELEASED = HIGH;

// Fire relay channels are active-high:
//   HIGH = fire command asserted
//   LOW  = fire command released
const int LASER_FIRE_ACTIVE = HIGH;
const int LASER_FIRE_RELEASED = LOW;

// Ready and standby use the same physical pulse pattern
const unsigned long laserControlPulse_ms = 3000;

///----------------------------------------------- Linear Calibration --------------------------------------------

const float linearMMPerRev = 4.0f;
const float linearPulsesPerRev = 1600.0f;
const float linearEncoderCPR = 1000.0f;
const float linearMinPositionMM = 0.0f;
const float linearMaxPositionMM = 177.0f;
const float linearRangeToleranceMM = 0.01f;
const int linearEncoderSign = +1;

///---------------------------------------------- Rotation Calibration -------------------------------------------

const float rotationEncoderCPR = 1000.0f;
const float rotationPulsesPerRev = 1600.0f;
const float rotationGearRatio = 2.5f;
const int rotationEncoderSign = -1;
const float rotationMapZeroToleranceDeg = 0.5f;

const float rotationMinPositionDeg = -180.0f;
const float rotationMaxPositionDeg = 180.0f;
const float rotationRangeToleranceDeg = 0.5f;

///------------------------------------------ FLEXION TUNING BLOCK -----------------------------------------------
/// Change these values first while experimenting with tip flexion calibration
/// The factor is selected from the absolute requested tip angle
/// Positive and negative flexion use the same ranges

const float flexionTwistFactor0To90 = 6.0f;
const float flexionTwistFactor90To180 = 6.5f;
const float flexionTwistFactor180To270 = 7.2f;

// The gear ratio converts ST3020 motor motion into cable twist behavior
const float flexionGearRatio = 3.5f;

// Tip gain is shown explicitly so it is easy to inspect while tuning
// If you change a twist factor above these gains update automatically
const float flexionTipGain0To90 = flexionTwistFactor0To90 / flexionGearRatio;
const float flexionTipGain90To180 = flexionTwistFactor90To180 / flexionGearRatio;
const float flexionTipGain180To270 = flexionTwistFactor180To270 / flexionGearRatio;

// Blend the calibration gain across each boundary instead of switching instantly
// A wider 30 degree blend means 75 to 105 degrees gradually changes from range 1 to range 2
// A wider 30 degree blend also means 165 to 195 degrees gradually changes from range 2 to range 3
// Increase this if you still feel hesitation near 90 or 180 degrees
// Decrease this if you want the three calibration ranges to stay more separated
const float flexionTransitionBlendWidthDeg = 30.0f;

// Slow the joystick target near the mechanical flexion limits instead of clipping abruptly
const float flexionSoftLimitZoneDeg = 10.0f;

///------------------------------------------ ST3020 Communication -----------------------------------------------

const long servo_baud = 1000000;
const int servo_ID = 1;
const int servo_speed = 500;
const int servo_acceleration = 100;
const int servoMaximumStep = 4095;
const int flexionNeutralStep = 2048;
const float degreesPerRevolution = 360.0f;
const float halfRevolutionDegrees = 180.0f;

///----------------------------------------- Flexion Speed Reduction ---------------------------------------------

const int flexionSlowdownStartTipDeg = 100;
const float flexionSpeedReductionPerTipDeg = 2.5f;
const int flexionMinimumSpeed = 50;

// Limit how quickly the ST3020 speed command itself is allowed to change
// This smooths speed changes caused by crossing flexion ranges or high flexion slowdown
// Higher values respond faster but can feel more abrupt
// Lower values feel smoother but can feel more delayed
const float flexionServoSpeedRampUnitsPerSec = 300.0f;

///------------------------------------------- Overstretch Warnings ----------------------------------------------

const int flexionSmallWarningTipDeg = 150;
const int flexionModerateWarningTipDeg = 200;
const int flexionMajorWarningTipDeg = 250;

///---------------------------------------------- Flexion Range ---------------------------------------------------

const float flexionMaxTipDeg = 270.0f;
const float flexionMinTipDeg = -270.0f;
const float flexionMaxMotorDeg = 180.0f;
const float flexionMinMotorDeg = -180.0f;
const int flexionStopMarginSteps = 1;
const float flexionRangeToleranceMotorDeg = degreesPerRevolution / (float)servoMaximumStep;
const float flexionRangeToleranceTipDeg =
  flexionRangeToleranceMotorDeg * flexionTipGain180To270;

///------------------------------------------ Smooth Motion Settings ---------------------------------------------

// Linear maximum joystick speed in millimeters per second
const float maximumLinearSpeedMMPerSec = 2.0f;

// Rotation maximum joystick speed in ureteroscope degrees per second
const float maximumRotationSpeedDegPerSec = 30.0f;

// Flexion maximum target-change speed in tip degrees per second
const float maximumFlexionTargetSpeedDegPerSec = 35.0f;

// Acceleration and deceleration values for smooth control
const float linearAccelerationMMPerSec2 = 6.0f;
const float linearDecelerationMMPerSec2 = 10.0f;
const float rotationAccelerationDegPerSec2 = 90.0f;
const float rotationDecelerationDegPerSec2 = 140.0f;
const float flexionAccelerationDegPerSec2 = 120.0f;
const float flexionDecelerationDegPerSec2 = 180.0f;

// Speeds below these values are treated as exact zero
const float stoppedLinearSpeedToleranceMMPerSec = 0.005f;
const float stoppedRotationSpeedToleranceDegPerSec = 0.05f;
const float stoppedFlexionSpeedToleranceDegPerSec = 0.05f;

///--------------------------------------- Absolute Position Controller ------------------------------------------

// These gains convert position error into a speed command
// Example for linear: if the target is 0.20 mm away and the gain is 8, the requested speed is 1.6 mm/s
// The requested speed is still limited by maximumLinearSpeedMMPerSec and acceleration smoothing
const float linearPositionControllerGain = 8.0f;
const float rotationPositionControllerGain = 5.0f;

// These tolerances decide when the axis is close enough to its absolute target to stop
// Linear tolerance is intentionally small because 0.01 mm is about 2.5 encoder counts
// Rotation tolerance is larger because the rotation encoder and gear train have more practical play
const float linearPositionTargetToleranceMM = 0.01f;
const float rotationPositionTargetToleranceDeg = 0.25f;

// If an idle target lands extremely close to home or a travel limit, snap it exactly there
// This prevents tiny floating-point leftovers such as 0.00003 mm from keeping HOME active
// Active joystick movement is allowed to leave home normally, even if the first increments are tiny
const float absoluteTargetSnapToleranceMM = 0.02f;
const float absoluteTargetSnapToleranceDeg = 0.25f;

// Encoder stall settings for runtime safety
const float linearStallMinimumSpeedMMPerSec = 0.15f;
const float rotationStallMinimumSpeedDegPerSec = 3.0f;
const unsigned long encoderStallTimeout_ms = 750;

// Loop timing settings
const unsigned long encoderUpdateInterval_ms = 10;
const unsigned long servoCommandInterval_ms = 40;
const unsigned long servoReadInterval_ms = 80;
const unsigned long statusPrintInterval_ms = 500;
const unsigned long startupSerialSettle_ms = 1200;
const unsigned long configurationPrintPause_ms = 200;
const unsigned long buttonSetupReleaseStable_ms = 500;
const unsigned int stepPulseHighTime_us = 8;
const unsigned int directionSetupTime_us = 10;

///--------------------------------------------- USB Host Objects -------------------------------------------------

USBHost usbHost;
USBHub usbHub(usbHost);
USBHIDParser hidParser(usbHost);
JoystickController joystick(usbHost);

///------------------------------------------ Joystick Calibration -----------------------------------------------

// Axis learning uses short windows so controller setup is quick
const unsigned long calibrationPreparation_ms = 1500;
const unsigned long calibrationCapture_ms = 3000;

// Center dead zone removes stick drift and tiny hand tremor near center
// Reduced from 0.18 to 0.12 so intentional joystick motion responds sooner
const float joystickDeadZoneFraction = 0.12f;

// USBHost_t36 can report many axes on complex controllers
const int maximumJoystickAxes = 64;

// An axis must move by at least this many raw units to be accepted
const int32_t minimumCalibrationSpan = 20;

// Live range tracking starts gently instead of treating the first tiny motion as full scale
const int32_t minimumLiveAxisHalfRange = 100;

// Change these signs if any joystick direction feels reversed
const int linearJoystickDirectionSign = +1;
const int rotationJoystickDirectionSign = -1;
const int flexionJoystickDirectionSign = +1;

// Diagnostics mode intentionally learns axes at startup
// Set this false after you know your controller matches the default axis constants below
const bool useStartupAxisLearning = true;

// Diagnostics mode intentionally learns buttons/triggers at startup
// The Guide button is skipped because it is only the controller power/connect button
const bool useStartupButtonLearning = true;

// Common default USB axis numbers for many game controllers
// If the joysticks respond incorrectly, change only these four constants first
const int defaultLinearXAxisIndex = 0;
const int defaultFlexionXAxisIndex = 2;
const int defaultRotationYAxisIndex = 5;

// Some gamepads report LT/RT as analog axes instead of normal button bits
// During button setup, an axis movement larger than this can be learned as a trigger command
const int32_t minimumButtonAxisChange = 80;
const float analogButtonPressFraction = 0.55f;
const float buttonSetupJoystickCenteredFraction = 0.18f;

// Assumed/default button masks for a common Xbox-style USB gamepad report
// If a button does the wrong command, use the diagnostics sketch to print the true mask,
// then update only the matching constant here
const uint32_t defaultHomeButtonMask = 0x00000002;         // B
const uint32_t defaultEnableReadyButtonMask = 0x00000010;  // Left bumper/shoulder
const uint32_t defaultFireButtonMask = 0x00000020;         // Right bumper/shoulder
const uint32_t defaultStopButtonMask = 0x00000004;         // Bottom face button on Sarah's controller

// Assumed/default analog trigger axes
// The code records the released value at startup, then treats movement in the
// positive direction as a press. Change the axis index or direction if diagnostics
// shows your LT/RT report differently
const bool defaultStandbyUsesAxis = false;            // LT reports as a button on Sarah's controller
const int defaultStandbyAxisIndex = 3;
const int defaultStandbyAxisDirection = +1;
const int32_t defaultStandbyAxisTravelEstimate = 255;
const uint32_t defaultStandbyButtonMask = 0x00000040; // LT

const bool defaultPauseUsesAxis = false;              // RT reports as a button on Sarah's controller
const int defaultPauseAxisIndex = 4;
const int defaultPauseAxisDirection = +1;
const int32_t defaultPauseAxisTravelEstimate = 255;
const uint32_t defaultPauseButtonMask = 0x00000080;   // RT

// Rotation and flexion share the right joystick, so only one right-stick axis may command motion
// If the other right-stick axis is larger than this value the stick is treated as diagonal
// A diagonal command intentionally produces no rotation and no flexion
const float rightStickAllowedCrossAxisMotion = 0.18f;

///--------------------------------------------- Home Button Settings --------------------------------------------

// Pressing the learned HOME button drives all axes back to the startup positions
// Linear returns to tracked 0 mm
// Rotation returns the unwrapped accumulated rotation to 0 degrees
// Flexion returns to absolute 0 degrees
const float homeLinearPositionMM = linearMinPositionMM;
const float homeRotationUreteroscopeDeg = 0.0f;
const float homeFlexionTipDeg = 0.0f;

// These tolerances decide when each axis is close enough to count as home
// Linear HOME uses currentLinearPositionMM, which comes from encoder feedback
// That is the most reliable absolute linear position available in this program
// This is intentionally larger than the normal target tolerance so a slightly larger region counts as 0 mm
const float homeLinearToleranceMM = 0.50f;
const float homeRotationToleranceDeg = 1.0f;
const float homeFlexionToleranceDeg = 1.0f;

// If HOME is trying to move linear toward 0 mm but the encoder stops changing inside the home tolerance
// The program treats that as physically home and redefines the current linear position as 0 mm
// This protects the carriage from continuing to push into the mechanical 0 mm end
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

///--------------------------------------------- Motion State -----------------------------------------------------

// These three variables are the joystick-requested target-position rates
// They are not sent directly to the motors
// They are integrated into absolute target positions first
float requestedLinearTargetRateMMPerSec = 0.0f;
float requestedRotationTargetRateDegPerSec = 0.0f;
float requestedFlexionTargetRateDegPerSec = 0.0f;

// These are the actual speed commands produced by the absolute position controller
// For normal joystick movement, linear speed is assigned directly like the working standardized-axis sketch
// Rotation still uses the position controller to chase its unwrapped absolute target
// For flexion, targetFlexionSpeedDegPerSec is the smooth speed used to move the absolute tip-angle target
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

///---------------------------------------------- Laser State -----------------------------------------------------

bool laserEnabled = false;
bool laserReady = false;
bool laserFiring = false;

///--------------------------------------------- Encoder State ----------------------------------------------------

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

///--------------------------------------------- Flexion State ----------------------------------------------------

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

// This is the filtered speed number sent to the ST3020
// It starts at the normal speed and moves gradually toward each newly calculated speed
float smoothedFlexionServoSpeed = servo_speed;

///--------------------------------------------- Status State -----------------------------------------------------

unsigned long lastStatusPrintTime_ms = 0;

///--------------------------------------------- Function Prototypes ---------------------------------------------

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

///---------------------------------------------------- Setup -----------------------------------------------------

// setup runs once when the Teensy starts
// It prepares every hardware interface and defines the startup position as HOME
void setup() {
  Serial.begin(115200);
  delay(startupSerialSettle_ms);

  // Configure each hardware group before controller diagnostics begin
  setupStepperPins();
  setupEncoders();
  setupServo();
  setupLaserOutputs();
  setupUsb();

  // Treat the physical startup position as the absolute home reference
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

// Prepare the stepper driver pins and leave both drivers enabled
// STEP pins start LOW so no accidental step pulse is sent during startup
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

// Prepare the quadrature encoder pins and attach interrupts to channel A
// Channel B is read inside the interrupt so the code can tell direction
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

// Start the serial bus used by the ST3020 smart servo
// The SCServo library sends commands through Serial1
void setupServo() {
  Serial1.begin(servo_baud);
  servo.pSerial = &Serial1;
}

// Start the USB host system that reads the physical game controller
// The calibration arrays are cleared before any controller data is processed
void setupUsb() {
  usbHost.begin();
  clearCalibrationRanges();
}

// Put every laser-related output into its inactive state before enabling the pins
// This helps prevent a brief unintended relay or fire signal during startup
void setupLaserOutputs() {
  driveLaserOutputsToReleasedState();

  pinMode(relay_laser, OUTPUT);
  pinMode(fire1_laser, OUTPUT);
  pinMode(fire2_laser, OUTPUT);

  driveLaserOutputsToReleasedState();
  lockLaserSoftwareState();
}

// Print the diagnostic startup guide to the Serial Monitor
// This tells the user what order the joystick and button prompts will use
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

// Give the Serial Monitor time to display each important setup message
// This keeps diagnostic prompts from scrolling too quickly while the user is setting up
void pauseAfterConfigurationPrint() {
  Serial.flush();
  delay(configurationPrintPause_ms);
}

///----------------------------------------------------- Loop -----------------------------------------------------

// loop runs repeatedly after setup finishes
// Each pass checks inputs, updates targets, moves hardware, watches faults, and prints status
void loop() {
  // USBHost_t36 requires Task to be called often so controller packets are received
  usbHost.Task();

  updateControllerConnection();
  handleSerialCommands();

  // Process fresh controller data only when a new packet is available
  if (joystick && joystick.available()) {
    processJoystickPacket();
    joystick.joystickDataClear();
  }

  // Axis calibration is split across time so the controller can keep being serviced
  if (joystickState == PREPARING_AXIS_CALIBRATION &&
      millis() - joystickStateStartTime >= calibrationPreparation_ms) {
    beginAxisCalibrationCapture();
  }

  if (joystickState == CAPTURING_AXIS_CALIBRATION &&
      millis() - joystickStateStartTime >= calibrationCapture_ms) {
    finishCurrentAxisCalibration();
  }

  // The order below matters because targets must be calculated before motors are commanded
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

///------------------------------------------ Controller Connection ----------------------------------------------

// Read one-character commands from the Serial Monitor
// This gives a backup way to test HOME and laser commands without pressing controller buttons
void handleSerialCommands() {
  while (Serial.available() > 0) {
    char command = (char)Serial.read();

    // Ignore whitespace so pressing Enter in the Serial Monitor does not create a command
    if (command == '\n' || command == '\r' || command == ' ' || command == '\t') {
      continue;
    }

    // Convert uppercase letters to lowercase so H and h behave the same
    if (command >= 'A' && command <= 'Z') {
      command = (char)(command + ('a' - 'A'));
    }

    runControllerCommand(command, "Serial Monitor");
  }
}

// Detect when the USB controller is plugged in or removed
// A new connection starts diagnostic learning or loads the default mappings
// A disconnection immediately stops motion and returns laser outputs to a safe state
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

// Load the known axis numbers for the standardized controller layout
// This path is used only when diagnostic axis learning is turned off
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

// Store one default axis number and reset the live range data for that axis
// The live range fills in as the controller begins reporting real values
void configureJoystickAxis(LearnedJoystickAxis *axis, int defaultAxisIndex) {
  axis->axisIndex = defaultAxisIndex;
  axis->minimum = 0;
  axis->center = 0;
  axis->maximum = 0;
  axis->raw = 0;
  axis->normalized = 0.0f;
  axis->rangeInitialized = false;
}

// Begin the diagnostic joystick axis-learning workflow
// This asks the user to move each joystick direction so the sketch can find real USB axis numbers
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

// Move through the axis-learning prompts one axis at a time
// Each prompt tells the user exactly which joystick direction to move
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

// Clear every saved button or trigger assignment
// This is used before default assignment, learning mode, and controller reconnection
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

// Load the known button masks for the standardized controller layout
// This path is used only when diagnostic button learning is turned off
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

// Assign a digital button by storing the USB button bit mask
// A mask is a single bit that USBHost_t36 sets when that physical button is pressed
void configureDefaultControllerButton(LearnedControllerButton *button, uint32_t defaultMask) {
  button->mask = defaultMask;
  button->usesAxis = false;
  button->axisIndex = -1;
  button->axisReleased = 0;
  button->axisPressed = 0;
  button->wasPressed = false;
}

// Assign a trigger that reports as an analog axis instead of a digital button
// The released value is measured at startup and the pressed value is estimated from travel
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

// Read one USB axis safely
// Invalid or missing axes return zero so the rest of the program can continue
int32_t readCurrentAxisOrZero(int axisIndex) {
  if (axisIndex < 0 ||
      axisIndex >= maximumJoystickAxes ||
      (joystick.axisMask() & (uint64_t(1) << axisIndex)) == 0) {
    return 0;
  }

  return joystick.getAxis(axisIndex);
}

// Record the current trigger and axis values before button learning waits for a press
// This makes it possible to detect which analog trigger moved later
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

// Find the analog axis that moved the most during button learning
// Motion joystick axes are ignored so they cannot accidentally become trigger commands
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

// Check whether an analog trigger axis has already been assigned to a command
// This prevents two commands from sharing the same trigger by accident
bool axisAlreadyAssignedToControllerButton(int candidateAxisIndex) {
  for (int index = 0; index < buttonSequenceCount; index++) {
    if (buttonSequence[index]->usesAxis &&
        buttonSequence[index]->axisIndex == candidateAxisIndex) {
      return true;
    }
  }

  return false;
}

// Check whether an axis is already being used for joystick motion
// Motion axes are not allowed to also become trigger commands
bool axisIsUsedForMotion(int candidateAxisIndex) {
  return candidateAxisIndex == linearAxis.axisIndex ||
         candidateAxisIndex == rotationAxis.axisIndex ||
         candidateAxisIndex == flexionAxis.axisIndex;
}

// Allow analog-axis learning only for prompts that are expected to be triggers
// Standby and Pause may be analog triggers on some controllers
bool buttonSetupCurrentPromptAcceptsAxis() {
  if (buttonSetupIndex < 0 || buttonSetupIndex >= buttonSequenceCount) {
    return false;
  }

  char command = buttonSequence[buttonSetupIndex]->command;
  return command == 's' || command == 'p';
}

// Check whether the controller has been quiet long enough to move to the next button prompt
bool buttonSetupReleaseComplete(uint32_t currentButtons) {
  return buttonSetupAllLearnedControlsReleased(currentButtons);
}

// Require all learned buttons, triggers, and joystick axes to be released before the next prompt
// This prevents a held button from being accidentally assigned to the next command
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

// Check whether all motion joystick axes are close enough to center during button setup
bool buttonSetupMotionAxesCentered() {
  return buttonSetupJoystickAxisCentered(&linearAxis) &&
         buttonSetupJoystickAxisCentered(&flexionAxis) &&
         buttonSetupJoystickAxisCentered(&rotationAxis);
}

// Check one learned joystick axis against its center value
// This keeps a moved joystick from interfering with button or trigger learning
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

// Tell the user which physical button or trigger to press during learning mode
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

// Begin diagnostic button-learning mode
// Motion and laser commands stay locked until all command buttons have been learned
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

// Watch for the next button or analog trigger during button-learning mode
// The code accepts one clean press, saves it, then waits for release before moving on
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

  // When no buttons are pressed, clear the baseline so the next press is captured cleanly
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

// Finish button-learning mode and unlock normal controller operation
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

// Print all learned or default controller assignments for debugging
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

// Check whether a digital button mask is already used by another command
bool buttonMaskAlreadyAssigned(uint32_t candidateMask) {
  for (int index = 0; index < buttonSequenceCount; index++) {
    if (buttonSequence[index]->mask == candidateMask) {
      return true;
    }
  }

  return false;
}

// Decide whether a learned command button is currently pressed
// Digital buttons use a bit mask and analog triggers use travel past a threshold
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

// Return the lowest set bit from a USB button mask
// This chooses one button if multiple buttons are pressed at once during learning
uint32_t lowestPressedButtonMask(uint32_t buttons) {
  return buttons & (~buttons + 1);
}

// Start the timed window where one joystick axis is moved for calibration
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

// Reset the temporary minimum and maximum values used during axis learning
void clearCalibrationRanges() {
  for (int axis = 0; axis < maximumJoystickAxes; axis++) {
    calibrationMinimum[axis] = INT32_MAX;
    calibrationMaximum[axis] = INT32_MIN;
    calibrationAxisSeen[axis] = false;
  }
}

///---------------------------------------------- Axis Discovery --------------------------------------------------

// Route each incoming joystick packet based on the current setup state
// During setup the packet is used for learning and during normal operation it is used for control
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

// Save the smallest and largest raw value seen for every USB axis during calibration
// The axis with the largest movement is treated as the one the user intentionally moved
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

// Pick the USB axis that moved the most during the current calibration prompt
// This gives the program an axis index, center value, and usable range
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

// Normalize all motion axes so the rest of the program can use values from -1 to +1
void updateAllNormalizedJoystickValues() {
  updateNormalizedJoystickValue(&linearAxis);
  updateNormalizedJoystickValue(&rotationAxis);
  updateNormalizedJoystickValue(&flexionAxis);
}

// Convert one raw USB axis value into a centered joystick value from -1 to +1
// The dead zone removes drift and the direction sign fixes reversed axes
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

// Keep expanding the known range of an axis as real controller values arrive
// This lets default axes self-adjust even without an explicit calibration prompt
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

///------------------------------------------- Joystick Targets ---------------------------------------------------

// Shape joystick input so small movements stay gentle and large movements still reach full speed
float applyJoystickResponseCurve(float value) {
  float cubicValue = value * value * value;
  return 0.35f * value + 0.65f * cubicValue;
}

// Allow horizontal motion only when the paired vertical axis is mostly centered
// This prevents accidental motion when a stick is pushed diagonally
float filterHorizontalAxisOnly(float rawHorizontalValue, float rawVerticalValue, float allowedCrossAxisMotion) {
  float horizontalMagnitude = fabsf(rawHorizontalValue);
  float verticalMagnitude = fabsf(rawVerticalValue);

  if (horizontalMagnitude <= 0.0f) {
    return 0.0f;
  }

  if (verticalMagnitude <= allowedCrossAxisMotion) {
    return rawHorizontalValue;
  }

  // If vertical motion is meaningful, the left stick is diagonal or vertical
  // Diagonal or vertical left-stick input intentionally commands no linear motion
  return 0.0f;
}

// Allow the right stick to command either flexion or rotation, but not both at the same time
// Diagonal right-stick motion is ignored so the two axes do not fight each other
void filterRightStickExclusiveAxes(float rawHorizontalValue, float rawVerticalValue, float *filteredHorizontalValue, float *filteredVerticalValue) {
  float horizontalMagnitude = fabsf(rawHorizontalValue);
  float verticalMagnitude = fabsf(rawVerticalValue);

  // Default to no motion, which makes diagonal commands safe by default
  *filteredHorizontalValue = 0.0f;
  *filteredVerticalValue = 0.0f;

  // If the stick is centered or both axes are tiny, keep both commands stopped
  if (horizontalMagnitude <= 0.0f && verticalMagnitude <= 0.0f) {
    return;
  }

  // Mostly horizontal means flexion only
  // Any meaningful vertical component turns this into a rejected diagonal command
  if (horizontalMagnitude > 0.0f &&
      verticalMagnitude <= rightStickAllowedCrossAxisMotion) {
    *filteredHorizontalValue = rawHorizontalValue;
    return;
  }

  // Mostly vertical means rotation only
  // Any meaningful horizontal component turns this into a rejected diagonal command
  if (verticalMagnitude > 0.0f &&
      horizontalMagnitude <= rightStickAllowedCrossAxisMotion) {
    *filteredVerticalValue = rawVerticalValue;
    return;
  }

  // If both axes are above the cross-axis tolerance, the stick is diagonal
  // Diagonal input intentionally commands neither rotation nor flexion
}

// Convert button edges into commands
// A command runs only when the button first changes from released to pressed
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

// Send a command from either the controller or the Serial Monitor to the correct handler
// HOME and laser commands are kept in one place so their safety rules stay consistent
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

// Drive every physical laser output to its inactive electrical state
void driveLaserOutputsToReleasedState() {
  digitalWrite(fire1_laser, LASER_FIRE_RELEASED);
  digitalWrite(fire2_laser, LASER_FIRE_RELEASED);
  digitalWrite(relay_laser, LASER_RELAY_RELEASED);
  laserFiring = false;
}

// Lock the software laser state so Enable and Ready must happen before Fire
void lockLaserSoftwareState() {
  laserEnabled = false;
  laserReady = false;
  laserFiring = false;
}

// Send the relay/contact pulse used for Ready and Standby transitions
// Motion is held before the pulse because laser control has priority over motion
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

// Release the fire outputs while keeping the enabled and ready state unchanged
// This is used for Pause and as the first step of a full Stop
void pauseLaserFiring() {
  digitalWrite(fire1_laser, LASER_FIRE_RELEASED);
  digitalWrite(fire2_laser, LASER_FIRE_RELEASED);
  laserFiring = false;
  holdCurrentMotionTargets();
  stopAllMotion();
}

// Stop firing, optionally send the ready-clearing pulse, then lock laser software commands
void stopLaserAndRelock(bool sendReadyClearPulse) {
  pauseLaserFiring();

  if (sendReadyClearPulse && laserEnabled && laserReady) {
    sendLaserControlPulse(laserControlPulse_ms);
  }

  driveLaserOutputsToReleasedState();
  lockLaserSoftwareState();
}

// Freeze the current motion targets at the measured positions
// This prevents stored targets from continuing to move after a laser or stop command
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

// Print the compact laser command reference
void printLaserCommandHelp() {
  Serial.println("LASER: Left bumper=Enable/Ready | Right bumper=Fire | Left trigger=Standby | Right trigger=Pause | Bottom face=Stop");
}

// Handle the combined left bumper behavior
// First press unlocks laser commands and second press sends Ready
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

// Enforce the full laser command state machine
// Fire is only allowed after Enable and Ready have both succeeded
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

// Start a coordinated return to the startup home position for all three axes
// HOME uses absolute targets instead of raw joystick speed
void startHomeReturn() {
  homeReturnActive = true;
  stopAllMotion();

  // HOME is now a target-position command instead of a raw speed command
  // The position controller will decide the safe motor speeds needed to reach these targets
  targetLinearPositionMM = homeLinearPositionMM;
  targetRotationUreteroscopeDeg = homeRotationUreteroscopeDeg;
  targetTipDeg = homeFlexionTipDeg;
  updateAbsoluteTargetDerivedNumbers();

  Serial.println("\nHOME button pressed");
  Serial.println("Absolute targets set to Linear 0 mm Rotation 0 deg Flexion 0 deg");
  Serial.println("Returning linear rotation and flexion to startup positions");
}

// Keep HOME targets fixed until all axes are close enough to home
// When HOME finishes, the absolute counters are reset to the startup reference
void updateHomeReturnTargets() {
  // Keep all three absolute targets pinned to the startup home positions
  // This means joystick movement is ignored while HOME is active
  targetLinearPositionMM = homeLinearPositionMM;
  targetRotationUreteroscopeDeg = homeRotationUreteroscopeDeg;
  targetTipDeg = homeFlexionTipDeg;
  targetFlexionSpeedDegPerSec = 0.0f;
  requestedLinearTargetRateMMPerSec = 0.0f;
  requestedRotationTargetRateDegPerSec = 0.0f;
  requestedFlexionTargetRateDegPerSec = 0.0f;
  updateAbsoluteTargetDerivedNumbers();

  // HOME detection for linear is based on the measured encoder-derived absolute position
  // If the carriage is already inside the home zone, stop trying to push farther into 0 mm
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

// Check whether linear, rotation, and flexion are all inside their home tolerances
bool homeReturnIsFinished() {
  float flexionHomeErrorDeg =
    currentFlexionStep >= 0
      ? measuredTipDeg - homeFlexionTipDeg
      : targetTipDeg - homeFlexionTipDeg;

  return linearIsInsideHomeTolerance() &&
         fabsf(currentRotationUreteroscopeDeg - homeRotationUreteroscopeDeg) <= homeRotationToleranceDeg &&
         fabsf(flexionHomeErrorDeg) <= homeFlexionToleranceDeg;
}

// Check whether the measured linear position is close enough to count as home
bool linearIsInsideHomeTolerance() {
  // Use encoder-derived currentLinearPositionMM instead of motor speed
  // Speed only says whether we are moving
  // Encoder-derived position is the best available estimate of where the linear axis actually is
  return fabsf(currentLinearPositionMM - homeLinearPositionMM) <= homeLinearToleranceMM;
}

// Check whether HOME is actively trying to move the linear carriage toward the lower limit
// This is used by the automatic home reference reset protection
bool linearIsBeingCommandedTowardHome() {
  bool targetIsHome =
    fabsf(targetLinearPositionMM - homeLinearPositionMM) <= homeLinearToleranceMM;

  bool movingTowardLowerLimit =
    targetLinearSpeedMMPerSec < -stoppedLinearSpeedToleranceMMPerSec ||
    currentLinearSpeedMMPerSec < -stoppedLinearSpeedToleranceMMPerSec;

  return homeReturnActive && targetIsHome && movingTowardLowerLimit;
}

// Redefine the current measured linear location as 0 mm
// This is only used when the carriage is already inside the HOME tolerance
void redefineLinearHomePosition(const char *reason) {
  stopLinearImmediately();

  // Reset the measured linear reference
  // currentLinearPositionMM is the best available absolute linear estimate because it comes from encoder counts
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

// During HOME, stop pushing into the lower limit if encoder motion stops inside the home zone
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

// Convert normalized joystick values into requested target rates for each axis
// Safety locks force all requested speeds to zero during faults or laser firing
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

  // Match the working standardized-axis sketch:
  // normal joystick control commands linear speed directly instead of nudging
  // an absolute target that a second controller has to chase
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

  // Use the unwrapped measured angle so the USB target system cannot rotate past the VR range
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

///------------------------------------- Absolute Target Position Layer ------------------------------------------

// Integrate requested rates into absolute target positions for linear and rotation
// Linear is held to measured position during normal direct-speed control
void updateAbsoluteLinearAndRotationTargets() {
  unsigned long now_us = micros();
  unsigned long elapsed_us = now_us - lastAbsoluteTargetUpdate_us;
  lastAbsoluteTargetUpdate_us = now_us;

  // Limit the elapsed time so a USB pause cannot create one huge target jump
  if (elapsed_us > 50000) {
    elapsed_us = 50000;
  }

  float elapsedSeconds = (float)elapsed_us / 1000000.0f;

  if (!controlsReady || systemFaultLatched || laserFiring) {
    return;
  }

  // During HOME, the target positions are pinned by updateHomeReturnTargets
  // That means joystick motion cannot drag the targets away while homing
  if (!homeReturnActive) {
    // Linear no longer uses a joystick-built absolute target during normal
    // control. Keep the printed target aligned with the measured position
    targetLinearPositionMM = currentLinearPositionMM;

    // Convert right-joystick vertical requested speed into an unwrapped absolute rotation target
    // This target does not wrap, so HOME can unwind the rotation back to the true startup position
    targetRotationUreteroscopeDeg += requestedRotationTargetRateDegPerSec * elapsedSeconds;

    // Clamp the accumulated rotation target to the same range used by the VR Teensy sketch
    targetRotationUreteroscopeDeg = clampFloat(
      targetRotationUreteroscopeDeg,
      rotationMinPositionDeg,
      rotationMaxPositionDeg
    );
  }

  snapAbsoluteTargetsNearLimits();
  updateAbsoluteTargetDerivedNumbers();
}

// Recalculate derived motor-side values after a target changes
// These values are used for step estimates, status printing, and flexion servo commands
void updateAbsoluteTargetDerivedNumbers() {
  // Linear motor target numbers derived from absolute carriage millimeters
  targetLinearMotorRevolutions = targetLinearPositionMM / linearMMPerRev;
  targetLinearMotorStepEstimate = estimateLinearMotorStepsFromMM(targetLinearPositionMM);

  // Rotation motor target numbers derived from absolute ureteroscope rotation
  targetRotationMotorDeg = targetRotationUreteroscopeDeg * rotationGearRatio;
  targetRotationMapDeg = normalizeRotationMapDeg(targetRotationUreteroscopeDeg);
  targetRotationMotorStepEstimate = estimateRotationMotorStepsFromDeg(targetRotationMotorDeg);

  // Flexion motor target numbers derived from absolute ureteroscope tip angle
  int calculatedStep = flexionNeutralStep;
  float calculatedMotorDeg = 0.0f;

  if (calculateFlexionStepFromTip(targetTipDeg, &calculatedStep, &calculatedMotorDeg)) {
    targetFlexionMotorDeg = calculatedMotorDeg;
    targetFlexionPhysicalDeg = targetFlexionMotorDeg + halfRevolutionDegrees;
    targetFlexionStep = calculatedStep;
  }
}

// Convert absolute position errors into speed commands for the axes that chase targets
// Rotation always chases its target and linear only chases during HOME
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

    // During HOME only, the linear motor chases the absolute home target
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

  // The rotation motor chases the unwrapped absolute ureteroscope rotation target
  // This means HOME returns the actual accumulated rotation back to startup 0 degrees
  float rotationErrorDeg = targetRotationUreteroscopeDeg - currentRotationUreteroscopeDeg;
  targetRotationSpeedDegPerSec = calculatePositionControllerSpeed(
    rotationErrorDeg,
    rotationPositionTargetToleranceDeg,
    rotationPositionControllerGain,
    maximumRotationSpeedDegPerSec
  );
}

// Turn a position error into a limited speed
// Small errors inside tolerance produce zero speed so the axis can settle
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

// Keep a floating-point value inside a minimum and maximum range
float clampFloat(float value, float minimumValue, float maximumValue) {
  if (value < minimumValue) {
    return minimumValue;
  }

  if (value > maximumValue) {
    return maximumValue;
  }

  return value;
}

// Clean up tiny leftover target values near home and travel limits
// This prevents floating-point leftovers from causing slow unwanted motion
void snapAbsoluteTargetsNearLimits() {
  bool linearTargetIsBeingMoved =
    fabsf(requestedLinearTargetRateMMPerSec) >= stoppedLinearSpeedToleranceMMPerSec;

  bool rotationTargetIsBeingMoved =
    fabsf(requestedRotationTargetRateDegPerSec) >= stoppedRotationSpeedToleranceDegPerSec;

  bool flexionTargetIsBeingMoved =
    fabsf(requestedFlexionTargetRateDegPerSec) >= stoppedFlexionSpeedToleranceDegPerSec;

  // Snap only when the operator is not actively moving away from the nearby value
  // This avoids the annoying case where a very small joystick command cannot leave 0
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

// Convert linear carriage position in millimeters into an estimated motor step count
long estimateLinearMotorStepsFromMM(float positionMM) {
  float motorRevolutions = positionMM / linearMMPerRev;
  return lroundf(motorRevolutions * linearPulsesPerRev);
}

// Convert rotation motor angle in degrees into an estimated motor step count
long estimateRotationMotorStepsFromDeg(float motorDeg) {
  return lroundf((motorDeg / degreesPerRevolution) * rotationPulsesPerRev);
}

///------------------------------------------- Velocity Controller -----------------------------------------------

// Smooth each target speed into a current speed using acceleration and deceleration limits
// This keeps motor motion from changing too abruptly
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

// Move one value toward another by no more than maximumChange
// This small helper is used for both axis speeds and servo speed smoothing
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

// Choose whether to use acceleration or deceleration based on speed direction and magnitude
float chooseRampRate(float currentValue, float targetValue, float acceleration, float deceleration) {
  bool sameDirection =
    (targetValue >= 0.0f && currentValue >= 0.0f) ||
    (targetValue <= 0.0f && currentValue <= 0.0f);

  bool increasingMagnitude = fabsf(targetValue) > fabsf(currentValue);

  return sameDirection && increasingMagnitude ? acceleration : deceleration;
}

///------------------------------------------ Nonblocking STEP Output ---------------------------------------------

// Generate STEP pulses for the linear motor without using delay
// The function returns quickly so USB, encoders, laser safety, and status updates keep running
void updateLinearStepperPulseOutput() {
  unsigned long now_us = micros();

  // End a pulse after the required HIGH time
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

  // Change direction only while STEP is LOW, then wait for setup time before the next pulse
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

  // Convert desired carriage speed into step frequency for the TB6600 driver
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

// Generate STEP pulses for the rotation motor without blocking the main loop
// Rotation speed is converted through the gear ratio before calculating pulses per second
void updateRotationStepperPulseOutput() {
  unsigned long now_us = micros();

  // End a pulse after the required HIGH time
  if (rotationStepPulseIsHigh && now_us - rotationStepPulseHighStart_us >= stepPulseHighTime_us) {
    digitalWrite(rotation_stepPin, LOW);
    rotationStepPulseIsHigh = false;
  }

  if (systemFaultLatched ||
      fabsf(currentRotationSpeedDegPerSec) < stoppedRotationSpeedToleranceDegPerSec) {
    return;
  }

  // Stop any pulse request that would push beyond the VR-matched rotation range
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

  // Change direction only while STEP is LOW, then wait for setup time before the next pulse
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

  // Convert ureteroscope speed into motor-side speed because the encoder and motor are geared
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
  // Match the working standardized-axis sketch: only block negative motion
  // when the measured linear position is actually at the lower travel limit
  if (speedMMPerSec < 0.0f &&
      currentLinearPositionMM <= linearMinPositionMM + linearRangeToleranceMM) {
    return true;
  }

  // The maximum side remains strict so the program never commands beyond 177 mm
  if (speedMMPerSec > 0.0f &&
      currentLinearPositionMM >= linearMaxPositionMM - linearRangeToleranceMM) {
    return true;
  }

  return false;
}

// Stop linear step output immediately and clear the stored active direction
void stopLinearImmediately() {
  targetLinearSpeedMMPerSec = 0.0f;
  currentLinearSpeedMMPerSec = 0.0f;
  digitalWrite(linear_stepPin, LOW);
  linearStepPulseIsHigh = false;
  linearActiveDirectionSign = 0;
}

// Stop rotation step output immediately and clear the stored active direction
void stopRotationImmediately() {
  targetRotationSpeedDegPerSec = 0.0f;
  currentRotationSpeedDegPerSec = 0.0f;
  digitalWrite(rotation_stepPin, LOW);
  rotationStepPulseIsHigh = false;
  rotationActiveDirectionSign = 0;
}

// Stop every motion command and clear joystick values
// This is used by faults, laser commands, disconnection, and setup transitions
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

///------------------------------------------- Encoder Feedback --------------------------------------------------

// Periodically convert encoder counts into measured linear and rotation positions
// This function is the main feedback source for HOME, limits, status, and stall detection
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

  // Convert the tracked carriage position back into motor-side numbers for printing and debugging
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

  // Convert the unwrapped rotation motor angle into an estimated absolute motor step position
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

// Watch for commanded motor motion without matching encoder motion
// A stall latches a fault so the system stops instead of continuing blindly
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

// Latch a fault, stop all motion, disable stepper drivers, and ask for a restart
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

///--------------------------------------------- Flexion Control --------------------------------------------------

// Update the absolute flexion target angle from the smoothed flexion speed
// The target is clamped so it cannot move beyond the allowed tip-angle range
void updateFlexionTargetAngle() {
  unsigned long now_us = micros();
  unsigned long elapsed_us = now_us - lastFlexionTargetUpdate_us;
  lastFlexionTargetUpdate_us = now_us;

  if (elapsed_us > 50000) {
    elapsed_us = 50000;
  }

  float elapsedSeconds = (float)elapsed_us / 1000000.0f;

  if (!homeReturnActive && !laserFiring) {
    // Convert right-joystick horizontal motion into an absolute tip-angle target
    // Releasing the joystick leaves this target where it is
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

// Choose the flexion twist factor for the requested tip angle
// Blended transitions prevent sudden behavior changes near 90 and 180 degrees
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

// Choose the tip gain that converts motor degrees into estimated tip degrees
// This mirrors the twist-factor selection so measured flexion stays consistent
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

// Smoothly map a value from 0 to 1 using an ease-in and ease-out curve
// This is used to blend calibration values near range boundaries
float smoothStep01(float value) {
  if (value <= 0.0f) {
    return 0.0f;
  }

  if (value >= 1.0f) {
    return 1.0f;
  }

  return value * value * (3.0f - 2.0f * value);
}

// Linearly blend between two floating-point values
float blendFloat(float firstValue, float secondValue, float blendAmount) {
  return firstValue + (secondValue - firstValue) * blendAmount;
}

// Slow flexion motion as the target approaches the allowed mechanical limits
// This makes the limit feel gradual instead of abruptly clipping the command
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

// Convert requested tip angle into an ST3020 step position
// The conversion checks both tip-angle limits and motor-angle limits
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

// Reduce ST3020 command speed as flexion moves farther from neutral
// This helps keep high flexion movements gentler
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

// Smooth changes in the ST3020 speed command itself
// This prevents sudden jumps when the selected flexion range or slowdown level changes
int updateSmoothedFlexionServoSpeed(int desiredServoSpeed) {
  unsigned long now_ms = millis();
  unsigned long elapsed_ms = now_ms - lastServoSpeedRampTime_ms;
  lastServoSpeedRampTime_ms = now_ms;

  // Cap the elapsed time so a pause or USB hiccup cannot create one large speed jump
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

// Send a new ST3020 position command when enough time has passed
// The command includes the target step, selected speed, and acceleration value
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

// Read the current ST3020 position when enough time has passed
// The measured servo step is converted back into motor degrees and estimated tip degrees
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

// Print a warning only when the flexion warning level changes
// This keeps the Serial Monitor readable while still showing important caution messages
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

///--------------------------------------------- Status Printing --------------------------------------------------

// Print a compact live status line at a fixed interval
// Status prints are skipped during setup and while a STEP pulse is HIGH
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

///--------------------------------------------- Encoder Functions -----------------------------------------------

// Keep the printed rotation map inside one positive or negative revolution
// The unwrapped rotation value is still kept separately for accurate HOME return
float normalizeRotationMapDeg(float angleDeg) {
  while (angleDeg >= degreesPerRevolution) {
    angleDeg -= degreesPerRevolution;
  }

  while (angleDeg <= -degreesPerRevolution) {
    angleDeg += degreesPerRevolution;
  }

  return angleDeg;
}

// Count one linear encoder channel A rising edge
// Channel B decides whether the count increases or decreases
void linearEncoderISR() {
  if (digitalRead(linear_encB) == HIGH) {
    linearEncoderCount++;
  } else {
    linearEncoderCount--;
  }
}

// Count one rotation encoder channel A rising edge
// Channel B decides whether the count increases or decreases
void rotationEncoderISR() {
  if (digitalRead(rotation_encB) == HIGH) {
    rotationEncoderCount++;
  } else {
    rotationEncoderCount--;
  }
}

// Safely copy the linear encoder count while interrupts are paused briefly
long getLinearEncoderCount() {
  noInterrupts();
  long count = linearEncoderCount;
  interrupts();
  return count;
}

// Safely copy the rotation encoder count while interrupts are paused briefly
long getRotationEncoderCount() {
  noInterrupts();
  long count = rotationEncoderCount;
  interrupts();
  return count;
}

// Reset the linear encoder count and the last processed signed count together
void resetLinearEncoder() {
  noInterrupts();
  linearEncoderCount = 0;
  interrupts();
  lastProcessedLinearSignedCount = 0;
}

// Reset the rotation encoder count and the last processed signed count together
void resetRotationEncoder() {
  noInterrupts();
  rotationEncoderCount = 0;
  interrupts();
  lastProcessedRotationSignedCount = 0;
}
