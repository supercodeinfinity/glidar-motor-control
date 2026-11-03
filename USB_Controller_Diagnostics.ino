/*
GLiDAR USB Controller Diagnostics
Description: Plug in a USB controller and check its readings in Serial Monitor
             See which buttons, joystick directions, and triggers the Teensy picks up
             This only prints readings and does not send motor or laser commands
Components: Teensy 4.1, USB game controller, computer with Serial Monitor
*/

#include <USBHost_t36.h>
#include <stdint.h>
#include <stdlib.h>

// Set up the USB host connection so the Teensy can read the controller
USBHost usbHost;
USBHub usbHub(usbHost);
USBHIDParser hidParser(usbHost);
JoystickController joystick(usbHost);

// Save readings for up to 64 axes and show the status every three seconds
const int maximumJoystickAxes = 64;
const unsigned long statusPrintInterval_ms = 3000;

// Skip tiny changes in the raw readings so Serial Monitor is easier to follow
const int32_t rawAxisPrintThreshold = 25;

// Ignore small changes near the stick center so they do not look like movement
const int32_t minimumLiveAxisHalfRange = 100;
const int32_t minimumAxisDeadband = 25;
const float axisMovedFraction = 0.25f;

// Use the same button numbers as the main controller sketch
// Each 1 in the bit pattern marks a button; this sketch only prints its name
const uint32_t homeButtonMask = 0x00000002;  // B
const uint32_t enableReadyButtonMask = 0x00000010;  // Left bumper
const uint32_t standbyButtonMask = 0x00000040;  // Left trigger
const uint32_t fireButtonMask = 0x00000020;  // Right bumper
const uint32_t pauseButtonMask = 0x00000080;  // Right trigger
const uint32_t stopButtonMask = 0x00000004;  // Bottom face button

// Use the axis numbers from the main controller sketch
const int leftJoystickXAxisIndex = 0;
const int leftJoystickYAxisIndex = 1;
const int rightJoystickXAxisIndex = 2;
const int leftTriggerAxisIndex = 3;
const int rightTriggerAxisIndex = 4;
const int rightJoystickYAxisIndex = 5;

// Remember which buttons were pressed in the last reading
struct ButtonDiagnostic {
  const char *name;
  uint32_t mask;
  bool wasPressed;
};

// Save each axis name, center, range, and last direction
struct AxisDiagnostic {
  const char *name;
  int axisIndex;
  int directionSign;
  const char *negativeText;
  const char *positiveText;
  bool initialized;
  int32_t center;
  int32_t minimum;
  int32_t maximum;
  int32_t lastRaw;
  int state;
};

ButtonDiagnostic buttonDiagnostics[] = {
  { "B / Home button", homeButtonMask, false },
  { "Left bumper / Enable-Ready button", enableReadyButtonMask, false },
  { "Left trigger / Standby button", standbyButtonMask, false },
  { "Right bumper / Fire button", fireButtonMask, false },
  { "Right trigger / Pause button", pauseButtonMask, false },
  { "Bottom face / Stop-Exit button", stopButtonMask, false }
};

const int buttonDiagnosticCount =
  sizeof(buttonDiagnostics) / sizeof(buttonDiagnostics[0]);

// Use these signs so up and down print the right way around
AxisDiagnostic axisDiagnostics[] = {
  { "Left joystick X", leftJoystickXAxisIndex, +1, "left", "right", false, 0, 0, 0, 0, 0 },
  { "Left joystick Y", leftJoystickYAxisIndex, -1, "down", "up", false, 0, 0, 0, 0, 0 },
  { "Right joystick X", rightJoystickXAxisIndex, +1, "left", "right", false, 0, 0, 0, 0, 0 },
  { "Left trigger analog axis", leftTriggerAxisIndex, +1, "negative", "pressed/increased", false, 0, 0, 0, 0, 0 },
  { "Right trigger analog axis", rightTriggerAxisIndex, +1, "negative", "pressed/increased", false, 0, 0, 0, 0, 0 },
  { "Right joystick Y", rightJoystickYAxisIndex, -1, "down", "up", false, 0, 0, 0, 0, 0 }
};

const int axisDiagnosticCount =
  sizeof(axisDiagnostics) / sizeof(axisDiagnostics[0]);

// Save the last readings so only new changes get printed
bool controllerWasConnected = false;
uint32_t lastButtonMask = 0;
uint64_t lastAxisMask = 0;
int32_t lastRawAxis[maximumJoystickAxes];
bool rawAxisSeen[maximumJoystickAxes];
unsigned long connectedAt_ms = 0;
unsigned long lastPacket_ms = 0;
unsigned long lastStatusPrint_ms = 0;
uint32_t packetCount = 0;

void clearRawAxisHistory();
void resetDiagnostics();
void printStartupMessage();
void printControllerConnection();
void printControllerDisconnection();
void printStatusIfReady();
void processJoystickPacket();
void printButtonChanges(uint32_t currentButtons);
void printNamedButtonEvents(uint32_t currentButtons);
void printAxisMaskChange(uint64_t axisMask);
void printNamedAxisEvents(uint64_t axisMask);
void printRawAxisChanges(uint64_t axisMask, uint64_t changedAxisMask);
void updateControllerConnection();
void printButtonAssignments();
void printAxisAssignments();
void printTextIfAvailable(const char *label, const uint8_t *text);
void printHex64(uint64_t value);
void printPaddedHex32(uint32_t value);
bool axisIsPresent(uint64_t axisMask, int axisIndex);
int32_t largerInt32(int32_t firstValue, int32_t secondValue);
float clampFloat(float value, float minimumValue, float maximumValue);
const char *joystickTypeName(JoystickController::joytype_t joystickType);

void setup() {
  Serial.begin(115200);

  // Wait up to four seconds for Serial Monitor, then keep going
  unsigned long serialWaitStart_ms = millis();
  while (!Serial && millis() - serialWaitStart_ms < 4000) {
  }

  clearRawAxisHistory();
  usbHost.begin();
  // Ask the USB library to report changes from any of the 64 axes
  joystick.axisChangeNotifyMask((uint64_t)-1);

  printStartupMessage();
}

// Keep checking USB and read each new set of controller values
void loop() {
  usbHost.Task();
  updateControllerConnection();

  if (joystick && joystick.available()) {
    processJoystickPacket();
    joystick.joystickDataClear();
  }

  printStatusIfReady();
}

void clearRawAxisHistory() {
  for (int axis = 0; axis < maximumJoystickAxes; axis++) {
    lastRawAxis[axis] = 0;
    rawAxisSeen[axis] = false;
  }
}

// Clear the old button and joystick readings when the connection changes
void resetDiagnostics() {
  lastButtonMask = 0;
  lastAxisMask = 0;
  packetCount = 0;
  lastPacket_ms = 0;
  clearRawAxisHistory();

  for (int index = 0; index < buttonDiagnosticCount; index++) {
    buttonDiagnostics[index].wasPressed = false;
  }

  for (int index = 0; index < axisDiagnosticCount; index++) {
    axisDiagnostics[index].initialized = false;
    axisDiagnostics[index].center = 0;
    axisDiagnostics[index].minimum = 0;
    axisDiagnostics[index].maximum = 0;
    axisDiagnostics[index].lastRaw = 0;
    axisDiagnostics[index].state = 0;
  }
}

void printStartupMessage() {
  Serial.println();
  Serial.println("============================================================");
  Serial.println("        TEENSY 4.1 USB CONTROLLER SERIAL DIAGNOSTICS");
  Serial.println("============================================================");
  Serial.println("This sketch prints controller input only.");
  Serial.println("No motor, servo, relay, laser, or fire output pins are used.");
  Serial.println();
  Serial.println("Open Serial Monitor at 115200 baud.");
  Serial.println("Plug the controller into the Teensy USB host port.");
  Serial.println();
  printButtonAssignments();
  printAxisAssignments();
  Serial.println("Waiting for controller...");
  Serial.println("============================================================");
}

// Print once when the controller connects or disconnects
void updateControllerConnection() {
  bool controllerIsConnected = (bool)joystick;

  if (controllerIsConnected && !controllerWasConnected) {
    controllerWasConnected = true;
    connectedAt_ms = millis();
    lastStatusPrint_ms = 0;
    resetDiagnostics();
    printControllerConnection();
  }

  if (!controllerIsConnected && controllerWasConnected) {
    controllerWasConnected = false;
    printControllerDisconnection();
    resetDiagnostics();
  }
}

// Show the controller's USB ID numbers and name so it is easier to identify
void printControllerConnection() {
  Serial.println();
  Serial.println("CONTROLLER CONNECTED");
  Serial.print("  VID:PID: 0x");
  Serial.print(joystick.idVendor(), HEX);
  Serial.print(":0x");
  Serial.println(joystick.idProduct(), HEX);

  printTextIfAvailable("  Manufacturer: ", joystick.manufacturer());
  printTextIfAvailable("  Product: ", joystick.product());
  printTextIfAvailable("  Serial number: ", joystick.serialNumber());

  Serial.print("  Joystick type: ");
  Serial.print(joystickTypeName(joystick.joystickType()));
  Serial.print(" (");
  Serial.print((int)joystick.joystickType());
  Serial.println(")");

  Serial.print("  Report ID: ");
  Serial.println(joystick.getReportID());
  Serial.println("Move sticks or press buttons to see events.");
  Serial.println();
}

void printControllerDisconnection() {
  Serial.println();
  Serial.println("CONTROLLER DISCONNECTED");
  Serial.println("Waiting for controller...");
  Serial.println();
}

void printStatusIfReady() {
  if (millis() - lastStatusPrint_ms < statusPrintInterval_ms) {
    return;
  }

  lastStatusPrint_ms = millis();

  if (!controllerWasConnected) {
    Serial.println("STATUS: waiting for controller");
    return;
  }

  Serial.print("STATUS: connected ");
  Serial.print((millis() - connectedAt_ms) / 1000);
  Serial.print(" s | packets ");
  Serial.print(packetCount);

  Serial.print(" | buttons 0x");
  Serial.print(lastButtonMask, HEX);

  Serial.print(" | axes ");
  printHex64(lastAxisMask);

  Serial.print(" | last packet ");
  if (lastPacket_ms == 0) {
    Serial.println("not received yet");
  } else {
    Serial.print(millis() - lastPacket_ms);
    Serial.println(" ms ago");
  }
}

// Compare the new readings with the last ones and print what changed
void processJoystickPacket() {
  packetCount++;
  lastPacket_ms = millis();

  uint32_t currentButtons = joystick.getButtons();
  uint64_t axisMask = joystick.axisMask();
  uint64_t changedAxisMask = joystick.axisChangedMask();

  printAxisMaskChange(axisMask);
  printNamedButtonEvents(currentButtons);
  printButtonChanges(currentButtons);
  printNamedAxisEvents(axisMask);
  printRawAxisChanges(axisMask, changedAxisMask);

  lastButtonMask = currentButtons;
  lastAxisMask = axisMask;
}

void printButtonChanges(uint32_t currentButtons) {
  // XOR compares the button bits and leaves a 1 where a button changed
  uint32_t changedButtons = currentButtons ^ lastButtonMask;

  if (changedButtons == 0) {
    return;
  }

  Serial.print("RAW BUTTON MASK: 0x");
  Serial.print(lastButtonMask, HEX);
  Serial.print(" -> 0x");
  Serial.println(currentButtons, HEX);

  for (int bit = 0; bit < 32; bit++) {
    uint32_t bitMask = (uint32_t)1 << bit;

    if ((changedButtons & bitMask) == 0) {
      continue;
    }

    Serial.print("  Raw button bit 0x");
    Serial.print(bitMask, HEX);
    Serial.println((currentButtons & bitMask) ? " pressed" : " released");
  }
}

// Find the button name and print whether it was pressed or released
void printNamedButtonEvents(uint32_t currentButtons) {
  for (int index = 0; index < buttonDiagnosticCount; index++) {
    ButtonDiagnostic *button = &buttonDiagnostics[index];
    bool isPressed = (currentButtons & button->mask) != 0;

    if (isPressed == button->wasPressed) {
      continue;
    }

    Serial.print(isPressed ? "PRESSED: " : "RELEASED: ");
    Serial.print(button->name);
    Serial.print(" | mask 0x");
    Serial.println(button->mask, HEX);

    button->wasPressed = isPressed;
  }
}

void printAxisMaskChange(uint64_t axisMask) {
  if (axisMask == lastAxisMask) {
    return;
  }

  Serial.print("AXIS MASK: ");
  printHex64(lastAxisMask);
  Serial.print(" -> ");
  printHex64(axisMask);
  Serial.println();
}

void printNamedAxisEvents(uint64_t axisMask) {
  for (int index = 0; index < axisDiagnosticCount; index++) {
    AxisDiagnostic *axis = &axisDiagnostics[index];

    if (!axisIsPresent(axisMask, axis->axisIndex)) {
      if (axis->initialized) {
        Serial.print("MISSING: ");
        Serial.print(axis->name);
        Serial.print(" | expected axis ");
        Serial.println(axis->axisIndex);
      }

      axis->initialized = false;
      axis->state = 0;
      continue;
    }

    int32_t raw = joystick.getAxis(axis->axisIndex);

    // Use the first reading as the center, so let go of the controls when connecting
    if (!axis->initialized) {
      axis->initialized = true;
      axis->center = raw;
      axis->minimum = raw;
      axis->maximum = raw;
      axis->lastRaw = raw;
      axis->state = 0;

      Serial.print("AXIS READY: ");
      Serial.print(axis->name);
      Serial.print(" | axis ");
      Serial.print(axis->axisIndex);
      Serial.print(" | center raw ");
      Serial.println(raw);
      continue;
    }

    if (raw < axis->minimum) axis->minimum = raw;
    if (raw > axis->maximum) axis->maximum = raw;

    // Use the biggest readings seen so far to estimate the joystick range
    int32_t positiveHalfRange = axis->maximum - axis->center;
    int32_t negativeHalfRange = axis->center - axis->minimum;
    int32_t halfRange = largerInt32(positiveHalfRange, negativeHalfRange);

    if (halfRange < minimumLiveAxisHalfRange) {
      halfRange = minimumLiveAxisHalfRange;
    }

    // Ignore small movement near center, then work out which direction to print
    int32_t deadband = (int32_t)((float)halfRange * axisMovedFraction);
    if (deadband < minimumAxisDeadband) {
      deadband = minimumAxisDeadband;
    }

    int32_t signedDelta = (raw - axis->center) * axis->directionSign;
    int newState = 0;

    if (signedDelta > deadband) {
      newState = +1;
    } else if (signedDelta < -deadband) {
      newState = -1;
    }

    // Print when the stick changes direction or comes back to center
    if (newState != axis->state) {
      float normalized = (float)signedDelta / (float)halfRange;
      normalized = clampFloat(normalized, -1.0f, 1.0f);

      if (newState == 0) {
        Serial.print("CENTERED: ");
        Serial.print(axis->name);
      } else {
        Serial.print("MOVED: ");
        Serial.print(axis->name);
        Serial.print(" ");
        Serial.print(newState > 0 ? axis->positiveText : axis->negativeText);
      }

      Serial.print(" | axis ");
      Serial.print(axis->axisIndex);
      Serial.print(" | raw ");
      Serial.print(raw);
      Serial.print(" | normalized ");
      Serial.println(normalized, 2);

      axis->state = newState;
    }

    axis->lastRaw = raw;
  }
}

// Show the raw readings too, including axes that do not have names above
void printRawAxisChanges(uint64_t axisMask, uint64_t changedAxisMask) {
  if (changedAxisMask == 0) {
    return;
  }

  for (int axis = 0; axis < maximumJoystickAxes; axis++) {
    if (!axisIsPresent(axisMask, axis)) {
      rawAxisSeen[axis] = false;
      continue;
    }

    uint64_t axisBit = (uint64_t)1 << axis;
    if ((changedAxisMask & axisBit) == 0) {
      continue;
    }

    int32_t raw = joystick.getAxis(axis);

    if (!rawAxisSeen[axis]) {
      rawAxisSeen[axis] = true;
      lastRawAxis[axis] = raw;
      Serial.print("RAW AXIS ");
      Serial.print(axis);
      Serial.print(" first value ");
      Serial.println(raw);
      continue;
    }

    if (labs(raw - lastRawAxis[axis]) >= rawAxisPrintThreshold) {
      Serial.print("RAW AXIS ");
      Serial.print(axis);
      Serial.print(": ");
      Serial.print(lastRawAxis[axis]);
      Serial.print(" -> ");
      Serial.println(raw);
      lastRawAxis[axis] = raw;
    }
  }
}

void printButtonAssignments() {
  Serial.println("Named button diagnostics:");
  for (int index = 0; index < buttonDiagnosticCount; index++) {
    Serial.print("  ");
    Serial.print(buttonDiagnostics[index].name);
    Serial.print(" = 0x");
    Serial.println(buttonDiagnostics[index].mask, HEX);
  }
  Serial.println();
}

void printAxisAssignments() {
  Serial.println("Named axis diagnostics:");
  for (int index = 0; index < axisDiagnosticCount; index++) {
    Serial.print("  ");
    Serial.print(axisDiagnostics[index].name);
    Serial.print(" = axis ");
    Serial.println(axisDiagnostics[index].axisIndex);
  }
  Serial.println();
}

void printTextIfAvailable(const char *label, const uint8_t *text) {
  if (text == nullptr || text[0] == 0) {
    return;
  }

  Serial.print(label);
  Serial.println((const char *)text);
}

// Split a 64-bit mask into two 32-bit parts so it can be printed
void printHex64(uint64_t value) {
  uint32_t highWord = (uint32_t)(value >> 32);
  uint32_t lowWord = (uint32_t)(value & 0xFFFFFFFF);

  Serial.print("0x");

  if (highWord != 0) {
    Serial.print(highWord, HEX);
    printPaddedHex32(lowWord);
  } else {
    Serial.print(lowWord, HEX);
  }
}

// Add zeros at the front so the lower half always prints as eight digits
void printPaddedHex32(uint32_t value) {
  if (value < 0x10000000UL) Serial.print("0");
  if (value < 0x01000000UL) Serial.print("0");
  if (value < 0x00100000UL) Serial.print("0");
  if (value < 0x00010000UL) Serial.print("0");
  if (value < 0x00001000UL) Serial.print("0");
  if (value < 0x00000100UL) Serial.print("0");
  if (value < 0x00000010UL) Serial.print("0");
  Serial.print(value, HEX);
}

// Check that the axis number is valid, then see if its bit is set
bool axisIsPresent(uint64_t axisMask, int axisIndex) {
  if (axisIndex < 0 || axisIndex >= maximumJoystickAxes) {
    return false;
  }

  return (axisMask & ((uint64_t)1 << axisIndex)) != 0;
}

int32_t largerInt32(int32_t firstValue, int32_t secondValue) {
  return firstValue > secondValue ? firstValue : secondValue;
}

float clampFloat(float value, float minimumValue, float maximumValue) {
  if (value < minimumValue) return minimumValue;
  if (value > maximumValue) return maximumValue;
  return value;
}

const char *joystickTypeName(JoystickController::joytype_t joystickType) {
  switch (joystickType) {
    case JoystickController::PS3:
      return "PS3";
    case JoystickController::PS4:
      return "PS4";
    case JoystickController::XBOXONE:
      return "Xbox One";
    case JoystickController::XBOX360:
      return "Xbox 360";
    case JoystickController::PS3_MOTION:
      return "PS3 Motion";
    case JoystickController::SpaceNav:
      return "SpaceNav";
    case JoystickController::SWITCH:
      return "Nintendo Switch";
    default:
      return "Unknown / generic HID";
  }
}
