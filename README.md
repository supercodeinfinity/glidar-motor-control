# GLiDAR — Selected Control Software

**GLiDAR: Guided Lithotripsy with Dynamic Autonomous Robot**

This repository highlights selected programming contributions from my internship on the University of Miami's GLiDAR medical robotics project. My work focused on bringing ureteroscope motion control, USB game-controller input, position feedback, diagnostics, and laser-interface commands together on a Teensy 4.1.

I also worked on the electronics and wiring that connected the software to the physical platform. These four sketches show different ways of controlling and checking the system. They are selected parts of my contribution to a larger team project.

## System demonstration

**Video:** [GOOGLE DRIVE VIDEO LINK — replace this placeholder before sharing]

*Complete GLiDAR robotic platform showing the motorized ureteroscope system, controller interface, electronics, and laser integration.*

The video shows the team's complete platform. The code below focuses on my control-software contributions.

## Included programs

Each folder contains a separate Arduino/Teensy sketch with a matching `.ino` filename.

| Program | What it demonstrates |
| --- | --- |
| [`3_DOF_Serial_Monitor_Laser`](3_DOF_Serial_Monitor_Laser/3_DOF_Serial_Monitor_Laser.ino) | Serial commands for three-axis movement and the laser interface. Linear and rotational commands are relative; flexion uses an absolute tip-angle target. Active axes can move concurrently. |
| [`3_DOF_USB_Controller_Mapping_Laser`](3_DOF_USB_Controller_Mapping_Laser/3_DOF_USB_Controller_Mapping_Laser.ino) | Guided discovery of joystick axes and command buttons, followed by controller-based motion, software home return, status reporting, and laser commands. |
| [`3_DOF_USB_Controller_Standardized_Controls_Laser`](3_DOF_USB_Controller_Standardized_Controls_Laser/3_DOF_USB_Controller_Standardized_Controls_Laser.ino) | A controller version with predefined axis and button mappings. Startup mapping prompts are disabled in the supplied configuration. |
| [`Standalone_Controller_Diagnostics`](Standalone_Controller_Diagnostics/Standalone_Controller_Diagnostics.ino) | USB controller connection information, device identifiers, button events, axis changes, and status reports. This diagnostic sketch does not command motors, servos, relays, or laser outputs. |

## My contributions

### Bringing the three motion axes together

I integrated control of the ureteroscope's three degrees of freedom into one Teensy 4.1 architecture:

| Movement | Hardware |
| --- | --- |
| Linear insertion and retraction | NEMA 14 stepper motor and TB6600 driver |
| Axial rotation | NEMA 17 stepper motor and TB6600 driver |
| Tip flexion | ST3020 serial bus smart servo |

My programming work included motor-to-motion conversion calculations, encoder-based tracking and feedback, servo position readback, software-defined home targets, motion limits, and error reporting.

For flexion, I incorporated separate conversion gains for the 0–90°, 90–180°, and 180–270° regions to account for the ureteroscope's nonlinear bending behavior. The controller versions blend the gains around the region boundaries. The reported tip angle is estimated from servo position and these calibration factors.

### Game-controller teleoperation

I developed controller interfaces that connect joystick movement with the robot's motion:

| Input | Assigned function |
| --- | --- |
| Left joystick, left/right | Linear insertion and retraction |
| Right joystick, up/down | Axial rotation |
| Right joystick, left/right | Tip flexion |
| B button | Return to software-defined home targets |

I implemented variable-speed input handling with dead zones, response shaping, and acceleration/deceleration smoothing. The mapping version helps identify a controller's inputs; the standardized version uses predefined mappings for the intended controller.

During normal joystick operation, linear motion can be combined with rotation or flexion. The right-stick filter selects rotation or flexion, so these versions do not provide unrestricted simultaneous manual control of all three axes. The Serial Monitor version can run the active axes concurrently, and the controller versions can coordinate a home return.

### Laser-interface commands and software checks

I integrated the Dornier laser relay/contact interface into the control system and implemented software commands for Enable, Ready, Fire, Pause, Standby, and Stop/Exit.

The controller sketches inhibit new joystick-driven motion commands while the software firing flag is active. The serial sketch includes command gating and stop handling when movement or conflicting input is received during firing.

These are software control checks. The sketches track the commanded laser state internally; they do not receive independent confirmation of actual laser readiness or emission. The repository documents a research prototype and does not establish clinical validation or a complete hardware safety system.

### Electronics and wiring

I helped consolidate the control electronics around a Teensy 4.1 and breakout board, connecting the two TB6600 drivers, linear and rotational encoders, ST3020 servo, Waveshare bus adapter, USB controller, relay modules, external power supplies, and Dornier interface.

My hands-on work included soldering and heat-shrinking wiring extensions and connector changes, integrating the controller's USB connection, and organizing power, ground, step/direction, RX/TX, and digital I/O wiring with consistent color coding. I also worked with high/low-trigger relay modules, Steute interface components, and 10 kΩ pull-up resistors.

I organized these components to make the platform easier to understand, maintain, and troubleshoot as the project develops.

### Diagnostics and troubleshooting

Across the motion-control sketches, I added configuration and status reporting, encoder-based motion checks, servo communication feedback, and error messages to help locate problems during development.

I also developed the standalone controller diagnostic sketch to inspect connections, device information, raw button masks, named button events, and joystick/trigger values separately from robot actuation. Names such as "Home" and "Fire" in its output are diagnostic labels only.

### Work toward the digital twin

I contributed to making the physical robot's motion conventions compatible with the intended Isaac Sim/VR environment. In the controller sketches, rotational targets are limited to −180° to +180°.

Connecting the physical robot to the digital twin remains ongoing work. These four sketches document the embedded-control side of that effort; they do not include a complete Isaac Sim or VR application.

## Implementation details and current scope

- **Positioning depends on the version.** The serial sketch accepts relative linear and rotation moves and an absolute flexion target. In the controller sketches, normal linear motion is speed-controlled, rotation follows an accumulated position target, and flexion follows a tip-angle target.
- **Home is software-defined.** Linear and rotation references are established at startup; flexion uses a configured neutral position. These sketches do not perform a search for independent homing switches.
- **Rotation handling differs.** The ±180° limit applies to the controller versions. The serial version wraps its stored rotation orientation after full turns and does not enforce that same travel limit.
- **Feedback is not a measurement of every physical state.** Encoders support linear/rotation tracking and control. Tip flexion is estimated from servo readback and calibration; laser state is tracked in software.
- **Fault handling has limits.** In the controller sketches, the motor-fault handler does not itself clear the laser outputs, and the Fire handler does not check the motor-fault latch. Stopping new motion commands also does not establish that an already-issued servo movement has physically stopped.
- **Diagnostics support development.** Status messages and software checks are not a substitute for separate hardware testing or validation.

## Development environment

- Teensy 4.1 with an Arduino-compatible Teensy development environment.
- `SCServo.h` / `SMS_STS` for the ST3020 in the three motion-control sketches.
- `USBHost_t36.h` for the two controller sketches and standalone controller diagnostics.
- Serial Monitor configured at 115200 baud in all four sketches.
- ST3020 communication through `Serial1` at 1,000,000 baud in the motion-control sketches.

The external libraries are not bundled, and exact library/toolchain versions are not pinned in this snapshot. Each sketch is a separate program and should be opened from its own matching folder. Hardware-specific pin assignments, interface polarity, controller mappings, and calibration values are defined in the source.

## Project context

GLiDAR is a team research project exploring robot-assisted ureteroscopy and kidney-stone treatment. My contribution focused on integrating the control software and electronics into a more unified platform that can support continued work in teleoperation, automation, and digital-twin research.

The complete platform reflects the work of the wider GLiDAR team. This repository presents selected internship contributions and does not claim ownership of the entire system.

