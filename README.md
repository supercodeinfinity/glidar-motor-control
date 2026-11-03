# GLiDAR — Robotics Control Software

**GLiDAR (Guided Lithotripsy with Dynamic Autonomous Robot)**  
*University of Miami | Student Research Internship | 2026*

## About the project

I am grateful to have conducted an internship this summer with the University of Miami's GLiDAR team.
GLiDAR is a research project exploring robot-assisted ureteroscopy, a procedure that uses a thin, camera-equipped instrument to reach kidney stones. My work focused on bringing separate motor controls, controller inputs, position feedback, and laser commands together using a **Teensy 4.1 microcontroller**. I also helped organize and connect the electronics that made the software work with the physical robot.

This repository shares **five selected programs I worked on**. They are part of a larger team project, not the full GLiDAR system.

## Watch the system in action

**[GLiDAR demonstration video](https://www.youtube.com/watch?v=Hd46UH1fy5Y)**

The video shows the robotic ureteroscope, handheld controller, electronics, endoscopic camera view, and laser activation during a supervised laboratory demonstration. A novice operator navigated to and fired on all **eight simulated stone targets** during the benchtop session.

## What I worked on

### Connecting three robotic movements

I wrote and developed code that brings the ureteroscope's three degrees of freedom into one control system:

| Movement | Hardware | What it does |
| --- | --- | --- |
| **Linear movement** | NEMA 14 stepper motor | Moves the ureteroscope forward and backward |
| **Rotation** | NEMA 17 stepper motor | Turns the ureteroscope around its axis |
| **Tip flexion** | ST3020 smart servo | Bends the tip to point in different directions |

I used encoder readings to control and verify linear and rotational movement, utilizing a 1X decoding program inspired by previous code. Because the ureteroscope tip does not bend at a constant rate, I used different calibration factors for three bending ranges and smoother transitions between them. I used serial monitor prints to help with testing.

### Turning a game controller into a robot controller

I developed programs that let the operator control the robot with a USB gamepad instead of entering every movement as a number. **Small joystick movements give slower motion, and larger movements give faster motion.** I also added dead zones to reduce unwanted movement from joystick drift, smoother speed changes, software travel limits, and a button to return the axes toward their starting positions.

The controller can command forward/backward movement along with either rotation or tip bending. The right joystick intentionally separates rotation and bending rather than accepting large diagonal inputs.

### Adding the laser interface and checking the system

I integrated software commands for **Enable, Ready, Fire, Pause, Standby, and Stop** into the same interface. The code requires an Enable → Ready sequence before accepting Fire and blocks new joystick-driven movement commands while the software firing state is active.

I also worked on the wiring and electronics, including connections to the motor drivers, encoders, servo, controller, and relay interface. Troubleshooting was a big part of the project, so I added diagnostic messages and created a separate program to check what the controller is actually sending before trying to move the robot.

## Programs in this repository

Each Arduino sketch is in its own folder with a matching `.ino` filename.

| Program | What it shows |
| --- | --- |
| [`Serial_Motion_Laser.ino`](Serial_Motion_Laser.ino) | Controls all three robotic movements through typed commands in the Arduino Serial Monitor, along with separate laser commands. Linear and rotation commands are relative, while tip bending uses a target angle. |
| [`Gamepad_Setup_And_Control_Laser.ino`](Gamepad_Setup_And_Control_Laser.ino) | Guides users through identifying the game controller's joysticks, buttons, and triggers before using them to control the robot's movements and laser interface. |
| [`Gamepad_Preset_Controls_Laser.ino`](Gamepad_Preset_Controls_Laser.ino) | Uses predefined game-controller mappings for easier operation, including proportional joystick control, encoder feedback, movement limits, and laser commands. |
| [`USB_Controller_Diagnostics.ino`](USB_Controller_Diagnostics.ino) | Displays controller connections, joystick movements, button presses, and trigger readings to help troubleshoot the controller. This program does not activate motors or laser outputs. |
| [`IsaacSim_VR_Teensy_Bridge_Draft.py`](IsaacSim_VR_Teensy_Bridge_Draft.py) | **Work in progress:** Uses Meta Quest 3 inputs to control the robotic digital twin in NVIDIA Isaac Sim and includes serial communication code intended to connect with the physical Teensy system. Full physical–digital synchronization is still being developed and tested. |

## My next step: connecting the physical robot and digital twin

Another part of my work is exploring how the physical robot could connect to its **digital twin** in NVIDIA Isaac Sim. The new Python script reads Quest 3 controller inputs, updates the simulated robot's joints, and prepares serial movement commands for the Teensy. I am still working on testing the connection with matching physical-robot code, so this is **not a finished or validated synchronized system**.

I hope that connecting the physical and virtual versions can eventually support robotic training, simulation, and future automation research.

## Research presentations

I presented our GLiDAR research posters at workshops during **IEEE/RSJ IROS 2026 in Pittsburgh**, including SurgTwin, RAISE, the 2nd International Workshop on Intelligent Robotics for Soft Tissue Surgery, and Training the Future Medical Roboticist. Sharing the research was another exciting part of the experience because I could explain the engineering decisions and answer questions about the project.

## Technical notes and limitations

The Arduino sketches were developed for a **Teensy 4.1** using an Arduino-compatible development environment. The motion programs use `SCServo.h`, and the USB controller programs use `USBHost_t36.h`. The experimental Quest 3 script uses Python with NVIDIA Isaac Sim/XR and `pyserial`.

This is an **experimental research prototype**, not software for clinical use. The HOME positions are based on startup references, not independently detected homing switches. Encoder readings track the linear and rotation motors; the actual tip angle is estimated from servo readings and calibration. The laser-control software tracks the commands it sends, not independently verified laser status. Software checks and fault handling still require further testing and do not establish complete hardware or clinical safety.

## Credits

GLiDAR is a team research project at the University of Miami. This repository highlights **my selected programming and electronics contributions** from the internship; the full robotic platform and research results reflect the work of the wider team.

**GLiDAR research team:** Sarah Schürer, Sarvesh Saini, Julio Ojalvo, Jonathan E. Katz, and Ubbo Visser.
