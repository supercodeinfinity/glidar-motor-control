# GLiDAR — My Robotics Control Software

**GLiDAR (Guided Lithotripsy with Dynamic Autonomous Robot)**  
*University of Miami | Student Research Internship | 2026*

## About the project

How do you make three different motors and a medical laser work through one controller? That was one of the main challenges I worked on during my summer internship with the University of Miami's GLiDAR team.

GLiDAR is a research project exploring robot-assisted ureteroscopy, a procedure that uses a thin, camera-equipped instrument to reach kidney stones. My work focused on bringing separate motor controls, controller inputs, position feedback, and laser commands together using a **Teensy 4.1 microcontroller**. I also helped organize and connect the electronics that made the software work with the physical robot.

This repository shares **five selected programs I worked on**. They are part of a larger team project, not the full GLiDAR system.

## Watch the system in action

**[GLiDAR demonstration video](INSERT_PUBLIC_VIDEO_URL_HERE)**

The video shows the robotic ureteroscope, handheld controller, electronics, endoscopic camera view, and laser activation during a supervised laboratory demonstration. Our team navigated to and fired on all **eight simulated stone targets** during the benchtop session. It was exciting to see the different parts of the system finally work through one interface!

## What I worked on

### Connecting three robotic movements

I wrote and developed code that brings the ureteroscope's three degrees of freedom into one control system:

| Movement | Hardware | What it does |
| --- | --- | --- |
| **Linear movement** | NEMA 14 stepper motor | Moves the ureteroscope forward and backward |
| **Rotation** | NEMA 17 stepper motor | Turns the ureteroscope around its axis |
| **Tip flexion** | ST3020 smart servo | Bends the tip to point in different directions |

I used encoder readings to track linear and rotational movement, calculated how motor motion translates into instrument movement, and added status messages to help with testing. Because the tip does not bend at a constant rate, I used different calibration factors for three bending ranges and smoother transitions between them.

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
| [`3_DOF_Serial_Monitor_Laser`](3_DOF_Serial_Monitor_Laser/3_DOF_Serial_Monitor_Laser.ino) | Moves the three axes using typed commands in the Arduino Serial Monitor, with separate laser commands. Linear and rotation commands are relative; tip bending uses a target angle. |
| [`3_DOF_USB_Controller_Mapping_Laser`](3_DOF_USB_Controller_Mapping_Laser/3_DOF_USB_Controller_Mapping_Laser.ino) | Walks through identifying the game controller's sticks, buttons, and triggers before using them to operate the robot. |
| [`3_DOF_USB_Controller_Standardized_Controls_Laser`](3_DOF_USB_Controller_Standardized_Controls_Laser/3_DOF_USB_Controller_Standardized_Controls_Laser.ino) | Uses preset game-controller mappings for more convenient operation, plus position feedback, movement limits, and laser commands. |
| [`Standalone_Controller_Diagnostics`](Standalone_Controller_Diagnostics/Standalone_Controller_Diagnostics.ino) | Displays joystick, button, trigger, and connection readings. It **does not** move motors or activate laser outputs. |
| [`GLiDAR_Quest3_VR_Control.py`](GLiDAR_Quest3_VR_Control.py) | **Work in progress:** reads Meta Quest 3 inputs to move the GLiDAR model in NVIDIA Isaac Sim and includes code to send movement requests toward a Teensy over serial. Full physical–digital synchronization is not yet demonstrated. |

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
