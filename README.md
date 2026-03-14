# 🐾 Development of a 12-DOF Quadrupedal Robot with Enhanced Navigation

🚀 A cost-effective quadrupedal robotic platform designed for stable
locomotion on uneven and non-planar terrains using embedded control and
torque-validated actuator design.

------------------------------------------------------------------------

## 📌 Project Overview

Robotic mobility in unstructured environments remains a major challenge
in robotics engineering. Traditional wheeled robots struggle on rocky
surfaces, stairs, and irregular terrain.

This project develops a **12-Degree-of-Freedom (12-DOF) quadrupedal
robot** capable of stable locomotion on uneven terrain using articulated
legs and embedded control systems.

Each leg contains three actuated joints:

-   Hip Roll
-   Hip Pitch
-   Knee Pitch

The robot is controlled using a **Raspberry Pi based architecture** with
servo actuators and remote control input for gait validation and terrain
testing.

------------------------------------------------------------------------

# 🤖 Robot Architecture

    User Input (PS4 Controller / Keyboard)
                │
                ▼
          Raspberry Pi
                │
         Gait Controller
                │
         Inverse Kinematics
                │
         Hardware Interface
                │
            Servo Driver
                │
            Servo Motors

The architecture integrates motion planning, joint coordination, and
hardware communication in a modular control pipeline.

------------------------------------------------------------------------

# ⚙️ Robot Specifications

  Parameter            Value
  -------------------- --------------------
  Degrees of Freedom   12 DOF (3 per leg)
  Body Length          380 mm
  Body Width           250 mm
  Shoulder Height      250 mm
  Thigh Length         140 mm
  Shank Length         140 mm
  Total Mass           \~3 kg
  Payload Capacity     \~0.5 kg

------------------------------------------------------------------------

# 🦾 Hardware Components

### Embedded System

-   Raspberry Pi (Main Controller)

### Actuators

-   STS3215 Serial Servo
-   35kg PWM Servo

### Mechanical Structure

-   3D printed using **Polycarbonate (PC) filament**

### Communication

-   I2C communication with servo driver
-   PWM signals for servo control

------------------------------------------------------------------------

# 🧠 Control System

### 1️⃣ Input Interface

Receives commands from: - PS4 Controller - Keyboard

### 2️⃣ Gait Controller

Implements motion modes: - Trotting gait - Rest posture control

### 3️⃣ Inverse Kinematics

Converts foot position to joint angles for each leg.

### 4️⃣ Hardware Interface

Responsible for: - Joint angle validation - Signal conversion - Servo
command transmission

------------------------------------------------------------------------

# 🧪 Testing and Validation

### Static Testing

-   Posture stability
-   Joint limit validation
-   Load bearing verification

### Dynamic Testing

-   Forward walking
-   Turning maneuvers
-   Incline traversal

### System Monitoring

-   Servo temperature monitoring
-   Power consumption analysis

The robot successfully demonstrated **stable locomotion and controlled
walking on flat and inclined surfaces.**

------------------------------------------------------------------------

# 📊 Torque Analysis

Example calculation:

τ = m × g × L

For the knee joint:

τ = 1.5 × 9.81 × 0.14 ≈ 2.06 Nm

The selected servo motors provide torque values higher than calculated
requirements, ensuring reliable operation.

------------------------------------------------------------------------

# 🌍 Applications

Quadrupedal robots like this can be used in:

-   Disaster inspection
-   Search and rescue missions
-   Rough terrain exploration
-   Agricultural monitoring
-   Industrial inspection in hazardous environments

------------------------------------------------------------------------

# 🔬 Future Improvements

Planned extensions include:

-   Vision-based obstacle detection
-   SLAM navigation
-   IMU-based balance stabilization
-   Autonomous path planning
-   Advanced gait optimization

------------------------------------------------------------------------

# 🎓 Academic Information

**Bachelor of Science in Engineering (Honours)**\
Department of Electrical and Telecommunication Engineering\
Faculty of Engineering\
South Eastern University of Sri Lanka

### Authors

-   Balakumar Premamyuresan\
-   Sahayanathan Renolson\
-   Jeyapalasingam Jeluxshan

### Supervisors

-   Dr. W.G.C.W Kumara\
-   Dr. Ajmal Hinas\
-   Eng. S. Sampavi

------------------------------------------------------------------------

# 📂 Repository Structure (Example)

    quadruped-robot/
    │
    ├── README.md
    ├── CAD/
    ├── firmware/
    ├── control/
    ├── simulation/
    ├── electronics/
    └── docs/

------------------------------------------------------------------------

# ⭐ Project Contribution

This project demonstrates that a locally engineered quadrupedal robot
can be developed using accessible hardware and analytical design
validation, supporting robotics research and education.

------------------------------------------------------------------------

# 📜 License

This project is intended for **academic and research purposes**.
