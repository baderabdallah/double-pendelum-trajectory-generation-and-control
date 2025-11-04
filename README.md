# Double Pendulum Trajectory Generation and Control

A comprehensive MATLAB/Simulink implementation of various control strategies for a double pendulum system, including trajectory generation, dynamics modeling, and advanced control techniques.

## 🎥 Demo Videos

### System Behavior Demonstrations

**Free Response of Double Pendulum**

https://github.com/user-attachments/assets/115badbf-d032-4918-a3c8-446416cc9849

### Control Demonstrations

**Cartesian Point Tracking**

https://github.com/user-attachments/assets/1386691a-06f8-4bbd-aa63-51c930f5369f

**Trajectory Following Control**

https://github.com/user-attachments/assets/1e76701c-3af7-41a0-a624-33b2fbf130d3

### Stabilization Control Performance

**Upright Centering - Slow Response**

https://github.com/user-attachments/assets/f5f47e83-1252-4d72-85ac-b36f1aa6b5b4

**Upright Centering - Fast Response**

https://github.com/user-attachments/assets/731587c3-f4a6-4cd3-a58c-b647bee29e2c

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Installation & Setup](#installation--setup)
- [Usage](#usage)
- [Control Methods](#control-methods)
- [System Parameters](#system-parameters)
- [Results](#results)
- [Contributing](#contributing)
- [License](#license)

## 🔍 Overview

This project implements and compares different control strategies for a double pendulum system. The double pendulum is a classic example in robotics and control theory, demonstrating complex nonlinear dynamics and serving as a benchmark for various control algorithms.

The implementation includes:
- Complete dynamics modeling of the double pendulum system
- Trajectory generation and planning
- Multiple control strategies (PID, PD with gravity compensation, Inverse dynamics)
- Both joint space and operational space control
- Comprehensive simulation and visualization tools

## ✨ Features

- **Multiple Control Strategies**: Implementation of various control methods for comparison
- **Dual Control Spaces**: Both joint space and operational space control implementations
- **Trajectory Planning**:  Trajectory generation for smooth motion
- **Real-time Simulation**: Interactive Simulink models for real-time analysis
- **Comprehensive Visualization**: Animation and plotting tools for result analysis
- **Modular Design**: Well-structured, reusable code components

## 📁 Project Structure

```
DoublePendulum/
├── Joint space PID control/           # PID control in joint coordinates
├── Joint space PD with gravity control/  # PD + gravity compensation (joint space)
├── Operational space PD with gravity control/  # PD + gravity compensation (operational space)
├── Operational space inverse dynamics control/  # Inverse dynamics control (operational space)
├── Videos/                            # Demonstration videos and results
└── README.md                          # This file
```

### Each Control Method Folder Contains:

- `m_I_doublependulum.m` - Double pendulum dynamics model
- `m_II_parameters.m` - System parameters and configuration
- `m_III_trajectory_generation.m` - Trajectory planning algorithms
- `m_IV_run_*.m` - Main simulation runner
- `m_V_plot_RefDes.m` - Results plotting and visualization
- `s_VI_*.slx` - Simulink model files
- `slprj/` - Simulink project files

## 🔧 Prerequisites

- **MATLAB** (R2018a or later recommended)
- **Simulink** 
- **Robotics Toolbox for MATLAB** (Peter Corke's toolbox)
- **Control System Toolbox** (optional, for advanced analysis)

## 🚀 Installation & Setup

1. **Clone the repository:**
   ```bash
   git clone https://github.com/baderabdallah/double-pendelum-trajectory-generation-and-control.git
   cd double-pendelum-trajectory-generation-and-control
   ```

2. **Install Robotics Toolbox:**
   - Download and install the Robotics Toolbox for MATLAB
   - Add the toolbox to your MATLAB path
   - Verify installation by running `ver` in MATLAB command window

3. **Setup MATLAB Path:**
   - Add the project directory and all subdirectories to your MATLAB path
   - Use `addpath(genpath('path/to/DoublePendulum'))` in MATLAB

## 🎯 Usage

### Quick Start

1. **Choose a control method** from the available folders
2. **Navigate to the selected folder** in MATLAB
3. **Run the simulation:**
   ```matlab
   % For Joint space PID control
   cd 'Joint space PID control'
   m_IV_run_pid_contr
   
   % For Joint space PD with gravity control
   cd 'Joint space PD with gravity control'
   m_IV_run_pd_wG_control
   
   % For Operational space controls
   cd 'Operational space PD with gravity control'
   m_IV_run_pd_wG_control_operational
   
   cd 'Operational space inverse dynamics control'
   m_IV_run_inverse_dynamics_operational
   ```

### Simulation Workflow

1. **Parameters Setup:** `m_II_parameters.m` sets system parameters
2. **Trajectory Generation:** `m_III_trajectory_generation.m` creates reference trajectories
3. **Simulation Execution:** `m_IV_run_*.m` runs the complete simulation
4. **Results Visualization:** Automatic plotting and animation generation

## 🎮 Control Methods

### 1. Joint Space PID Control
- **Description:** Classical PID control applied in joint coordinates
- **Advantages:** Simple implementation, well-understood behavior
- **Use Case:** Basic trajectory tracking with acceptable performance

### 2. Joint Space PD with Gravity Control
- **Description:** PD control with gravity compensation in joint space
- **Advantages:** Better steady-state performance, reduced steady-state error
- **Use Case:** Improved tracking with gravity effects compensation

### 3. Operational Space PD with Gravity Control
- **Description:** PD control with gravity compensation in Cartesian coordinates
- **Advantages:** Intuitive end-effector control, natural task specification
- **Use Case:** Direct end-effector positioning and trajectory following

### 4. Operational Space Inverse Dynamics Control
- **Description:** Model-based inverse dynamics control in operational space
- **Advantages:** Excellent tracking performance, theoretically perfect control
- **Use Case:** High-precision applications requiring exact trajectory following

## ⚙️ System Parameters

The double pendulum system is characterized by:

```matlab
% Physical Parameters (defined in m_II_parameters.m)
Grav = 10;      % Gravity acceleration (m/s²)
M = 1;          % Mass of each link (kg)
L1 = 1;         % Length of first link (m)
L2 = 1;         % Length of second link (m)
Ts = 0.03;      % Sampling time (s)
```

### Modifying Parameters

To experiment with different system configurations:
1. Edit the `m_II_parameters.m` file in the desired control folder
2. Modify physical parameters (mass, length, gravity)
3. Adjust control gains for optimal performance
4. Re-run the simulation

## 📊 Results

The simulation generates:
- **Joint angle trajectories** vs. reference trajectories
- **End-effector position plots** in Cartesian coordinates
- **Control effort visualization** (torque inputs)
- **Real-time animation** of the double pendulum motion
- **Performance metrics** (tracking error, settling time, overshoot)

### Typical Output
- Angular position and velocity plots for both joints
- X-Y trajectory of the end-effector
- Control torque time histories
- 3D animation showing pendulum motion

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues, feature requests, or pull requests.

### Development Guidelines
1. Follow MATLAB coding standards
2. Document new functions thoroughly
3. Test all control methods after modifications
4. Update README for significant changes

## 📄 License

This project is developed for educational and research purposes. Please cite appropriately if used in academic work.

## 📧 Contact

**Author:** Abdallah Bader  
**Date:** April 17, 2018  
**Course:** Robotics Class with Prof. Rizzo

---

## 🎥 Demo Videos

Check the `Videos/` folder for demonstration videos showing the different control methods in action.

## 🔗 References

- Robotics Toolbox for MATLAB by Peter Corke
- Modern Robotics: Mechanics, Planning, and Control
- Classical control theory and nonlinear dynamics literature
