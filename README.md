# COMAU SmartSix: Kinematic Modeling and Control

This repository contains the implementation of kinematic modeling and control strategies for the **COMAU SmartSix** industrial manipulator, developed as a final project for the Robotic Manipulators course at UFMG.

The project covers the complete pipeline from mathematical modeling using Denavit-Hartenberg parameters to practical execution in both simulated environments and a real laboratory setup.

## 🤖 Project Overview

The primary objective was to enable the 6-DOF COMAU SmartSix robot to perform complex geometric trajectories. Specifically, the project involves:
* **Mathematical Modeling:** Defining the forward kinematics using the Denavit-Hartenberg (DH) convention.
* **Kinematic Control:** Implementing Regulation Control for point-to-point positioning and Trajectory Tracking for continuous movement.
* **Simulation:** Validating the models in **MATLAB (Robotics Toolbox)** and **CoppeliaSim**.
* **Real-World Application:** Programming the physical robot using the **PDL2** language to draw geometric shapes.

---

## 🛠 Simulation Details

The simulation serves as a validation bridge between the mathematical theory and the physical robot.

### 1. Forward Kinematics (DH Parameters)
The robot's geometric structure was mapped using conventional Denavit-Hartenberg parameters based on manufacturer specifications. 

| Joint | $\theta$ (rad) | $d$ (m) | $a$ (m) | $\alpha$ (rad) |
| :--- | :--- | :--- | :--- | :--- |
| 1 | $q_1$ | -0.450 | 0.150 | $\pi/2$ |
| 2 | $q_2 - \pi/2$ | 0 | 0.590 | $\pi$ |
| 3 | $q_3 + \pi/2$ | 0 | 0.130 | $-\pi/2$ |
| 4 | $q_4$ | -0.6471 | 0 | $-\pi/2$ |
| 5 | $q_5$ | 0 | 0 | $\pi/2$ |
| 6 | $q_6 + \pi$ | -0.095 | 0 | $\pi$ |

> **Note:** A $180^\circ$ rotation around the X-axis was applied to the base to align the simulation coordinates with the physical laboratory setup.

### 2. Control Strategies
Two main control techniques were implemented in the MATLAB environment:

* **Regulation Control:** Used for initial movements to set the end-effector at a starting point. It ensures the final position error converges to zero without strictly constraining the path timing.
* **Trajectory Tracking:** Used for drawing shapes. This employs a proportional controller with a velocity feedforward action to follow time-indexed references (e.g., parametric equations for a circle).

### 3. The "Brazilian Flag" Challenge
In the simulation environment, the robot was tasked with drawing a simplified Brazilian flag in the YZ plane. 


The sequence includes:
* **Rectangle:** Linear interpolation between four vertices ($P_1$ to $P_4$).
* **Rhombus (Losango):** Linear interpolation between center-edge points ($P_5, P_{L1}, P_6, P_{L2}$).
* **Circle:** A timed parametric trajectory calculated using trigonometric relations to ensure tangency with the rhombus.

---

## 📺 Results & Videos

The system performance was evaluated by monitoring joint positions, control actions, and Euclidean errors (Position/Orientation), all of which converged to zero as expected.

***CoppeliaSim Simulation:** [Watch here](https://youtu.be/VgL7sGtXyPA) 
* **MATLAB/Robotics Toolbox:** [Watch here](https://youtu.be/kEf42UPHgCg) 
* **Real Robot Execution:** [Watch here](https://youtu.be/sKTQvtZwOzc) 

---
