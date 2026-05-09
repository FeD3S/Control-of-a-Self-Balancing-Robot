# Two-Wheeled Balancing Robot: Longitudinal State-Space Control

## Overview
This repository contains the simulation models and control algorithms for the longitudinal state-space control of a two-wheeled balancing robot (inverted pendulum). The project was developed as part of the Control Engineering Laboratory coursework for the Master's Degree in Control Systems Engineering at the Università degli Studi di Padova.

The implemented controller simultaneously stabilizes the robot's upward vertical position and tracks a desired longitudinal position set-point, assuming straight-line motion.

## Features
* **Nonlinear Dynamics Simulation**: Complete planar electromechanical model implemented via standard Simulink blocks (chain of integrators) and a Level-2 MATLAB S-Function.
* **Sensor Modeling**: Accurate simulation of the onboard MPU6050 (accelerometer and gyroscope) with noise and quantization, alongside Hall-effect incremental encoders.
* **State Estimation**: Implementation of a discrete-time Complementary Filter to fuse accelerometer and gyroscope data for reliable tilt angle estimation, mitigating drift and high-frequency noise.
* **LQR Control**: Design of a Linear Quadratic Regulator (LQR) to compute optimal feedback gains, including an integral action component for robust constant set-point tracking and disturbance rejection.
* **Hardware Deployment**: Ready-to-deploy Simulink configuration for the Arduino Mega 2560 microcontroller using the Balancing Robot Toolbox (BRT).

## Repository Structure
* `balrob_params.m`: Initialization script containing all geometrical, inertial, and electrical parameters of the robot, as well as sensor conversion gains.
* `sfun_balrob_long_dyn.m`: MATLAB Level-2 S-Function evaluating the nonlinear equations of motion (mass matrix, Coriolis/centrifugal terms, viscous friction, and gravity loading).
* `simulink_models/`: Directory containing `.slx` files for the isolated physical plant, the state-observer, the full closed-loop simulation, and the hardware-deployment model.

## Setup and Usage
1. Open MATLAB and ensure the Control System Toolbox is installed. 
2. Run the `balrob_params.m` script to load the robot parameters into the workspace.
3. Open the main Simulink control model.
4. Run the simulation to observe the balance restoration and step-response tracking.
5. To deploy to hardware, open the deployment `.slx` model, ensure the Arduino is connected, and initiate the build process via the Simulink hardware interface.

## Authors
* **Federico Saporiti**
* **Davide Pillon**
* **Leonardo Luigi Pepe**

Control Systems Engineering
