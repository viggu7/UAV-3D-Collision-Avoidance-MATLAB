# UAV-3D-Collision-Avoidance-MATLAB

Progressive development of a 3D UAV collision avoidance system with Kalman-based state estimation, adaptive blended control, multi-intruder handling, and full attitude tracking (roll, pitch, yaw) implemented in MATLAB.

---

## Project Overview

This repository contains the complete development progression of a 3D reactive UAV collision avoidance framework built in MATLAB.

The system simulates a UAV navigating toward a goal while:

- Avoiding multiple moving intruders
- Handling sensor noise and uncertainty
- Estimating intruder states using a Kalman filter
- Performing adaptive blended control
- Tracking roll, pitch, and yaw attitudes
- Logging direction changes and safety metrics

The project evolves step-by-step from a basic reactive avoidance model to an uncertainty-aware tracking and attitude analysis framework.

---

## Development Stages

### Version 1 – Basic Reactive Avoidance
- Direct goal-seeking velocity
- Repulsive force–based obstacle avoidance
- Safety and detection radius zones
- 3D trajectory visualization

### Version 2 – Enhanced Blended Control
- Smooth velocity blending
- Emergency and warning behavior zones
- Performance metric tracking

### Version 3 – Kalman Filter Integration
- Constant velocity motion model
- Noisy sensor simulation
- State prediction and correction
- Uncertainty-aware safety radius expansion

### Version 4 – Ground Safety and Direction Tracking
- Ground collision prevention
- Heading change detection (>30°)
- Event logging with UAV position
- Distance tracking per intruder

### Final Version – Full Attitude Tracking
- Roll, pitch, yaw estimation
- Acceleration-based bank angle computation
- Attitude logging during maneuver events
- CSV export for post-analysis
- Real-time 3D, top-view, and distance plots

---

## Core Algorithms

- Kalman Filter (Prediction and Update)
- Repulsive Potential Field Method
- Adaptive Blended Control Law
- Euler Integration for State Update
- Heading Change Detection using Angular Difference
- Attitude Estimation from Velocity and Acceleration

---

## Safety Logic

- Safety radius: 25 m
- Detection radius: 50 m
- Uncertainty-based safety expansion
- Ground avoidance below 15 m altitude
- Hard altitude constraints for UAV and intruders

---

## Outputs

- 3D trajectory visualization
- UAV–intruder distance plots
- Direction change event tables
- Attitude tracking tables
- CSV exports for further analysis

---

## Tools Used

- MATLAB
- Linear state estimation (Kalman filter)
- Basic aerospace kinematics

---

## Repository Structure

Each version of the code represents a structured improvement over the previous version:

- Base reactive avoidance model
- Improved blended control version
- Kalman-integrated version
- Ground-safe enhanced version
- Final attitude tracking version

---

## Future Work

- 6DOF rigid-body dynamics integration (Simulink)
- Nonlinear motion models
- Wind disturbance modeling
- Sensor fusion with IMU models
- Path planning optimization

---

## License

MIT License
