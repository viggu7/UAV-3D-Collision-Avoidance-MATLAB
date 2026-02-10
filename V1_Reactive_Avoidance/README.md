# V1 – Basic 3D Reactive Collision Avoidance

## Overview

This is the first implementation of the 3D UAV collision avoidance system.

**Version 1** focuses on **purely reactive obstacle avoidance** without any state estimation or filtering. The UAV navigates toward a predefined goal while dynamically avoiding multiple moving intruders using repulsive vector-based control.

This version establishes the foundational motion and avoidance logic used in all later improvements.

---

## Objective

* Implement 3D goal-directed UAV motion
* Handle multiple moving intruders
* Introduce safety and detection zones
* Develop a blended avoidance + goal-seeking control law
* Visualize trajectories and safety regions in real time

---

## System Description

### UAV Model

* **Point-mass kinematic model**
* **Constant speed motion**
* **Goal-directed velocity computation**
* **Euler integration** for position update

### Intruder Model

* **Multiple intruders** (4 concurrent obstacles)
* **Constant velocity motion**
* **No prediction or filtering** (true positions used directly)

---

## Collision Avoidance Strategy

This version uses a **reactive potential-field style approach**.

### Detection Zone

If an intruder enters the **detection radius**:
* A repulsive force vector is generated
* Weight increases as distance decreases

### Safety Zone

If intruder enters **safety radius**:
* Strong avoidance weight applied
* Collision event counter incremented

### Blended Control Law

Final UAV velocity is computed as:

```
U = α(Avoidance Vector) + (1 − α)(Goal Direction)
```

Where:
* **α increases as threat level increases**
* **Emergency mode** heavily prioritizes avoidance

Velocity magnitude is normalized to maintain constant speed.

---

## Key Parameters

| Parameter | Value |
|-----------|-------|
| Time step (dt) | 0.2 s |
| Total simulation time | 30 s |
| UAV speed | 20 m/s |
| Safety radius | 25 m |
| Detection radius | 50 m |
| Number of intruders | 4 |

---

## Performance Metrics Tracked

* ✓ Minimum distance to intruders
* ✓ Total distance traveled
* ✓ Avoidance event count
* ✓ Distance to goal
* ✓ Safety status (SAFE / WARNING / CRITICAL)

---

## Visualization

The simulation provides:

* **3D trajectory** of UAV and intruders
* **Safety sphere** (cyan)
* **Detection sphere** (yellow)
* **Real-time velocity vector**
* **Live mission metrics panel**

---

## Limitations of Version 1

* Uses **true intruder positions** (no sensor noise modeling)
* **No prediction capability**
* **No uncertainty handling**
* **No attitude dynamics**
* **Purely kinematic motion**

---

## Purpose in Project Evolution

**V1 serves as the baseline implementation.**

It provides:
* Core avoidance logic
* Blended control architecture
* Performance metric framework
* Visualization structure

All subsequent versions build on this foundation by adding:
* Kalman filtering
* Uncertainty-aware avoidance
* Adaptive speed control
* Attitude tracking
* Direction change analysis

---

## How to Run

Open MATLAB and execute:

```matlab
UAV_V1_ReactiveAvoidance.m
```

The simulation will run for **30 seconds** or until the **goal is reached**.

---

## File Structure

```
V1_ReactiveAvoidance/
├── UAV_V1_ReactiveAvoidance.m    % Main simulation script
└── README.md                      % This file
```

---

## Expected Output

Upon successful execution, you will see:

1. **3D Visualization Window**
   - Real-time UAV and intruder positions
   - Trajectory trails
   - Safety zones

2. **Console Output**
   - Goal reached notification
   - Final statistics (distance, time, safety metrics)

3. **Performance Summary**
   - Minimum approach distance
   - Total mission time
   - Collision warnings (if any)

---

## Next Steps

This version represents the **foundational reactive collision avoidance stage** of the UAV system development.

**Proceed to V2** to add:
* Kalman filtering for state estimation
* Sensor noise modeling
* Uncertainty-aware collision avoidance
* Improved prediction capabilities

---

## Version Information

| Item | Details |
|------|---------|
| **Version** | 1.0 - Reactive Avoidance |
| **Date** | 2025 |
| **Status** | ✓ Baseline Implementation Complete |
| **Next Version** | V2 - Kalman Filter Integration |

---

**This version represents the foundational reactive collision avoidance stage of the UAV system development.**
