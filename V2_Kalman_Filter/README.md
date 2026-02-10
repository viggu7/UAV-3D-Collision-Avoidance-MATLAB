# 3D Reactive Collision Avoidance with Kalman Filtering

A MATLAB simulation demonstrating autonomous UAV navigation with real-time collision avoidance using Kalman filtering for state estimation under sensor noise and uncertainty.

## Overview

This simulation models a UAV navigating through 3D space toward a goal while avoiding multiple dynamic intruders. The system uses Kalman filters to estimate intruder positions and velocities from noisy sensor measurements, enabling robust collision avoidance even under uncertainty.

## Key Features

- **Kalman Filtering**: Individual Kalman filters track each intruder's position and velocity
- **Sensor Simulation**: Realistic range-limited sensors with Gaussian measurement noise
- **Uncertainty-Aware Avoidance**: Safety margins adapt based on estimation uncertainty
- **Multi-Intruder Handling**: Simultaneous tracking and avoidance of multiple threats
- **Real-time Visualization**: Live 3D animation with trajectories, estimates, and metrics
- **Performance Analysis**: Tracking of estimation errors and avoidance maneuvers

## System Architecture

### State Estimation
- **State Vector**: `[x, y, z, vx, vy, vz]` for each intruder
- **Motion Model**: Constant velocity with process noise
- **Measurements**: Position-only observations with configurable noise
- **Sensor Range**: 100m detection radius

### Collision Avoidance
- **Safety Radius**: 25m (critical avoidance zone)
- **Detection Radius**: 50m (early warning zone)
- **Avoidance Strategy**: Repulsive potential field with perpendicular escape component
- **Adaptive Blending**: Dynamic mixing of goal-directed and avoidance behaviors

### Control Law
```
α = avoidance weight (0.5 to 0.95 based on proximity)
v_uav = α * v_avoid + (1-α) * v_goal
```

## Parameters

### Simulation
- **Time Step**: 0.2s
- **Duration**: 30s
- **UAV Speed**: 20 m/s

### Kalman Filter
- **Process Noise Intensity**: q = 5
- **Measurement Noise**: σ = 5m
- **Initial Position Uncertainty**: 100m²
- **Initial Velocity Uncertainty**: 25 (m/s)²

### Scenario
- **Starting Position**: [0, 0, 0]
- **Goal Position**: [200, 200, 100]
- **Number of Intruders**: 4 (configurable)
- **Intruder Dynamics**: Constant velocity with varied trajectories

## Usage

### Running the Simulation
```matlab
% Simply run the script
run('collision_avoidance_kalman.m')
```

### Customizing Intruders
```matlab
% Add/modify intruder initial conditions
intruder_pos = [
     x1  y1  z1;
     x2  y2  z2;
     ...
];

intruder_vel = [
    vx1 vy1 vz1;
    vx2 vy2 vz2;
    ...
];
```

### Adjusting Safety Parameters
```matlab
safety_radius = 25;      % Minimum safe distance
detection_radius = 50;   % Start avoidance at this range
sensor_range = 100;      % Maximum sensor detection range
```

## Visualization

The simulation provides four synchronized displays:

1. **3D Trajectory View**: Main visualization showing:
   - Blue solid line: UAV trajectory
   - Red dashed lines: True intruder trajectories
   - Magenta squares: Kalman filter estimates
   - Cyan X's: Noisy sensor measurements
   - Translucent sphere: Safety bubble

2. **Metrics Panel**: Real-time statistics including:
   - Time elapsed
   - Distance to goal
   - Closest intruder distance
   - Minimum recorded separation
   - Avoidance event counter
   - Kalman filter performance

3. **Estimation Error Plot**: Per-intruder tracking accuracy over time

4. **Status Indicator**: Current navigation mode (NOMINAL/AVOIDING)

## Performance Metrics

The simulation tracks and reports:
- Total distance traveled
- Minimum separation achieved
- Number of avoidance maneuvers executed
- Average/maximum estimation error
- Filter uncertainty levels

## Algorithm Details

### Prediction Step
```
x̂(k|k-1) = F * x̂(k-1|k-1)
P(k|k-1) = F * P(k-1|k-1) * F' + Q
```

### Update Step (when measurement available)
```
Innovation: y = z - H * x̂(k|k-1)
Innovation Covariance: S = H * P(k|k-1) * H' + R
Kalman Gain: K = P(k|k-1) * H' * S^(-1)
State Update: x̂(k|k) = x̂(k|k-1) + K * y
Covariance Update: P(k|k) = (I - K*H) * P(k|k-1)
```

### Uncertainty-Adaptive Safety
```
effective_safety = safety_radius + √(trace(P_position))
weight = weight * exp(-uncertainty/10)
```

## Requirements

- MATLAB R2016b or later
- No additional toolboxes required
- Graphics hardware for real-time 3D rendering

## Typical Output

```
========== Simulation Complete ==========
Total time: 30.0 s
Total distance traveled: 342.8 m
Minimum distance to intruders: 18.3 m
Avoidance maneuvers: 47
Final distance to goal: 8.2 m

--- Kalman Filter Performance ---
Average estimation error: 4.67 m
Max estimation error: 12.34 m
=========================================
```

## Future Enhancements

- Extended Kalman Filter for non-linear dynamics
- Multi-sensor fusion (radar + vision)
- Predictive collision detection using velocity estimates
- Adaptive process/measurement noise tuning
- 3D obstacle fields
- Formation flying with multiple UAVs

## Author & License

Research code for UAV collision avoidance studies.
MIT License - Free to use and modify with attribution.

## References

- Kalman, R.E. (1960). "A New Approach to Linear Filtering and Prediction Problems"
- Van Der Merwe, R. (2004). "Sigma-Point Kalman Filters for Probabilistic Inference"
- Lavalle, S.M. (2006). "Planning Algorithms" - Chapter 13: Sensor-Based Planning
