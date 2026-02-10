# Enhanced 3D Reactive Collision Avoidance with Kalman Filtering
## OPTIMIZED VERSION - Precision Goal-Reaching

A MATLAB simulation demonstrating autonomous UAV navigation with enhanced goal-reaching precision, featuring Kalman filtering for state estimation, adaptive speed control, and intelligent collision avoidance in 3D airspace.

## 🎯 Key Improvements Over Base Version

This optimized version includes four major enhancements:

1. **Precision Goal Threshold**: Reduced from 10m to 1.5m for accurate goal arrival
2. **Adaptive Speed Control**: Dynamic speed reduction from 20 m/s to 5 m/s near goal
3. **Enhanced Goal Attraction**: Distance-based blending weights favor goal-seeking when close
4. **Improved Final Approach**: Smooth deceleration and precise terminal guidance

## 📊 System Overview

### Mission Profile
- **Starting Position**: [0, 0, 30m altitude]
- **Goal Position**: [200, 200, 80m altitude]
- **Mission Distance**: ~283m (direct path)
- **Operating Airspace**: 250m × 250m × 120m

### Altitude Safety Configuration

```
┌─────────────────────────────────────────┐
│ ALTITUDE LAYERS                         │
├─────────────────────────────────────────┤
│ Ceiling:              120m              │
│ Intruder Zone:        55m - 95m         │
│ Goal Altitude:        80m               │
│ UAV Start:            30m               │
│ Ground Warning:       15m (climb alert) │
│ UAV Hard Floor:       5m (bounce-back)  │
│ Ground Level:         0m                │
└─────────────────────────────────────────┘
```

**Safety Features**:
- UAV minimum altitude: 5m (hard floor with upward boost)
- Ground avoidance activates: Below 15m
- Intruder minimum altitude: 20m (bounce-back on contact)
- Altitude markers: Visual dotted lines to ground plane

## 🚁 Core Features

### 1. Adaptive Speed Control

**Speed Profile**:
```matlab
if distance_to_goal < 50m:
    speed = 5 + 15 × (distance/50)²
else:
    speed = 20 m/s (maximum)
```

**Benefits**:
- Smooth deceleration prevents goal overshoot
- Maintains maneuverability near obstacles
- Quadratic reduction for natural feel
- Minimum 5 m/s maintains control authority

### 2. Enhanced Goal Attraction

**Adaptive Blending**:
```matlab
if distance_to_goal < 20m:
    goal_weight = 0.3 + 0.5 × (1 - distance/20)
    → Increases from 30% to 80% as UAV approaches
```

**Effect**: Balances obstacle avoidance with goal-seeking based on proximity

### 3. Kalman Filter State Estimation

**State Vector** (per intruder): `x = [x, y, z, vx, vy, vz]ᵀ`

**Prediction**:
```
x̂(k|k-1) = F·x̂(k-1|k-1)
P(k|k-1) = F·P(k-1|k-1)·Fᵀ + Q
```

**Update** (when measurement available):
```
Innovation:      y = z - H·x̂(k|k-1)
Kalman Gain:     K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹
State Update:    x̂(k|k) = x̂(k|k-1) + K·y
Covariance:      P(k|k) = (I - K·H)·P(k|k-1)
```

### 4. Collision Avoidance Logic

**Repulsive Force**:
```matlab
if d < safety_radius (25m):
    weight = 5.0 × (safety_radius / d)
else if d < detection_radius (50m):
    weight = 2.0 × (detection_radius - d) / detection_radius
```

**Uncertainty Handling**:
```matlab
effective_safety = safety_radius + √(trace(P_position))
confidence = exp(-uncertainty / 10)
weight = weight × confidence
```

**Control Blending**:
```matlab
if closest_distance < safety_radius:
    α = 0.95 × (1 - goal_weight)
else if closest_distance < detection_radius:
    α = 0.70 × (1 - goal_weight)
else:
    α = 0.50 × (1 - goal_weight)

v_uav = α·v_avoid + (1-α)·v_goal
```

## 📈 Performance Tracking

### Real-Time Metrics
- **Position tracking**: UAV and intruder trajectories
- **Distance monitoring**: To goal and each intruder
- **Maneuver analysis**: Heading changes > 30°
- **Estimation accuracy**: Kalman filter error tracking
- **Speed adaptation**: Current vs. maximum speed

### Direction Change Detection

Automatically logs events when heading changes exceed 30°:

```
Event Data Captured:
- Timestamp
- Heading change magnitude (degrees)
- Distance to each intruder
- UAV 3D position
- Safety status (Critical/Warning/Safe)
```

**Export**: CSV file `UAV_Direction_Changes_Optimized.csv`

## 🎮 Multi-Intruder Scenario

### Default Configuration (4 Intruders)

| Intruder | Start Position [x,y,z] | Velocity [vx,vy,vz] | Altitude | Trajectory |
|----------|------------------------|---------------------|----------|------------|
| 1        | [0, 200, 55]           | [15, -5, 0]         | 55m      | Southeast  |
| 2        | [200, 100, 85]         | [-10, 0, -2]        | 85m      | West+Down  |
| 3        | [120, 220, 65]         | [-12, -10, 1]       | 65m      | Southwest  |
| 4        | [220, 120, 95]         | [-10, -12, -1]      | 95m      | Southwest  |

**Collision Geometry**: Converging paths create multiple conflict zones

## 🖥️ Visualization System

### Main 3D View (Subplots 1,2,4,5)
- **Ground plane**: Semi-transparent with grid
- **Trajectories**: 
  - Blue solid: UAV path
  - Red dashed: Intruder paths (color-coded per intruder)
- **Altitude markers**: Dotted lines from objects to ground
- **Ground projections**: Cross markers showing XY position
- **Labels**: Altitude, speed, distances displayed
- **Safety sphere**: Transparent cyan bubble around UAV
- **Velocity vectors**: 
  - Blue solid: Actual UAV velocity
  - Green dashed: Desired velocity (to goal)
  - Red arrows: Intruder velocities

### Top-Down View (Subplot 3)
- XY plane projection
- Safety circle around UAV
- Velocity vector overlays
- Intruder altitude labels

### Distance Plot (Subplot 6)
- Time-series of UAV-intruder separations
- Safety zone (25m) and detection zone (50m) thresholds
- Real-time metrics:
  - Direction changes counter
  - Current speed
  - Distance to goal

### Live Status Panel
- Color-coded intruder markers (4 distinct colors)
- Altitude-aware labeling
- Speed adaptation indicator
- Goal proximity feedback

## 📐 Parameters

### Simulation
```matlab
dt = 0.2;              % Time step (seconds)
total_time = 30;       % Maximum duration (seconds)
```

### UAV
```matlab
uav_speed = 20;        % Maximum speed (m/s)
min_speed = 5;         % Minimum speed near goal (m/s)
goal_threshold = 1.5;  % Success radius (meters)
```

### Safety Zones
```matlab
safety_radius = 25;        % Critical avoidance (meters)
detection_radius = 50;     % Early warning (meters)
sensor_range = 100;        % Maximum detection (meters)
min_altitude = 5;          % UAV floor (meters)
ground_safety = 15;        % Climb trigger (meters)
```

### Kalman Filter
```matlab
% Process noise
q = 5;

% Measurement noise  
R = 25 × I₃  (σ = 5m)

% Initial uncertainty
P₀ = diag([100, 100, 100, 25, 25, 25])
```

## 🚀 Usage

### Basic Execution
```matlab
% Run the complete simulation
run('collision_avoidance_optimized.m')
```

### Customizing Intruders
```matlab
% Modify starting positions
intruder_pos = [
    x1  y1  z1;
    x2  y2  z2;
    % ... add more rows
];

% Modify velocities
intruder_vel = [
    vx1 vy1 vz1;
    vx2 vy2 vz2;
    % ... add more rows
];
```

### Adjusting Goal
```matlab
uav_goal = [200 200 80];  % [x, y, altitude]
```

### Tuning Avoidance Behavior
```matlab
% More aggressive avoidance
safety_radius = 30;
detection_radius = 60;

% More cautious speed reduction
if dist_to_goal < 70
    uav_speed = 3 + (max_speed - 3) * (dist_to_goal / 70)^2;
end
```

## 📊 Output Report Structure

### Console Output Includes:

**Mission Statistics**:
- Total simulation time
- Distance traveled
- Average speed
- Final distance to goal (highlighted improvement)
- Final UAV speed

**Safety Metrics**:
- Minimum separation from all intruders
- Total avoidance maneuvers
- Direction changes (> 30°)
- Maximum/average maneuver angles

**Kalman Filter Performance**:
- Average estimation error
- Maximum estimation error
- Minimum estimation error

**Direction Change Analysis**:
- Detailed table of all heading changes
- UAV-intruder distances at each event
- Statistical summary per intruder
- Closest intruder identification
- Safety status classification

**Optimization Summary**:
- Confirms enabled features
- Performance improvements vs. base version

## 🔬 Algorithm Details

### Adaptive Control Law

```
Step 1: Calculate distance to goal
    d_goal = ‖goal - position‖

Step 2: Adaptive speed
    if d_goal < 50m:
        v_max = 5 + 15·(d_goal/50)²
    else:
        v_max = 20

Step 3: Desired velocity
    v_desired = (goal - position) / ‖goal - position‖ · v_max

Step 4: Collision avoidance
    for each intruder i:
        r_i = position_i - position_uav
        d_i = ‖r_i‖
        
        if d_i < detection_radius:
            repulsion_i = -r_i / d_i
            weight_i = f(d_i, uncertainty_i)
            v_avoid += repulsion_i · weight_i

Step 5: Goal attraction weight
    if d_goal < 20m:
        w_goal = 0.3 + 0.5·(1 - d_goal/20)
    else:
        w_goal = 0.3

Step 6: Blending
    α = α_base · (1 - w_goal)
    v_final = α·v_avoid + (1-α)·v_desired
    v_uav = v_final / ‖v_final‖ · v_max
```

### Ground Avoidance

```matlab
if altitude < 15m:
    ground_weight = 8.0 × (15 / altitude)
    v_avoid += [0, 0, 1] × ground_weight

if altitude < 5m:
    altitude = 5m  % Hard floor
    if v_z < 0:
        v_z = |v_z| + 5  % Bounce with boost
```

## 💾 Data Export

**CSV File**: `UAV_Direction_Changes_Optimized.csv`

Columns:
- `EventNum`: Sequential event ID
- `Time`: Timestamp (seconds)
- `HeadingChange`: Magnitude (degrees)
- `Dist_Int1` to `Dist_Int4`: Distance to each intruder (meters)
- `UAV_X`, `UAV_Y`, `UAV_Z`: Position at event (meters)

**Usage**: Import into Excel, Python, or R for post-processing analysis

## 🎯 Typical Performance

```
========== Simulation Complete ==========
Total time: 18.2 s
Total distance traveled: 287.3 m
Minimum distance to intruders: 21.4 m
Avoidance maneuvers: 34
Final distance to goal: 0.8 m ⭐ IMPROVED!

--- Kalman Filter Performance ---
Average estimation error: 3.82 m
Max estimation error: 11.67 m
=========================================
```

## 🔧 Requirements

- **MATLAB**: R2016b or later
- **Toolboxes**: None required
- **RAM**: 2GB minimum
- **Graphics**: Hardware acceleration recommended for smooth rendering

## 🎓 Educational Value

This simulation demonstrates:
- Sensor fusion with Kalman filtering
- Real-time motion planning
- Multi-objective optimization (goal + safety)
- Uncertainty quantification
- Adaptive control strategies
- 3D collision geometry

**Ideal for**: Robotics courses, UAV path planning research, autonomous systems education

## 🔮 Future Enhancements

### Algorithm Improvements
- [ ] Extended Kalman Filter (EKF) for nonlinear dynamics
- [ ] Unscented Kalman Filter (UKF) for better uncertainty handling
- [ ] Particle filters for multi-modal distributions
- [ ] Model Predictive Control (MPC) for trajectory optimization

### Scenario Complexity
- [ ] Wind field modeling
- [ ] Dynamic obstacles (moving terrain)
- [ ] No-fly zones (geofencing)
- [ ] Multi-UAV coordination
- [ ] Emergency landing sites

### Realism Features
- [ ] Actuator dynamics (thrust/attitude limits)
- [ ] Battery constraints (energy optimization)
- [ ] Communication delays
- [ ] Sensor occlusion modeling
- [ ] GPS denied navigation

## 📚 Key Formulas Reference

**Distance Calculation**:
```
d = √[(x₂-x₁)² + (y₂-y₁)² + (z₂-z₁)²]
```

**Kalman Gain**:
```
K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹
```

**State Update**:
```
x̂ = x̂⁻ + K·(z - H·x̂⁻)
```

**Repulsive Force**:
```
F = -k·(r/‖r‖)/d²
```

**Blended Control**:
```
v = α·v_avoid + (1-α)·v_goal
```

**Adaptive Speed**:
```
v = 5 + 15·(d/50)²  for d < 50m
```

## 📖 References

1. Kalman, R.E. (1960). "A New Approach to Linear Filtering and Prediction Problems"
2. LaValle, S.M. (2006). "Planning Algorithms" - Chapter 13
3. Khatib, O. (1986). "Real-Time Obstacle Avoidance for Manipulators and Mobile Robots"
4. Welch, G. & Bishop, G. (2006). "An Introduction to the Kalman Filter"

## 📄 License

MIT License - Free to use and modify with attribution.

## 👨‍💻 Author

Research code for advanced UAV collision avoidance studies.  
Optimized version: 2025

---

**Note**: This is the enhanced version with precision goal-reaching. For the base version, see `README.md`.
