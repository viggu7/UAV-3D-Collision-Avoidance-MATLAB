# Enhanced 3D Reactive Collision Avoidance with Attitude Tracking
## Complete Edition - Roll, Pitch, Yaw Analysis

A comprehensive MATLAB simulation for autonomous UAV navigation featuring Kalman filtering, collision avoidance, and complete 6-DOF attitude tracking (roll, pitch, yaw) for all aircraft in the scenario.

## 🆕 Major New Feature: Attitude Tracking

This version adds **full 6-DOF orientation tracking** for the UAV and all intruders, providing:

- **Roll Angle**: Bank angle during turns (rotation about longitudinal axis)
- **Pitch Angle**: Climb/descent angle (rotation about lateral axis)  
- **Yaw Angle**: Heading/compass direction (rotation about vertical axis)

### Why Attitude Matters

Attitude angles provide critical information about aircraft behavior:
- **Flight dynamics analysis**: Understanding how the UAV maneuvers
- **Collision prediction**: Anticipating future positions from orientation
- **Energy management**: Bank angle indicates turn tightness/energy cost
- **Realistic simulation**: Mimics actual flight control systems

## 📐 Attitude Calculation Method

### Mathematical Foundation

```matlab
function [roll, pitch, yaw] = calculate_attitude(velocity, acceleration)
```

**Inputs**:
- `velocity`: 3D velocity vector [vx, vy, vz] in m/s
- `acceleration`: 3D acceleration vector [ax, ay, az] in m/s²

**Outputs**:
- `roll`: Bank angle in radians (-π to π)
- `pitch`: Climb angle in radians (-π/2 to π/2)
- `yaw`: Heading in radians (-π to π)

### Calculation Formulas

**Yaw (Heading)**:
```
ψ = atan2(vy, vx)
```
- Compass direction in horizontal plane
- 0° = East, 90° = North, 180° = West, -90° = South

**Pitch (Climb Angle)**:
```
θ = atan2(vz, √(vx² + vy²))
```
- Positive = climbing
- Negative = descending
- Zero = level flight

**Roll (Bank Angle)**:
```
φ = atan2(a_lateral, g)

where:
  a_lateral = √(ax² + ay²)
  g = 9.81 m/s² (gravitational acceleration)
```
- Derived from coordinated turn physics
- In steady turn: tan(φ) = a_lateral / g
- Positive = right bank
- Negative = left bank

### Physical Interpretation

**Roll Angle**:
- 0° = Wings level
- +30° = Moderate right turn
- +60° = Steep right turn
- -45° = Aggressive left turn

**Pitch Angle**:
- 0° = Level flight
- +15° = Typical climb
- -10° = Descent
- ±90° = Vertical flight

**Yaw Angle**:
- 0° = Flying East
- 90° = Flying North
- 180° = Flying West
- -90° = Flying South

## 📊 Enhanced Data Tracking

### Direction Change Events with Attitude

When the UAV makes significant heading changes (>30°), the system now captures:

**UAV State**:
- Position [X, Y, Z]
- Attitude [Roll, Pitch, Yaw]
- Heading change magnitude
- Distance to goal

**All Intruder States**:
- Distance from UAV
- Attitude [Roll, Pitch, Yaw] for each intruder
- Relative geometry

### CSV Export Files

**1. Direction Change Summary**: `UAV_Attitude_Data.csv`
```
Columns:
- EventNum, Time, HeadingChange
- UAV_Roll, UAV_Pitch, UAV_Yaw
- Int1_Roll, Int1_Pitch, Int1_Yaw
- Int2_Roll, Int2_Pitch, Int2_Yaw
- Int3_Roll, Int3_Pitch, Int3_Yaw
- Int4_Roll, Int4_Pitch, Int4_Yaw
```

**2. Complete UAV History**: `UAV_Complete_Attitude_History.csv`
```
Columns: Time, Roll_deg, Pitch_deg, Yaw_deg
Frequency: Every 0.2 seconds
Duration: Entire simulation
```

**3. Individual Intruder Histories**: `Intruder1_Attitude_History.csv`, etc.
```
Columns: Time, Roll_deg, Pitch_deg, Yaw_deg
Separate file for each of 4 intruders
```

## 🎨 Enhanced Visualization

### Attitude Display in Labels

**UAV Label Format**:
```
UAV [45m]
R:12.3° P:5.7° Y:135.2°
```

**Intruder Label Format**:
```
I1 [65m]
R:0.0° P:-2.3° Y:180.0°
```

**Where**:
- Altitude in brackets
- R = Roll
- P = Pitch  
- Y = Yaw
- All angles in degrees

### Info Box Display

Located on bottom-left of screen:
```
╔═══ STATUS ═══╗
║ Dir Chg:   5
║ Speed: 18.5 m/s
║ Maneuv: 42.3°
║ To Goal: 124 m
║ Alt: 67 m
╚═════════════╝
```

### Label Positioning Strategy

To prevent overlap:
- UAV: Offset +8m to the right
- Goal: Offset -8m to the left
- Intruders: Different offsets per aircraft
  - Intruder 1: +8, 0
  - Intruder 2: -8, 0
  - Intruder 3: 0, +8
  - Intruder 4: 0, -8

## 📈 Output Report Structure

### Final Attitudes Section

```
─────────────────────────────────────────────────────
 FINAL ATTITUDES
─────────────────────────────────────────────────────
  UAV Final Attitude:
    Roll:    12.34 degrees
    Pitch:    5.67 degrees
    Yaw:    135.89 degrees

  Intruder Final Attitudes:
    Intruder 1: R:  0.00° P: -2.34° Y:180.00°
    Intruder 2: R:  8.45° P:  1.23° Y: 90.00°
    Intruder 3: R: -5.67° P:  3.45° Y:225.00°
    Intruder 4: R:  2.34° P: -1.11° Y:315.00°
```

### Direction Change Event Table with Attitude

```
┌──────┬────────┬───────────┬──────────────────────────────────────────────────────────────────┐
│ Event│ Time   │  Heading  │              UAV ATTITUDE (degrees)                              │
│  #   │  (s)   │ Change(°) │    Roll         Pitch          Yaw        Distance to Goal (m)  │
├──────┼────────┼───────────┼──────────────────────────────────────────────────────────────────┤
│   1  │   3.2  │   45.3    │   12.34        5.67         135.89              156.7           │
│   2  │   7.8  │   38.7    │   -8.91        3.45         178.23              124.3           │
│  ... │  ...   │   ...     │   ...          ...          ...                  ...            │
└──────┴────────┴───────────┴──────────────────────────────────────────────────────────────────┘
```

### Intruder Attitudes at Direction Changes

```
┌──────┬────────┬────────────────────────────────┬────────────────────────────────┬────────────────────────┐
│ Event│ Time   │      Intruder 1                │      Intruder 2                │      Intruder 3        │
│  #   │  (s)   │   Roll  Pitch   Yaw            │   Roll  Pitch   Yaw            │   Roll  Pitch   Yaw    │
├──────┼────────┼────────────────────────────────┼────────────────────────────────┼────────────────────────┤
│   1  │   3.2  │   0.00  -2.34  180.00          │   8.45   1.23   90.00          │  -5.67   3.45  225.00 │
│   2  │   7.8  │   1.23  -1.45  182.34          │   7.89   0.98   88.76          │  -4.32   3.21  228.90 │
│  ... │  ...   │   ...    ...    ...            │   ...    ...    ...            │   ...    ...    ...   │
└──────┴────────┴────────────────────────────────┴────────────────────────────────┴────────────────────────┘
```

## 🔬 Technical Implementation

### Attitude Update Loop

```matlab
% Store previous velocities for acceleration calculation
prev_uav_vel = uav_vel;
prev_intruder_vel = intruder_vel;

% Main simulation loop
for t = 0:dt:total_time
    
    % ... [position updates] ...
    
    % Calculate UAV acceleration
    uav_accel = (uav_vel - prev_uav_vel) / dt;
    
    % Update UAV attitude
    [uav_roll, uav_pitch, uav_yaw] = calculate_attitude(uav_vel, uav_accel);
    
    % Store in history
    uav_attitude_history = [uav_attitude_history; 
                            t, rad2deg(uav_roll), 
                            rad2deg(uav_pitch), 
                            rad2deg(uav_yaw)];
    
    % Update intruder attitudes
    for k = 1:n_intruders
        intruder_accel = (intruder_vel(k,:) - prev_intruder_vel(k,:)) / dt;
        [intruder_roll(k), intruder_pitch(k), intruder_yaw(k)] = ...
            calculate_attitude(intruder_vel(k,:), intruder_accel);
        intruder_attitude_history{k} = [intruder_attitude_history{k}; 
                                        t, rad2deg(intruder_roll(k)), 
                                        rad2deg(intruder_pitch(k)), 
                                        rad2deg(intruder_yaw(k))];
    end
    
    % Update previous velocities
    prev_uav_vel = uav_vel;
    prev_intruder_vel = intruder_vel;
end
```

### Data Storage Structure

**Attitude History Arrays**:
```matlab
% UAV: [time, roll, pitch, yaw] × N timesteps
uav_attitude_history = [];

% Intruders: Cell array of [time, roll, pitch, yaw] × N timesteps
intruder_attitude_history = cell(4, 1);
```

**Direction Change Data Structure**:
```matlab
direction_change_data = struct(
    'Time', {},
    'HeadingChange', {},
    'UAV_X', {}, 'UAV_Y', {}, 'UAV_Z', {},
    'UAV_Roll', {}, 'UAV_Pitch', {}, 'UAV_Yaw', {},
    'Int1_Roll', {}, 'Int1_Pitch', {}, 'Int1_Yaw', {},
    'Int2_Roll', {}, 'Int2_Pitch', {}, 'Int2_Yaw', {},
    'Int3_Roll', {}, 'Int3_Pitch', {}, 'Int3_Yaw', {},
    'Int4_Roll', {}, 'Int4_Pitch', {}, 'Int4_Yaw', {}
);
```

## 📊 Post-Processing Examples

### Python: Analyze Roll Angle During Maneuvers

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load attitude data
df = pd.read_csv('UAV_Attitude_Data.csv')

# Plot roll angle vs heading change
plt.figure(figsize=(10, 6))
plt.scatter(df['HeadingChange'], abs(df['UAV_Roll']), 
            c=df['Time'], cmap='viridis', s=100, alpha=0.7)
plt.colorbar(label='Time (s)')
plt.xlabel('Heading Change (degrees)')
plt.ylabel('Absolute Roll Angle (degrees)')
plt.title('UAV Bank Angle vs Course Correction')
plt.grid(True)
plt.show()

# Analyze turn aggressiveness
df['turn_rate'] = df['HeadingChange'] / df['Time'].diff()
correlation = df[['UAV_Roll', 'turn_rate']].corr()
print(f"Roll-TurnRate Correlation: {correlation.iloc[0,1]:.3f}")
```

### MATLAB: 3D Attitude Trajectory Plot

```matlab
% Load complete UAV history
data = readtable('UAV_Complete_Attitude_History.csv');

% Create 3D attitude space plot
figure;
plot3(data.Roll_deg, data.Pitch_deg, data.Yaw_deg, 'b-', 'LineWidth', 2);
xlabel('Roll (degrees)'); 
ylabel('Pitch (degrees)'); 
zlabel('Yaw (degrees)');
title('UAV Attitude Trajectory in Euler Space');
grid on;
view(45, 30);

% Mark start and end
hold on;
plot3(data.Roll_deg(1), data.Pitch_deg(1), data.Yaw_deg(1), ...
      'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
plot3(data.Roll_deg(end), data.Pitch_deg(end), data.Yaw_deg(end), ...
      'r^', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
legend('Attitude Path', 'Start', 'End');
```

### Python: Intruder Attitude Comparison

```python
import pandas as pd
import matplotlib.pyplot as plt

fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

for i in range(1, 5):
    df = pd.read_csv(f'Intruder{i}_Attitude_History.csv')
    
    axes[0].plot(df['Time'], df['Roll_deg'], label=f'Intruder {i}')
    axes[1].plot(df['Time'], df['Pitch_deg'], label=f'Intruder {i}')
    axes[2].plot(df['Time'], df['Yaw_deg'], label=f'Intruder {i}')

axes[0].set_ylabel('Roll (°)')
axes[0].set_title('Intruder Attitude Comparison')
axes[0].legend()
axes[0].grid(True)

axes[1].set_ylabel('Pitch (°)')
axes[1].grid(True)

axes[2].set_ylabel('Yaw (°)')
axes[2].set_xlabel('Time (s)')
axes[2].grid(True)

plt.tight_layout()
plt.show()
```

## 🎯 Core Features Summary

### From Previous Versions
✅ Kalman filtering for state estimation  
✅ Uncertainty-aware collision avoidance  
✅ Adaptive speed control near goal  
✅ Ground avoidance with altitude safety  
✅ Direction change detection (>30°)  
✅ Multiple intruder tracking  
✅ Real-time 3D visualization  
✅ Distance monitoring and analysis  

### New in Attitude Tracking Version
✨ **Full 6-DOF attitude tracking** (roll, pitch, yaw)  
✨ **Physics-based attitude calculation** from velocity/acceleration  
✨ **Attitude display in 3D labels**  
✨ **Complete attitude history export** (CSV)  
✨ **Direction change event attitude capture**  
✨ **Coordinated turn bank angle estimation**  
✨ **Per-aircraft attitude analysis**  

## 📐 Parameters

### Attitude Calculation
```matlab
g = 9.81;  % Gravitational acceleration (m/s²)
           % Used in coordinated turn formula
```

### Time Step
```matlab
dt = 0.2;  % Affects acceleration calculation accuracy
           % Smaller dt = smoother attitude estimates
```

## 🚀 Usage

### Basic Execution
```matlab
% Run simulation with attitude tracking
run('collision_avoidance_attitude.m')
```

### Customizing Attitude Sensitivity

**Reduce roll angle noise**:
```matlab
% In calculate_attitude function
if norm(acceleration) > 0.1  % Increase threshold from 0.01
    % ... calculate roll
else
    roll = 0;  % Ignore small accelerations
end
```

**Smooth attitude with filter**:
```matlab
% After attitude calculation
alpha_filter = 0.8;  % Smoothing factor
uav_roll = alpha_filter * uav_roll + (1-alpha_filter) * prev_roll;
```

## 📊 Typical Output

### Console Report

```
╔════════════════════════════════════════════════════╗
║   SIMULATION WITH ATTITUDE TRACKING - COMPLETE     ║
╚════════════════════════════════════════════════════╝

─────────────────────────────────────────────────────
 MISSION STATISTICS
─────────────────────────────────────────────────────
  Total time:                18.6 seconds
  Total distance traveled:   294.7 meters
  Average speed:             15.8 m/s
  Final distance to goal:    0.83 meters

─────────────────────────────────────────────────────
 FINAL ATTITUDES
─────────────────────────────────────────────────────
  UAV Final Attitude:
    Roll:    12.34 degrees
    Pitch:    5.67 degrees
    Yaw:    135.89 degrees

  Intruder Final Attitudes:
    Intruder 1: R:  0.00° P: -2.34° Y:180.00°
    Intruder 2: R:  8.45° P:  1.23° Y: 90.00°
    Intruder 3: R: -5.67° P:  3.45° Y:225.00°
    Intruder 4: R:  2.34° P: -1.11° Y:315.00°

─────────────────────────────────────────────────────
 EXPORTING COMPLETE ATTITUDE HISTORY
─────────────────────────────────────────────────────
✓ UAV complete attitude history: UAV_Complete_Attitude_History.csv
✓ Intruder 1 attitude history: Intruder1_Attitude_History.csv
✓ Intruder 2 attitude history: Intruder2_Attitude_History.csv
✓ Intruder 3 attitude history: Intruder3_Attitude_History.csv
✓ Intruder 4 attitude history: Intruder4_Attitude_History.csv

╔════════════════════════════════════════════════════╗
║  ATTITUDE DEFINITIONS:
║  • Roll:  Bank angle (rotation about X-axis)
║  • Pitch: Climb/descent angle (rotation about Y)
║  • Yaw:   Heading angle (rotation about Z-axis)
║  
║  All angles in degrees, range: -180° to +180°
╚════════════════════════════════════════════════════╝
```

## 🎓 Applications

### Research Applications
- **Flight dynamics validation**: Compare with real aircraft data
- **Control algorithm development**: Test autopilot strategies
- **Energy analysis**: Bank angle correlates with turn energy
- **Trajectory optimization**: Minimize aggressive maneuvers

### Educational Uses
- **Aircraft dynamics**: Visualize Euler angles in motion
- **Coordinated turns**: Understand bank angle physics
- **State estimation**: Track orientation from sensors
- **6-DOF simulation**: Full position + orientation

### Industrial Applications
- **UAV autopilot testing**: Validate flight control logic
- **Air traffic management**: Analyze conflict resolution maneuvers
- **Flight simulation**: Generate realistic scenario data
- **Sensor fusion**: Test IMU + GPS integration

## 🔮 Future Enhancements

### Attitude-Related
- [ ] Quaternion representation (avoid gimbal lock)
- [ ] Angular velocity tracking (ωx, ωy, ωz)
- [ ] Attitude constraints (max bank/pitch limits)
- [ ] Energy-optimal attitude control
- [ ] Coordinated vs. uncoordinated turn detection

### Advanced Physics
- [ ] Aerodynamic drag based on attitude
- [ ] Lift calculation from bank angle
- [ ] Stall detection at high pitch
- [ ] Wind effects on attitude
- [ ] Thrust vectoring control

### Visualization
- [ ] 3D aircraft models with orientation
- [ ] Attitude indicator (artificial horizon)
- [ ] Flight path vector overlay
- [ ] Angular rate display
- [ ] Animated attitude history playback

## 📚 Key Formulas

**Euler Angles from Velocity**:
```
Yaw:   ψ = atan2(vy, vx)
Pitch: θ = atan2(vz, √(vx² + vy²))
```

**Bank Angle from Turn**:
```
Roll:  φ = atan2(a_lateral, g)
where: a_lateral = √(ax² + ay²)
```

**Angular Difference** (handling wrap-around):
```
Δψ = angdiff(ψ_current, ψ_previous)
```

**Coordinated Turn Radius**:
```
R = v² / (g × tan(φ))
```

## 📖 References

1. **Stevens, B.L. & Lewis, F.L.** (2003). "Aircraft Control and Simulation" - Chapter 1: Equations of Motion
2. **Beard, R.W. & McLain, T.W.** (2012). "Small Unmanned Aircraft: Theory and Practice" - Chapter 3: Kinematics
3. **Kalman, R.E.** (1960). "A New Approach to Linear Filtering and Prediction Problems"
4. **Khatib, O.** (1986). "Real-Time Obstacle Avoidance for Manipulators and Mobile Robots"
5. **Etkin, B. & Reid, L.D.** (1996). "Dynamics of Flight: Stability and Control"

## 📄 License

MIT License - Free to use and modify with attribution.

## 👨‍💻 Author

Research code for UAV collision avoidance with comprehensive attitude tracking.  
Enhanced with 6-DOF orientation analysis: 2025

---

**Note**: This is the most comprehensive version with full attitude tracking. For simpler versions, see:
- `README.md` - Basic version
- `README_OPTIMIZED.md` - Goal precision version
- `README_ENHANCED.md` - Direction change tracking version
