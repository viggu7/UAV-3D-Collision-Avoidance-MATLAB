% ═══════════════════════════════════════════════════════════════════════════
% ENHANCED 3D REACTIVE COLLISION AVOIDANCE WITH KALMAN FILTERING
% WITH ATTITUDE TRACKING (ROLL, PITCH, YAW)
% 
% COMPREHENSIVE VERSION WITH DETAILED COMMENTS
% - Info box moved to left side to avoid graph overlap
% - Maneuver angle added to display
% - Major code sections heavily commented
% 
% Author: Enhanced for attitude tracking demonstration
% Date: 2025
% ═══════════════════════════════════════════════════════════════════════════

clear; clc; close all;

%% ═══════════════════════════════════════════════════════════════════════════
%  SECTION 1: SIMULATION TIME PARAMETERS
%  Purpose: Define the temporal resolution and duration of the simulation
% ═══════════════════════════════════════════════════════════════════════════
dt = 0.2;           % Time step (seconds) - Controls simulation update rate
total_time = 30;    % Total simulation duration (seconds)

%% ═══════════════════════════════════════════════════════════════════════════
%  SECTION 2: UAV CONFIGURATION
%  Purpose: Initialize the UAV's starting position, speed, and goal
% ═══════════════════════════════════════════════════════════════════════════
uav_pos = [0 0 20];             % Initial UAV position [x, y, z] in meters
uav_speed = 20;                 % UAV cruise speed (m/s)
uav_goal = [200 200 80];        % Goal/destination position [x, y, z] in meters
max_speed = uav_speed;          % Maximum allowable UAV speed (m/s)

% Calculate initial velocity vector pointing toward goal
uav_vel = (uav_goal - uav_pos) / norm(uav_goal - uav_pos) * uav_speed;

% Initialize trajectory history (stores all positions visited by UAV)
uav_traj = uav_pos;

% Initialize UAV attitude angles (orientation in 3D space)
uav_roll = 0;   % Bank angle (rotation about longitudinal axis)
uav_pitch = 0;  % Climb/descent angle (rotation about lateral axis)
uav_yaw = 0;    % Heading angle (rotation about vertical axis)

% Storage for complete attitude history over time
uav_attitude_history = [];

%% ═══════════════════════════════════════════════════════════════════════════
%  SECTION 3: INTRUDERS CONFIGURATION
%  Purpose: Define multiple moving obstacles (intruders) in the airspace
% ═══════════════════════════════════════════════════════════════════════════
% Initial positions of 4 intruders [x, y, z] in meters
intruder_pos = [0 200 55;       % Intruder 1
                200 100 85;     % Intruder 2
                120 220 65;     % Intruder 3
                220 120 95];    % Intruder 4

% Constant velocities of intruders [vx, vy, vz] in m/s
intruder_vel = [15 -5 0;        % Intruder 1 moving right and down
                -10 0 -2;       % Intruder 2 moving left and descending
                -12 -10 1;      % Intruder 3 moving left, down, and ascending
                -10 -12 -1];    % Intruder 4 moving left, down, and descending

n_intruders = size(intruder_pos, 1);  % Number of intruders (4)

% Initialize trajectory storage for each intruder (cell array)
intruder_traj = cell(n_intruders, 1);
for k = 1:n_intruders
    intruder_traj{k} = intruder_pos(k, :);
end

% Initialize attitude angles for all intruders
intruder_roll = zeros(n_intruders, 1);
intruder_pitch = zeros(n_intruders, 1);
intruder_yaw = zeros(n_intruders, 1);

% Storage for intruder attitude histories
intruder_attitude_history = cell(n_intruders, 1);
for k = 1:n_intruders
    intruder_attitude_history{k} = [];
end

%% ═══════════════════════════════════════════════════════════════════════════
%  SECTION 4: KALMAN FILTER INITIALIZATION
%  Purpose: Set up Kalman filters to estimate intruder positions/velocities
%           from noisy sensor measurements
% ═══════════════════════════════════════════════════════════════════════════
kf = cell(n_intruders, 1);  % Cell array to store Kalman filter for each intruder

for k = 1:n_intruders
    % STATE VECTOR: [x, y, z, vx, vy, vz]' - position and velocity
    kf{k}.x = [intruder_pos(k, :)'; intruder_vel(k, :)'];
    
    % COVARIANCE MATRIX: Uncertainty in state estimates
    % Higher values = more uncertainty initially
    kf{k}.P = diag([100 100 100 25 25 25]);
    
    % STATE TRANSITION MATRIX (F): Predicts next state using constant velocity model
    % x_new = x_old + vx*dt, y_new = y_old + vy*dt, etc.
    kf{k}.F = [1 0 0 dt 0 0;   % x position update
               0 1 0 0 dt 0;   % y position update
               0 0 1 0 0 dt;   % z position update
               0 0 0 1 0 0;    % vx remains constant
               0 0 0 0 1 0;    % vy remains constant
               0 0 0 0 0 1];   % vz remains constant
    
    % PROCESS NOISE COVARIANCE (Q): Accounts for model inaccuracies
    % Higher q = more process noise (intruders can deviate from constant velocity)
    q = 5;  % Process noise parameter
    kf{k}.Q = q * [dt^4/4 0 0 dt^3/2 0 0; 
                   0 dt^4/4 0 0 dt^3/2 0; 
                   0 0 dt^4/4 0 0 dt^3/2; 
                   dt^3/2 0 0 dt^2 0 0; 
                   0 dt^3/2 0 0 dt^2 0; 
                   0 0 dt^3/2 0 0 dt^2];
    
    % OBSERVATION MATRIX (H): Maps state to measurements (we only measure position)
    kf{k}.H = [1 0 0 0 0 0;   % Measure x position
               0 1 0 0 0 0;   % Measure y position
               0 0 1 0 0 0];  % Measure z position
    
    % MEASUREMENT NOISE COVARIANCE (R): Sensor measurement uncertainty
    kf{k}.R = 25 * eye(3);  % 25 m^2 variance in each axis
end

%% ═══════════════════════════════════════════════════════════════════════════
%  SECTION 5: SENSOR & SAFETY PARAMETERS
%  Purpose: Define detection ranges and safety zones
% ═══════════════════════════════════════════════════════════════════════════
sensor_range = 100;             % Maximum range UAV can detect intruders (m)
measurement_noise_std = 5;      % Standard deviation of sensor noise (m)
safety_radius = 25;             % Minimum safe distance from intruders (m)
detection_radius = 50;          % Range at which avoidance maneuvers begin (m)

%% ═══════════════════════════════════════════════════════════════════════════
%  SECTION 6: PERFORMANCE TRACKING VARIABLES
%  Purpose: Monitor and record simulation metrics
% ═══════════════════════════════════════════════════════════════════════════
min_distance = inf;             % Track closest approach to any intruder
collision_events = 0;           % Count safety radius violations
total_distance = 0;             % Total distance traveled by UAV
avoidance_count = 0;            % Number of timesteps with active avoidance
direction_changes = 0;          % Number of significant heading changes

% Track heading for direction change detection
previous_heading = atan2(uav_vel(2), uav_vel(1));

% Preallocate arrays for performance data
estimation_errors = zeros(total_time/dt + 1, n_intruders);
maneuver_angles = zeros(total_time/dt + 1, 1);
maneuver_angles_deg = zeros(total_time/dt + 1, 1);
time_vec = 0:dt:total_time;
time_idx = 1;
current_maneuver_angle = 0;
current_maneuver_angle_deg = 0;
uav_intruder_distances = zeros(total_time/dt + 1, n_intruders);

%% ═══════════════════════════════════════════════════════════════════════════
%  SECTION 6B: DIRECTION CHANGE TRACKING WITH ATTITUDE
%  Purpose: Store detailed data whenever UAV makes significant course changes
% ═══════════════════════════════════════════════════════════════════════════
direction_change_data = struct('Time', {}, 'HeadingChange', {}, ...
                               'Dist_Int1', {}, 'Dist_Int2', {}, ...
                               'Dist_Int3', {}, 'Dist_Int4', {}, ...
                               'UAV_X', {}, 'UAV_Y', {}, 'UAV_Z', {}, ...
                               'UAV_Roll', {}, 'UAV_Pitch', {}, 'UAV_Yaw', {}, ...
                               'Int1_Roll', {}, 'Int1_Pitch', {}, 'Int1_Yaw', {}, ...
                               'Int2_Roll', {}, 'Int2_Pitch', {}, 'Int2_Yaw', {}, ...
                               'Int3_Roll', {}, 'Int3_Pitch', {}, 'Int3_Yaw', {}, ...
                               'Int4_Roll', {}, 'Int4_Pitch', {}, 'Int4_Yaw', {});
dc_count = 0;

%% ═══════════════════════════════════════════════════════════════════════════
%  ATTITUDE CALCULATION FUNCTION
%  Purpose: Calculate roll, pitch, yaw angles from velocity and acceleration
%           This gives the orientation of the aircraft in 3D space
% ═══════════════════════════════════════════════════════════════════════════
function [roll, pitch, yaw] = calculate_attitude(velocity, acceleration)
    % INPUTS:
    %   velocity: [vx, vy, vz] - velocity vector in m/s
    %   acceleration: [ax, ay, az] - acceleration vector in m/s^2
    % OUTPUTS:
    %   roll: bank angle in radians (rotation about x-axis)
    %   pitch: climb angle in radians (rotation about y-axis)
    %   yaw: heading angle in radians (rotation about z-axis)
    
    vx = velocity(1); 
    vy = velocity(2); 
    vz = velocity(3);
    v_mag = norm(velocity);
    
    % Handle near-zero velocity (hovering/stationary)
    if v_mag < 0.1
        roll = 0; pitch = 0; yaw = 0;
        return;
    end
    
    % YAW: Heading angle in horizontal (XY) plane
    % Calculated using arctangent of velocity components
    yaw = atan2(vy, vx);  % Range: -π to π radians
    
    % PITCH: Angle of climb (+) or descent (-)
    % Calculated from vertical velocity component
    pitch = atan2(vz, sqrt(vx^2 + vy^2));
    
    % ROLL: Bank angle during turns
    % Estimated from lateral acceleration (centripetal)
    if length(acceleration) == 3 && norm(acceleration) > 0.01
        ax = acceleration(1); 
        ay = acceleration(2);
        
        % Lateral acceleration magnitude in horizontal plane
        a_lateral = sqrt(ax^2 + ay^2);
        
        % Bank angle from coordinated turn physics
        % In a coordinated turn: tan(roll) = a_lateral / g
        g = 9.81;  % Gravitational acceleration (m/s^2)
        roll = atan2(a_lateral, g);
    else
        roll = 0;  % No roll if acceleration is negligible
    end
end

%% ═══════════════════════════════════════════════════════════════════════════
%  SECTION 7: MAIN SIMULATION LOOP
%  Purpose: Iterate through time, updating positions and performing avoidance
% ═══════════════════════════════════════════════════════════════════════════
figure('Position', [50 50 1800 1000]);

% Store previous velocities for acceleration calculation
prev_uav_vel = uav_vel;
prev_intruder_vel = intruder_vel;

for t = 0:dt:total_time
    % Check if UAV reached goal (within 1.5m tolerance)
    if norm(uav_goal - uav_pos) < 1.5
        fprintf('✓ Goal Reached at t = %.1f seconds!\n', t);
        break;
    end
    
    %% ═══════════════════════════════════════════════════════════════════════
    %  STEP 1: SIMULATE SENSOR MEASUREMENTS
    %  Purpose: Generate noisy position measurements for intruders in range
    %% ═══════════════════════════════════════════════════════════════════════
    measurements = cell(n_intruders, 1);
    measured_positions = nan(n_intruders, 3);
    
    for k = 1:n_intruders
        % Calculate range from UAV to intruder
        range_to_intruder = norm(intruder_pos(k, :) - uav_pos);
        
        % Only measure if within sensor range
        if range_to_intruder < sensor_range
            % Add Gaussian noise to true position
            noise = measurement_noise_std * randn(1, 3);
            measurements{k} = intruder_pos(k, :)' + noise';
            measured_positions(k, :) = measurements{k}';
        else
            % Out of range - no measurement available
            measurements{k} = [];
        end
    end
    
    %% ═══════════════════════════════════════════════════════════════════════
    %  STEP 2: KALMAN FILTER UPDATE
    %  Purpose: Fuse predictions with measurements to estimate true states
    %           This is the core of the tracking algorithm
    %% ═══════════════════════════════════════════════════════════════════════
    estimated_pos = zeros(n_intruders, 3);
    estimated_vel = zeros(n_intruders, 3);
    covariances = zeros(n_intruders, 1);
    
    for k = 1:n_intruders
        % PREDICT STEP: Use motion model to predict next state
        kf{k}.x = kf{k}.F * kf{k}.x;  % x_predicted = F * x_previous
        kf{k}.P = kf{k}.F * kf{k}.P * kf{k}.F' + kf{k}.Q;  % Propagate uncertainty
        
        % UPDATE STEP: Correct prediction using measurement (if available)
        if ~isempty(measurements{k})
            % Innovation (measurement residual)
            y = measurements{k} - kf{k}.H * kf{k}.x;
            
            % Innovation covariance
            S = kf{k}.H * kf{k}.P * kf{k}.H' + kf{k}.R;
            
            % Kalman Gain (optimal weighting between prediction and measurement)
            K = kf{k}.P * kf{k}.H' / S;
            
            % State update: x_new = x_predicted + K * innovation
            kf{k}.x = kf{k}.x + K * y;
            
            % Covariance update: uncertainty decreases after measurement
            kf{k}.P = (eye(6) - K * kf{k}.H) * kf{k}.P;
        end
        
        % Extract estimated position and velocity from state vector
        estimated_pos(k, :) = kf{k}.x(1:3)';
        estimated_vel(k, :) = kf{k}.x(4:6)';
        
        % Store total uncertainty (trace of position covariance)
        covariances(k) = trace(kf{k}.P(1:3, 1:3));
        
        % Record estimation error (for performance analysis)
        estimation_errors(time_idx, k) = norm(estimated_pos(k, :) - intruder_pos(k, :));
        
        % Record actual distance to intruder
        uav_intruder_distances(time_idx, k) = norm(intruder_pos(k, :) - uav_pos);
    end
    
    %% ═══════════════════════════════════════════════════════════════════════
    %  STEP 3: CALCULATE DESIRED VELOCITY WITH ADAPTIVE SPEED
    %  Purpose: Slow down as UAV approaches goal for smooth arrival
    %% ═══════════════════════════════════════════════════════════════════════
    dist_to_goal = norm(uav_goal - uav_pos);
    
    % Adaptive speed reduction near goal
    if dist_to_goal < 50
        % Quadratic slowdown within 50m of goal
        speed_factor = (dist_to_goal / 50)^2;
        uav_speed = 5 + (max_speed - 5) * speed_factor;  % Min 5 m/s, max 20 m/s
    else
        uav_speed = max_speed;  % Full speed when far from goal
    end
    
    % Calculate desired velocity vector pointing toward goal
    desired_vel = (uav_goal - uav_pos);
    if norm(desired_vel) > 0
        desired_vel = desired_vel / norm(desired_vel) * uav_speed;
    end
    
    %% ═══════════════════════════════════════════════════════════════════════
    %  STEP 4: COLLISION AVOIDANCE LOGIC
    %  Purpose: Calculate repulsive forces to avoid intruders and ground
    %           This is the main collision avoidance algorithm
    %% ═══════════════════════════════════════════════════════════════════════
    avoid = false;              % Flag indicating avoidance maneuver needed
    avoidance_vec = [0 0 0];    % Accumulated avoidance vector
    closest_dist = inf;         % Track closest intruder distance
    
    % ─────────────────────────────────────────────────────────────────────
    % GROUND AVOIDANCE: Prevent UAV from flying too low
    % ─────────────────────────────────────────────────────────────────────
    ground_safety_altitude = 15.0;  % Minimum safe altitude (m)
    
    if uav_pos(3) < ground_safety_altitude
        avoid = true;
        ground_dist = uav_pos(3);
        
        % Strong upward repulsion inversely proportional to altitude
        ground_weight = 8.0 * (ground_safety_altitude / (ground_dist + 1e-6));
        avoidance_vec = avoidance_vec + [0 0 1] * ground_weight;  % Upward force
    end
    
    % ─────────────────────────────────────────────────────────────────────
    % INTRUDER AVOIDANCE: Avoid collisions with moving obstacles
    % ─────────────────────────────────────────────────────────────────────
    for k = 1:n_intruders
        % Vector from UAV to estimated intruder position
        r = estimated_pos(k, :) - uav_pos;
        d = norm(r);  % Distance to intruder
        
        closest_dist = min(closest_dist, d);
        
        % Also track minimum distance using true positions
        true_dist = norm(intruder_pos(k, :) - uav_pos);
        if true_dist < min_distance
            min_distance = true_dist;
        end
        
        % Adjust safety zones based on estimation uncertainty
        uncertainty = sqrt(covariances(k));
        effective_safety = safety_radius + uncertainty;      % Expand safety zone
        effective_detection = detection_radius + uncertainty; % Expand detection zone
        
        % Check if intruder is within detection range
        if d < effective_detection
            avoid = true;  % Trigger avoidance behavior
            
            % Calculate repulsion direction (away from intruder)
            repulsion = -r / (d + 1e-6);  % Normalized vector pointing away
            
            % Calculate repulsion strength based on proximity
            if d < effective_safety
                % CRITICAL: Inside safety radius - strong repulsion
                weight = 5.0 * (effective_safety / (d + 1e-6));
                collision_events = collision_events + 1;
            else
                % WARNING: Inside detection radius - moderate repulsion
                % Weight decreases linearly with distance
                weight = 2.0 * (effective_detection - d) / effective_detection;
            end
            
            % Scale weight by estimation confidence
            % More uncertain estimates get lower weight
            confidence = exp(-uncertainty / 10);
            weight = weight * confidence;
            
            % Add weighted repulsion to total avoidance vector
            avoidance_vec = avoidance_vec + repulsion * weight;
            
            % Add perpendicular component for smoother avoidance
            % This makes UAV go "around" obstacles instead of just backing away
            perp = cross(r, desired_vel);  % Perpendicular to approach vector
            if norm(perp) > 1e-6
                perp = perp / norm(perp);
                avoidance_vec = avoidance_vec + perp * weight * 0.5;
            end
        end
    end
    
    %% ═══════════════════════════════════════════════════════════════════════
    %  STEP 5: BLENDED CONTROL WITH GOAL ATTRACTION
    %  Purpose: Combine avoidance vector with goal-seeking behavior
    %           Balance between safety and progress toward goal
    %% ═══════════════════════════════════════════════════════════════════════
    if avoid
        avoidance_count = avoidance_count + 1;
        
        % Normalize avoidance vector
        avoidance_vec = avoidance_vec / (norm(avoidance_vec) + 1e-6);
        
        % Increase goal attraction when very close to goal
        if dist_to_goal < 20
            goal_weight = 0.3 + 0.5 * (1 - dist_to_goal/20);  % Up to 0.8
        else
            goal_weight = 0.3;  % Standard goal weight
        end
        
        % Adjust blend factor (alpha) based on threat level
        if closest_dist < safety_radius
            % CRITICAL: Prioritize avoidance (95% avoidance)
            alpha = 0.95 * (1 - goal_weight);
        elseif closest_dist < detection_radius
            % WARNING: Moderate avoidance priority (70% avoidance)
            alpha = 0.70 * (1 - goal_weight);
        else
            % CAUTION: Balanced approach (50% avoidance)
            alpha = 0.5 * (1 - goal_weight);
        end
        
        % Blend avoidance and goal-seeking velocities
        % uav_vel = alpha * (avoidance) + (1-alpha) * (goal-seeking)
        uav_vel = alpha * avoidance_vec * uav_speed + (1 - alpha) * desired_vel;
        
        % Ensure velocity magnitude equals desired speed
        vel_mag = norm(uav_vel);
        if vel_mag > 0
            uav_vel = uav_vel / vel_mag * uav_speed;
        end
    else
        % No obstacles detected - fly directly toward goal
        uav_vel = desired_vel;
        vel_mag = norm(uav_vel);
        
        % Limit to maximum speed
        if vel_mag > uav_speed
            uav_vel = uav_vel / vel_mag * uav_speed;
        end
    end
    
    %% ═══════════════════════════════════════════════════════════════════════
    %  STEP 5B: CALCULATE ATTITUDES
    %  Purpose: Determine orientation angles for all aircraft
    %% ═══════════════════════════════════════════════════════════════════════
    
    % UAV attitude from velocity and acceleration
    uav_accel = (uav_vel - prev_uav_vel) / dt;
    [uav_roll, uav_pitch, uav_yaw] = calculate_attitude(uav_vel, uav_accel);
    uav_attitude_history = [uav_attitude_history; t, rad2deg(uav_roll), rad2deg(uav_pitch), rad2deg(uav_yaw)];
    
    % Intruder attitudes
    for k = 1:n_intruders
        intruder_accel = (intruder_vel(k, :) - prev_intruder_vel(k, :)) / dt;
        [intruder_roll(k), intruder_pitch(k), intruder_yaw(k)] = calculate_attitude(intruder_vel(k, :), intruder_accel);
        intruder_attitude_history{k} = [intruder_attitude_history{k}; t, rad2deg(intruder_roll(k)), rad2deg(intruder_pitch(k)), rad2deg(intruder_yaw(k))];
    end
    
    %% ═══════════════════════════════════════════════════════════════════════
    %  STEP 6: CALCULATE MANEUVER ANGLE
    %  Purpose: Measure deviation from direct path to goal
    %% ═══════════════════════════════════════════════════════════════════════
    if norm(desired_vel) > 0 && norm(uav_vel) > 0
        % Angle between actual velocity and desired velocity
        cos_angle = dot(uav_vel, desired_vel) / (norm(uav_vel) * norm(desired_vel));
        cos_angle = max(-1, min(1, cos_angle));  % Clamp to valid range
        current_maneuver_angle = acos(cos_angle);  % Radians
        current_maneuver_angle_deg = rad2deg(current_maneuver_angle);  % Degrees
    else
        current_maneuver_angle = 0;
        current_maneuver_angle_deg = 0;
    end
    
    % Store maneuver angle history
    maneuver_angles(time_idx) = current_maneuver_angle;
    maneuver_angles_deg(time_idx) = current_maneuver_angle_deg;
    
    %% ═══════════════════════════════════════════════════════════════════════
    %  STEP 7: TRACK DIRECTION CHANGES WITH ATTITUDE
    %  Purpose: Record significant course corrections
    %% ═══════════════════════════════════════════════════════════════════════
    current_heading = atan2(uav_vel(2), uav_vel(1));
    heading_change = abs(rad2deg(angdiff(current_heading, previous_heading)));
    
    % Detect significant direction changes (>30 degrees)
    if heading_change > 30
        direction_changes = direction_changes + 1;
        dc_count = dc_count + 1;
        
        % Calculate distances to all intruders at this moment
        dist_to_intruders = zeros(1, n_intruders);
        for k = 1:n_intruders
            dist_to_intruders(k) = norm(intruder_pos(k, :) - uav_pos);
        end
        
        % Store complete state information for this direction change event
        direction_change_data(dc_count).Time = t;
        direction_change_data(dc_count).HeadingChange = heading_change;
        direction_change_data(dc_count).Dist_Int1 = dist_to_intruders(1);
        direction_change_data(dc_count).Dist_Int2 = dist_to_intruders(2);
        direction_change_data(dc_count).Dist_Int3 = dist_to_intruders(3);
        direction_change_data(dc_count).Dist_Int4 = dist_to_intruders(4);
        direction_change_data(dc_count).UAV_X = uav_pos(1);
        direction_change_data(dc_count).UAV_Y = uav_pos(2);
        direction_change_data(dc_count).UAV_Z = uav_pos(3);
        direction_change_data(dc_count).UAV_Roll = rad2deg(uav_roll);
        direction_change_data(dc_count).UAV_Pitch = rad2deg(uav_pitch);
        direction_change_data(dc_count).UAV_Yaw = rad2deg(uav_yaw);
        direction_change_data(dc_count).Int1_Roll = rad2deg(intruder_roll(1));
        direction_change_data(dc_count).Int1_Pitch = rad2deg(intruder_pitch(1));
        direction_change_data(dc_count).Int1_Yaw = rad2deg(intruder_yaw(1));
        direction_change_data(dc_count).Int2_Roll = rad2deg(intruder_roll(2));
        direction_change_data(dc_count).Int2_Pitch = rad2deg(intruder_pitch(2));
        direction_change_data(dc_count).Int2_Yaw = rad2deg(intruder_yaw(2));
        direction_change_data(dc_count).Int3_Roll = rad2deg(intruder_roll(3));
        direction_change_data(dc_count).Int3_Pitch = rad2deg(intruder_pitch(3));
        direction_change_data(dc_count).Int3_Yaw = rad2deg(intruder_yaw(3));
        direction_change_data(dc_count).Int4_Roll = rad2deg(intruder_roll(4));
        direction_change_data(dc_count).Int4_Pitch = rad2deg(intruder_pitch(4));
        direction_change_data(dc_count).Int4_Yaw = rad2deg(intruder_yaw(4));
    end
    
    previous_heading = current_heading;
    
    %% ═══════════════════════════════════════════════════════════════════════
    %  STEP 8: UPDATE POSITIONS
    %  Purpose: Integrate velocities to get new positions
    %% ═══════════════════════════════════════════════════════════════════════
    prev_pos = uav_pos;
    prev_uav_vel = uav_vel;
    prev_intruder_vel = intruder_vel;
    
    % Update UAV position using simple Euler integration
    uav_pos = uav_pos + uav_vel * dt;
    
    % GROUND COLLISION PREVENTION FOR UAV
    min_altitude = 5.0;
    if uav_pos(3) < min_altitude
        uav_pos(3) = min_altitude;  % Hard floor
        if uav_vel(3) < 0
            uav_vel(3) = abs(uav_vel(3)) + 5;  % Bounce upward
        end
    end
    
    % Accumulate total distance traveled
    total_distance = total_distance + norm(uav_pos - prev_pos);
    
    % Update intruder positions
    intruder_pos = intruder_pos + intruder_vel * dt;
    
    % GROUND COLLISION PREVENTION FOR INTRUDERS
    min_intruder_altitude = 20.0;
    for k = 1:n_intruders
        if intruder_pos(k, 3) < min_intruder_altitude
            intruder_pos(k, 3) = min_intruder_altitude;
            if intruder_vel(k, 3) < 0
                intruder_vel(k, 3) = -intruder_vel(k, 3);  % Reverse vertical velocity
            end
        end
    end
    
    % Store trajectory points
    uav_traj = [uav_traj; uav_pos];
    for k = 1:n_intruders
        intruder_traj{k} = [intruder_traj{k}; intruder_pos(k, :)];
    end
    
    time_idx = time_idx + 1;
    
    %% ═══════════════════════════════════════════════════════════════════════
    %  STEP 9: REAL-TIME VISUALIZATION
    %  Purpose: Display 3D view, top view, and distance plots
    %% ═══════════════════════════════════════════════════════════════════════
    clf;
    
    % ─────────────────────────────────────────────────────────────────────
    % MAIN 3D VIEW (takes up left 2/3 of figure)
    % ─────────────────────────────────────────────────────────────────────
    subplot(2, 3, [1 2 4 5]);
    hold on; grid on;
    
    % Draw ground plane with mesh
    [X_ground, Y_ground] = meshgrid(0:50:250, 0:50:250);
    Z_ground = zeros(size(X_ground));
    surf(X_ground, Y_ground, Z_ground, 'FaceAlpha', 0.1, 'EdgeColor', [0.7 0.7 0.7], ...
         'FaceColor', [0.9 0.9 0.85], 'EdgeAlpha', 0.3);
    
    % Plot trajectories (paths traveled)
    plot3(uav_traj(:, 1), uav_traj(:, 2), uav_traj(:, 3), 'b-', 'LineWidth', 2.5);
    for k = 1:n_intruders
        plot3(intruder_traj{k}(:, 1), intruder_traj{k}(:, 2), intruder_traj{k}(:, 3), 'r--', 'LineWidth', 1.5);
    end
    
    % ─────────────────────────────────────────────────────────────────────
    % UAV VISUALIZATION with altitude line and clean label
    % ─────────────────────────────────────────────────────────────────────
    plot3(uav_pos(1), uav_pos(2), uav_pos(3), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 14);
    plot3([uav_pos(1) uav_pos(1)], [uav_pos(2) uav_pos(2)], [0 uav_pos(3)], 'b:', 'LineWidth', 1.5);
    plot3(uav_pos(1), uav_pos(2), 0, 'bx', 'MarkerSize', 10, 'LineWidth', 2);
    
    % UAV label offset to right
    text(uav_pos(1) + 8, uav_pos(2), uav_pos(3) + 3, ...
         sprintf('UAV [%.0fm]\nR:%.0f° P:%.0f° Y:%.0f°', uav_pos(3), rad2deg(uav_roll), rad2deg(uav_pitch), rad2deg(uav_yaw)), ...
         'FontSize', 8, 'Color', [0 0 0.8], ...
         'HorizontalAlignment', 'left', 'BackgroundColor', [1 1 1 0.85], 'EdgeColor', [0.3 0.3 1], 'LineWidth', 0.5);
    
    % ─────────────────────────────────────────────────────────────────────
    % INTRUDER VISUALIZATION with different colors and offset labels
    % ─────────────────────────────────────────────────────────────────────
    intruder_colors = {[0.8 0 0], [0.9 0.4 0], [0.7 0 0.3], [0.6 0 0.5]};
    label_offsets = [8 0; -8 0; 0 8; 0 -8];  % Different offset for each intruder
    
    for k = 1:n_intruders
        % Intruder marker and altitude line
        plot3(intruder_pos(k, 1), intruder_pos(k, 2), intruder_pos(k, 3), ...
              'o', 'MarkerFaceColor', intruder_colors{k}, 'MarkerEdgeColor', 'k', ...
              'MarkerSize', 12, 'LineWidth', 1.5);
        plot3([intruder_pos(k, 1) intruder_pos(k, 1)], ...
              [intruder_pos(k, 2) intruder_pos(k, 2)], ...
              [0 intruder_pos(k, 3)], ':', 'Color', intruder_colors{k}, 'LineWidth', 1.5);
        plot3(intruder_pos(k, 1), intruder_pos(k, 2), 0, 'x', ...
              'Color', intruder_colors{k}, 'MarkerSize', 10, 'LineWidth', 2);
        
        % Compact label with offset to prevent overlap
        text(intruder_pos(k, 1) + label_offsets(k, 1), intruder_pos(k, 2) + label_offsets(k, 2), intruder_pos(k, 3) + 2, ...
             sprintf('I%d [%.0fm]\nR:%.0f P:%.0f Y:%.0f', k, intruder_pos(k, 3), ...
                     rad2deg(intruder_roll(k)), rad2deg(intruder_pitch(k)), rad2deg(intruder_yaw(k))), ...
             'FontSize', 7, 'Color', intruder_colors{k}, ...
             'HorizontalAlignment', 'center', 'BackgroundColor', [1 1 1 0.8], 'EdgeColor', intruder_colors{k}, 'LineWidth', 0.5);
        
        % Velocity arrow showing intruder movement direction
        quiver3(intruder_pos(k, 1), intruder_pos(k, 2), intruder_pos(k, 3), ...
                intruder_vel(k, 1)*2, intruder_vel(k, 2)*2, intruder_vel(k, 3)*2, ...
                'Color', intruder_colors{k}, 'LineWidth', 2, 'MaxHeadSize', 1.5);
    end
    
    % Estimated positions from Kalman filter (purple squares)
    plot3(estimated_pos(:, 1), estimated_pos(:, 2), estimated_pos(:, 3), ...
          'ms', 'MarkerSize', 10, 'LineWidth', 2);
    
    % ─────────────────────────────────────────────────────────────────────
    % GOAL VISUALIZATION
    % ─────────────────────────────────────────────────────────────────────
    plot3(uav_goal(1), uav_goal(2), uav_goal(3), 'g^', ...
          'MarkerSize', 18, 'LineWidth', 3, 'MarkerFaceColor', 'g');
    plot3([uav_goal(1) uav_goal(1)], [uav_goal(2) uav_goal(2)], [0 uav_goal(3)], 'g:', 'LineWidth', 1.5);
    text(uav_goal(1) - 8, uav_goal(2), uav_goal(3) + 3, ...
         sprintf('GOAL [%.0fm]\nDist: %.0fm', uav_goal(3), dist_to_goal), ...
         'FontSize', 8, 'FontWeight', 'bold', 'Color', [0 0.6 0], ...
         'HorizontalAlignment', 'right', 'BackgroundColor', [1 1 1 0.85], 'EdgeColor', [0 0.8 0], 'LineWidth', 0.5);
    
    % Safety sphere around UAV (cyan transparent bubble)
    [xs, ys, zs] = sphere(20);
    surf(uav_pos(1) + safety_radius * xs, uav_pos(2) + safety_radius * ys, uav_pos(3) + safety_radius * zs, ...
         'FaceAlpha', 0.08, 'EdgeColor', 'none', 'FaceColor', 'cyan');
    
    % ─────────────────────────────────────────────────────────────────────
    % VELOCITY VECTORS
    % ─────────────────────────────────────────────────────────────────────
    % Blue arrow: actual UAV velocity
    quiver3(uav_pos(1), uav_pos(2), uav_pos(3), uav_vel(1)*3, uav_vel(2)*3, uav_vel(3)*3, ...
            'b', 'LineWidth', 3, 'MaxHeadSize', 2);
    
    % Green dashed arrow: desired velocity (toward goal)
    quiver3(uav_pos(1), uav_pos(2), uav_pos(3), desired_vel(1)*3, desired_vel(2)*3, desired_vel(3)*3, ...
            'g--', 'LineWidth', 2, 'MaxHeadSize', 1.5);
    
    % Axis labels and title
    xlabel('X (m)', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Y (m)', 'FontSize', 11, 'FontWeight', 'bold');
    zlabel('Altitude Z (m)', 'FontSize', 11, 'FontWeight', 'bold');
    title(sprintf('3D UAV Collision Avoidance | t = %.1f s | Speed: %.1f m/s', t, uav_speed), ...
          'FontSize', 12, 'FontWeight', 'bold');
    
    axis equal; xlim([0 250]); ylim([0 250]); zlim([0 120]); 
    view(45, 30);  % 3D viewing angle
    
    % Compact legend
    legend({'Trajectory', 'Intruder Path', 'UAV', '', '', 'Goal', '', '', 'Safety Zone', 'UAV Velocity', 'Desired Vel'}, ...
           'Location', 'northeast', 'FontSize', 7);
    
    % ─────────────────────────────────────────────────────────────────────
    % TOP VIEW (bird's eye view in XY plane)
    % ─────────────────────────────────────────────────────────────────────
    subplot(2, 3, 3);
    hold on; grid on;
    
    % Plot 2D trajectories
    plot(uav_traj(:, 1), uav_traj(:, 2), 'b-', 'LineWidth', 2);
    for k = 1:n_intruders
        plot(intruder_traj{k}(:, 1), intruder_traj{k}(:, 2), '--', ...
             'Color', intruder_colors{k}, 'LineWidth', 1.5);
    end
    
    % Current positions
    plot(uav_pos(1), uav_pos(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 12);
    for k = 1:n_intruders
        plot(intruder_pos(k, 1), intruder_pos(k, 2), 'o', ...
             'MarkerFaceColor', intruder_colors{k}, 'MarkerEdgeColor', 'k', ...
             'MarkerSize', 10, 'LineWidth', 1.5);
        text(intruder_pos(k, 1), intruder_pos(k, 2) - 5, sprintf('I%d', k), ...
             'FontSize', 7, 'HorizontalAlignment', 'center', 'Color', intruder_colors{k}, 'FontWeight', 'bold');
    end
    
    plot(uav_goal(1), uav_goal(2), 'g^', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
    
    % Safety circle (top view)
    theta_circle = linspace(0, 2*pi, 50);
    circle_x = uav_pos(1) + safety_radius * cos(theta_circle);
    circle_y = uav_pos(2) + safety_radius * sin(theta_circle);
    plot(circle_x, circle_y, 'c-', 'LineWidth', 1.5);
    
    xlabel('X (m)', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel('Y (m)', 'FontSize', 10, 'FontWeight', 'bold');
    title('TOP VIEW (XY)', 'FontSize', 12, 'FontWeight', 'bold');
    axis equal; xlim([0 250]); ylim([0 250]);
    
    % ─────────────────────────────────────────────────────────────────────
    % DISTANCE PLOT (UAV to each intruder over time)
    % ─────────────────────────────────────────────────────────────────────
    subplot(2, 3, 6);
    hold on; grid on;
    
    plot_colors = {'b', 'r', 'g', 'm'};
    for k = 1:n_intruders
        plot(time_vec(1:time_idx-1), uav_intruder_distances(1:time_idx-1, k), ...
             [plot_colors{k} '-'], 'LineWidth', 2, 'DisplayName', sprintf('Int %d', k));
    end
    
    % Reference lines for safety zones
    yline(safety_radius, 'r--', 'LineWidth', 2, 'Label', 'Safety');
    yline(detection_radius, 'y--', 'LineWidth', 1.5, 'Label', 'Detection');
    
    xlabel('Time (s)', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel('Distance (m)', 'FontSize', 10, 'FontWeight', 'bold');
    title('UAV-INTRUDER DISTANCES', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([0 total_time]); 
    ylim([0 max(150, max(uav_intruder_distances(:))+10)]);
    legend('Location', 'northeast', 'FontSize', 8);
    
    % ═════════════════════════════════════════════════════════════════════
    % INFO BOX - White text on transparent background for dark theme
    % Positioned on left side to avoid overlap with graph
    % ═════════════════════════════════════════════════════════════════════
    annotation('textbox', [0.515 0.01 0.16 0.20], 'String', ...
               sprintf('╔═══ STATUS ═══╗\n║ Dir Chg:  %2d\n║ Speed: %4.1f m/s\n║ Maneuv: %4.1f°\n║ To Goal: %3.0f m\n║ Alt: %3.0f m\n╚═════════════╝', ...
                       direction_changes, uav_speed, current_maneuver_angle_deg, dist_to_goal, uav_pos(3)), ...
               'FontSize', 10, 'FontWeight', 'bold', 'EdgeColor', 'none', ...
               'Color', [1 1 1], 'BackgroundColor', 'none', 'FontName', 'Courier New', ...
               'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');
    
    drawnow; pause(0.01);
end

%% ═══════════════════════════════════════════════════════════════════════════
%  FINAL SIMULATION REPORT
%% ═══════════════════════════════════════════════════════════════════════════
fprintf('\n╔════════════════════════════════════════════════════╗\n');
fprintf('║   SIMULATION WITH ATTITUDE TRACKING - COMPLETE     ║\n');
fprintf('╚════════════════════════════════════════════════════╝\n\n');
fprintf('─────────────────────────────────────────────────────\n');
fprintf(' MISSION STATISTICS\n');
fprintf('─────────────────────────────────────────────────────\n');
fprintf('  Total time:                %.1f seconds\n', t);
fprintf('  Total distance traveled:   %.1f meters\n', total_distance);
fprintf('  Average speed:             %.1f m/s\n', total_distance/t);
fprintf('  Final distance to goal:    %.2f meters\n', norm(uav_goal - uav_pos));
fprintf('\n─────────────────────────────────────────────────────\n');
fprintf(' FINAL ATTITUDES\n');
fprintf('─────────────────────────────────────────────────────\n');
fprintf('  UAV Final Attitude:\n');
fprintf('    Roll:  %7.2f degrees\n', rad2deg(uav_roll));
fprintf('    Pitch: %7.2f degrees\n', rad2deg(uav_pitch));
fprintf('    Yaw:   %7.2f degrees\n', rad2deg(uav_yaw));
fprintf('\n  Intruder Final Attitudes:\n');
for k = 1:n_intruders
    fprintf('    Intruder %d: R:%6.2f° P:%6.2f° Y:%6.2f°\n', ...
            k, rad2deg(intruder_roll(k)), rad2deg(intruder_pitch(k)), rad2deg(intruder_yaw(k)));
end

%% DIRECTION CHANGE TABLE WITH ATTITUDE
fprintf('\n╔═══════════════════════════════════════════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║                           DIRECTION CHANGE EVENTS WITH ATTITUDE DATA                                          ║\n');
fprintf('╚═══════════════════════════════════════════════════════════════════════════════════════════════════════════════╝\n\n');

if dc_count > 0
    fprintf('┌──────┬────────┬───────────┬──────────────────────────────────────────────────────────────────────────────────┐\n');
    fprintf('│ Event│ Time   │  Heading  │                         UAV ATTITUDE (degrees)                                   │\n');
    fprintf('│  #   │  (s)   │ Change(°) │    Roll         Pitch          Yaw              Distance to Goal (m)            │\n');
    fprintf('├──────┼────────┼───────────┼──────────────────────────────────────────────────────────────────────────────────┤\n');
    for i = 1:dc_count
        dist_goal = sqrt((direction_change_data(i).UAV_X - uav_goal(1))^2 + ...
                         (direction_change_data(i).UAV_Y - uav_goal(2))^2 + ...
                         (direction_change_data(i).UAV_Z - uav_goal(3))^2);
        fprintf('│  %2d  │ %6.1f │  %6.1f   │  %7.2f     %7.2f      %7.2f              %6.1f                    │\n', ...
                i, direction_change_data(i).Time, direction_change_data(i).HeadingChange, ...
                direction_change_data(i).UAV_Roll, direction_change_data(i).UAV_Pitch, ...
                direction_change_data(i).UAV_Yaw, dist_goal);
    end
    fprintf('└──────┴────────┴───────────┴──────────────────────────────────────────────────────────────────────────────────┘\n');
    
    % Intruder attitudes at direction changes
    fprintf('\n┌───────────────────────────────────────────────────────────────────────────────────────────────────────────────┐\n');
    fprintf('│                           INTRUDER ATTITUDES AT DIRECTION CHANGES (degrees)                                   │\n');
    fprintf('├──────┬────────┬────────────────────────────────┬────────────────────────────────┬────────────────────────────┤\n');
    fprintf('│ Event│ Time   │      Intruder 1                │      Intruder 2                │      Intruder 3            │\n');
    fprintf('│  #   │  (s)   │   Roll  Pitch   Yaw            │   Roll  Pitch   Yaw            │   Roll  Pitch   Yaw        │\n');
    fprintf('├──────┼────────┼────────────────────────────────┼────────────────────────────────┼────────────────────────────┤\n');
    for i = 1:dc_count
        fprintf('│  %2d  │ %6.1f │ %6.2f %6.2f %6.2f       │ %6.2f %6.2f %6.2f       │ %6.2f %6.2f %6.2f   │\n', ...
                i, direction_change_data(i).Time, ...
                direction_change_data(i).Int1_Roll, direction_change_data(i).Int1_Pitch, direction_change_data(i).Int1_Yaw, ...
                direction_change_data(i).Int2_Roll, direction_change_data(i).Int2_Pitch, direction_change_data(i).Int2_Yaw, ...
                direction_change_data(i).Int3_Roll, direction_change_data(i).Int3_Pitch, direction_change_data(i).Int3_Yaw);
    end
    fprintf('└──────┴────────┴────────────────────────────────┴────────────────────────────────┴────────────────────────────┘\n');
    
    fprintf('\n┌─────────────────────────────────────────────────────────────┐\n');
    fprintf('│               Intruder 4 Attitudes (degrees)                │\n');
    fprintf('├──────┬────────┬──────────────────────────────────────────────┤\n');
    fprintf('│ Event│ Time   │   Roll      Pitch       Yaw                  │\n');
    fprintf('├──────┼────────┼──────────────────────────────────────────────┤\n');
    for i = 1:dc_count
        fprintf('│  %2d  │ %6.1f │ %7.2f   %7.2f   %7.2f              │\n', ...
                i, direction_change_data(i).Time, ...
                direction_change_data(i).Int4_Roll, direction_change_data(i).Int4_Pitch, direction_change_data(i).Int4_Yaw);
    end
    fprintf('└──────┴────────┴──────────────────────────────────────────────┘\n');
    
    % Export comprehensive data
    EventNum = (1:dc_count)';
    Time = [direction_change_data.Time]';
    HeadingChange = [direction_change_data.HeadingChange]';
    UAV_Roll = [direction_change_data.UAV_Roll]';
    UAV_Pitch = [direction_change_data.UAV_Pitch]';
    UAV_Yaw = [direction_change_data.UAV_Yaw]';
    Int1_Roll = [direction_change_data.Int1_Roll]';
    Int1_Pitch = [direction_change_data.Int1_Pitch]';
    Int1_Yaw = [direction_change_data.Int1_Yaw]';
    Int2_Roll = [direction_change_data.Int2_Roll]';
    Int2_Pitch = [direction_change_data.Int2_Pitch]';
    Int2_Yaw = [direction_change_data.Int2_Yaw]';
    Int3_Roll = [direction_change_data.Int3_Roll]';
    Int3_Pitch = [direction_change_data.Int3_Pitch]';
    Int3_Yaw = [direction_change_data.Int3_Yaw]';
    Int4_Roll = [direction_change_data.Int4_Roll]';
    Int4_Pitch = [direction_change_data.Int4_Pitch]';
    Int4_Yaw = [direction_change_data.Int4_Yaw]';
    
    attitude_table = table(EventNum, Time, HeadingChange, ...
                           UAV_Roll, UAV_Pitch, UAV_Yaw, ...
                           Int1_Roll, Int1_Pitch, Int1_Yaw, ...
                           Int2_Roll, Int2_Pitch, Int2_Yaw, ...
                           Int3_Roll, Int3_Pitch, Int3_Yaw, ...
                           Int4_Roll, Int4_Pitch, Int4_Yaw);
    writetable(attitude_table, 'UAV_Attitude_Data.csv');
    fprintf('\n✓ Attitude data exported to: UAV_Attitude_Data.csv\n');
end

%% EXPORT COMPLETE ATTITUDE HISTORY
fprintf('\n─────────────────────────────────────────────────────\n');
fprintf(' EXPORTING COMPLETE ATTITUDE HISTORY\n');
fprintf('─────────────────────────────────────────────────────\n');

% UAV complete history
uav_complete = array2table(uav_attitude_history, 'VariableNames', {'Time', 'Roll_deg', 'Pitch_deg', 'Yaw_deg'});
writetable(uav_complete, 'UAV_Complete_Attitude_History.csv');
fprintf('✓ UAV complete attitude history: UAV_Complete_Attitude_History.csv\n');

% Intruder histories
for k = 1:n_intruders
    if ~isempty(intruder_attitude_history{k})
        int_table = array2table(intruder_attitude_history{k}, ...
                               'VariableNames', {'Time', 'Roll_deg', 'Pitch_deg', 'Yaw_deg'});
        filename = sprintf('Intruder%d_Attitude_History.csv', k);
        writetable(int_table, filename);
        fprintf('✓ Intruder %d attitude history: %s\n', k, filename);
    end
end

fprintf('\n╔════════════════════════════════════════════════════╗\n');
fprintf('║  ATTITUDE DEFINITIONS:\n');
fprintf('║  • Roll:  Bank angle (rotation about X-axis)\n');
fprintf('║  • Pitch: Climb/descent angle (rotation about Y)\n');
fprintf('║  • Yaw:   Heading angle (rotation about Z-axis)\n');
fprintf('║  \n');
fprintf('║  All angles in degrees, range: -180° to +180°\n');
fprintf('╚════════════════════════════════════════════════════╝\n\n');
