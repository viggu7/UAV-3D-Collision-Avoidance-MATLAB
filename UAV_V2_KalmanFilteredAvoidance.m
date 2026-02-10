% 3D Reactive Collision Avoidance with Kalman Filtering
clear; clc; close all;

%---------------- Simulation Parameters ----------------%
dt = 0.2;
total_time = 30;

%---------------- UAV States ---------------------%
uav_pos = [0 0 0];
uav_speed = 20;
uav_goal = [200 200 100];
uav_vel = (uav_goal - uav_pos) / norm(uav_goal - uav_pos) * uav_speed;
uav_traj = uav_pos;

%---------------- Multiple Intruders (True States) -------------%
intruder_pos = [
     0 200 50;
   200 100 80;
   120 220 60;
   220 120 90
];
intruder_vel = [
    15  -5   0;
   -10   0  -2;
   -12 -10   1;
   -10 -12  -1
];
n_intruders = size(intruder_pos, 1);
intruder_traj = cell(n_intruders, 1);
for k = 1:n_intruders
    intruder_traj{k} = intruder_pos(k, :);
end

%---------------- Kalman Filter Initialization -------------%
% State vector: [x, y, z, vx, vy, vz]' for each intruder
kf = cell(n_intruders, 1);

for k = 1:n_intruders
    kf{k}.x = [intruder_pos(k, :)'; intruder_vel(k, :)'];  % Initial state (6x1)
    kf{k}.P = diag([100 100 100 25 25 25]);  % Initial covariance
    
    % State transition matrix (constant velocity model)
    kf{k}.F = [1 0 0 dt 0  0;
               0 1 0 0  dt 0;
               0 0 1 0  0  dt;
               0 0 0 1  0  0;
               0 0 0 0  1  0;
               0 0 0 0  0  1];
    
    % Process noise covariance
    q = 5;  % Process noise intensity
    kf{k}.Q = q * [dt^4/4  0      0      dt^3/2  0       0;
                   0       dt^4/4 0      0       dt^3/2  0;
                   0       0      dt^4/4 0       0       dt^3/2;
                   dt^3/2  0      0      dt^2    0       0;
                   0       dt^3/2 0      0       dt^2    0;
                   0       0      dt^3/2 0       0       dt^2];
    
    % Measurement matrix (we measure position only)
    kf{k}.H = [1 0 0 0 0 0;
               0 1 0 0 0 0;
               0 0 1 0 0 0];
    
    % Measurement noise covariance
    measurement_noise = 5;  % meters
    kf{k}.R = measurement_noise^2 * eye(3);
end

%---------------- Sensor Parameters ---------------%
sensor_range = 100;  % Maximum detection range
measurement_noise_std = 5;  % Standard deviation of position measurements

%---------------- Safety Parameters ---------------%
safety_radius = 25;
detection_radius = 50;
min_distance = inf;
collision_events = 0;
total_distance = 0;
avoidance_count = 0;

% Storage for estimation error and maneuver angles
estimation_errors = zeros(total_time/dt + 1, n_intruders);
maneuver_angles = zeros(total_time/dt + 1, 1);
maneuver_angles_deg = zeros(total_time/dt + 1, 1);
time_vec = 0:dt:total_time;
time_idx = 1;
current_maneuver_angle = 0;

%---------------- Simulation Loop ----------------%
figure('Position', [100 100 1400 700]);

for t = 0:dt:total_time
    %% -------- Check Goal Reached -------- %%
    if norm(uav_goal - uav_pos) < 10
        disp('Goal Reached!');
        break;
    end
    
    %% -------- Sensor Measurements with Noise -------- %%
    measurements = cell(n_intruders, 1);
    measured_positions = nan(n_intruders, 3);
    
    for k = 1:n_intruders
        range_to_intruder = norm(intruder_pos(k, :) - uav_pos);
        
        if range_to_intruder < sensor_range
            % Add measurement noise
            noise = measurement_noise_std * randn(1, 3);
            measurements{k} = intruder_pos(k, :)' + noise';
            measured_positions(k, :) = measurements{k}';
        else
            measurements{k} = [];  % Out of sensor range
        end
    end
    
    %% -------- Kalman Filter Update -------- %%
    estimated_pos = zeros(n_intruders, 3);
    estimated_vel = zeros(n_intruders, 3);
    covariances = zeros(n_intruders, 1);
    
    for k = 1:n_intruders
        % Prediction step
        kf{k}.x = kf{k}.F * kf{k}.x;
        kf{k}.P = kf{k}.F * kf{k}.P * kf{k}.F' + kf{k}.Q;
        
        % Update step (if measurement available)
        if ~isempty(measurements{k})
            y = measurements{k} - kf{k}.H * kf{k}.x;  % Innovation
            S = kf{k}.H * kf{k}.P * kf{k}.H' + kf{k}.R;  % Innovation covariance
            K = kf{k}.P * kf{k}.H' / S;  % Kalman gain
            
            kf{k}.x = kf{k}.x + K * y;
            kf{k}.P = (eye(6) - K * kf{k}.H) * kf{k}.P;
        end
        
        % Extract estimates
        estimated_pos(k, :) = kf{k}.x(1:3)';
        estimated_vel(k, :) = kf{k}.x(4:6)';
        covariances(k) = trace(kf{k}.P(1:3, 1:3));
        
        % Calculate estimation error
        estimation_errors(time_idx, k) = norm(estimated_pos(k, :) - intruder_pos(k, :));
    end
    
    %% -------- Nominal motion -------- %%
    desired_vel = (uav_goal - uav_pos);
    desired_vel = desired_vel / norm(desired_vel) * uav_speed;
    
    %% -------- Collision Avoidance using Kalman Estimates -------- %%
    avoid = false;
    avoidance_vec = [0 0 0];
    closest_dist = inf;
    
    for k = 1:n_intruders
        % Use estimated position for avoidance
        r = estimated_pos(k, :) - uav_pos;
        d = norm(r);
        closest_dist = min(closest_dist, d);
        
        % Track minimum distance (using true position)
        true_dist = norm(intruder_pos(k, :) - uav_pos);
        if true_dist < min_distance
            min_distance = true_dist;
        end
        
        % Account for uncertainty in position estimate
        uncertainty = sqrt(covariances(k));
        effective_safety = safety_radius + uncertainty;
        effective_detection = detection_radius + uncertainty;
        
        if d < effective_detection
            avoid = true;
            
            % Repulsive vector
            repulsion = -r / (d + 1e-6);
            
            % Weight increases as intruder gets closer
            if d < effective_safety
                weight = 5.0 * (effective_safety / (d + 1e-6));
                collision_events = collision_events + 1;
            else
                weight = 2.0 * (effective_detection - d) / effective_detection;
            end
            
            % Reduce weight if uncertainty is high
            confidence = exp(-uncertainty / 10);
            weight = weight * confidence;
            
            avoidance_vec = avoidance_vec + repulsion * weight;
            
            % Perpendicular component
            perp = cross(r, desired_vel);
            if norm(perp) > 1e-6
                perp = perp / norm(perp);
                avoidance_vec = avoidance_vec + perp * weight * 0.5;
            end
        end
    end
    
    %% -------- Blended Control Law -------- %%
    if avoid
        avoidance_count = avoidance_count + 1;
        avoidance_vec = avoidance_vec / (norm(avoidance_vec) + 1e-6);
        
        if closest_dist < safety_radius
            alpha = 0.95;
        elseif closest_dist < detection_radius
            alpha = 0.70;
        else
            alpha = 0.5;
        end
        
        uav_vel = alpha * avoidance_vec * uav_speed + (1 - alpha) * desired_vel;
        vel_mag = norm(uav_vel);
        if vel_mag > 0
            uav_vel = uav_vel / vel_mag * uav_speed;
        end
    else
        uav_vel = desired_vel;
        vel_mag = norm(uav_vel);
        if vel_mag > uav_speed
            uav_vel = uav_vel / vel_mag * uav_speed;
        end
    end
    
    %% -------- Calculate Maneuver Angle -------- %%
    % Angle between actual velocity and direct path to goal
    if norm(desired_vel) > 0 && norm(uav_vel) > 0
        cos_angle = dot(uav_vel, desired_vel) / (norm(uav_vel) * norm(desired_vel));
        cos_angle = max(-1, min(1, cos_angle));  % Clamp to [-1, 1]
        current_maneuver_angle = acos(cos_angle);  % Radians
        current_maneuver_angle_deg = rad2deg(current_maneuver_angle);
    else
        current_maneuver_angle = 0;
        current_maneuver_angle_deg = 0;
    end
    
    maneuver_angles(time_idx) = current_maneuver_angle;
    maneuver_angles_deg(time_idx) = current_maneuver_angle_deg;
    
    %% -------- State Update -------- %%
    prev_pos = uav_pos;
    uav_pos = uav_pos + uav_vel * dt;
    total_distance = total_distance + norm(uav_pos - prev_pos);
    
    % Update true intruder positions
    intruder_pos = intruder_pos + intruder_vel * dt;
    
    % Store trajectories
    uav_traj = [uav_traj; uav_pos];
    for k = 1:n_intruders
        intruder_traj{k} = [intruder_traj{k}; intruder_pos(k, :)];
    end
    
    time_idx = time_idx + 1;
    
    %% -------- Visualization -------- %%
    clf;
    
    % Main 3D plot
    subplot(2, 2, [1 3]);
    hold on; grid on;
    
    % Plot trajectories
    plot3(uav_traj(:, 1), uav_traj(:, 2), uav_traj(:, 3), ...
          'b-', 'LineWidth', 2);
    for k = 1:n_intruders
        plot3(intruder_traj{k}(:, 1), intruder_traj{k}(:, 2), ...
              intruder_traj{k}(:, 3), 'r--', 'LineWidth', 1);
    end
    
    % Current positions
    plot3(uav_pos(1), uav_pos(2), uav_pos(3), ...
          'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 12);
    
    % True intruder positions
    plot3(intruder_pos(:, 1), intruder_pos(:, 2), intruder_pos(:, 3), ...
          'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
    
    % Estimated intruder positions
    plot3(estimated_pos(:, 1), estimated_pos(:, 2), estimated_pos(:, 3), ...
          'ms', 'MarkerSize', 12, 'LineWidth', 2);
    
    % Measured positions (if available)
    valid_measurements = ~isnan(measured_positions(:, 1));
    if any(valid_measurements)
        plot3(measured_positions(valid_measurements, 1), ...
              measured_positions(valid_measurements, 2), ...
              measured_positions(valid_measurements, 3), ...
              'cx', 'MarkerSize', 10, 'LineWidth', 2);
    end
    
    plot3(uav_goal(1), uav_goal(2), uav_goal(3), ...
          'g^', 'MarkerSize', 15, 'LineWidth', 2, 'MarkerFaceColor', 'g');
    
    % Safety sphere
    [xs, ys, zs] = sphere(20);
    surf(uav_pos(1) + safety_radius * xs, ...
         uav_pos(2) + safety_radius * ys, ...
         uav_pos(3) + safety_radius * zs, ...
         'FaceAlpha', 0.15, 'EdgeColor', 'none', 'FaceColor', 'cyan');
    
    % Velocity vector
    quiver3(uav_pos(1), uav_pos(2), uav_pos(3), ...
            uav_vel(1) * 2, uav_vel(2) * 2, uav_vel(3) * 2, ...
            'b', 'LineWidth', 2, 'MaxHeadSize', 2);
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title(sprintf('Kalman-Filtered Collision Avoidance | t = %.1f s', t));
    axis equal;
    xlim([0 250]); ylim([0 250]); zlim([0 120]);
    view(45, 30);
    legend('UAV', 'Intruder True', 'UAV Pos', 'True Pos', 'Est. Pos', ...
           'Measurements', 'Goal', 'Safety', 'Velocity', 'Location', 'best');
    
    % Metrics panel
    subplot(2, 2, 2);
    hold on; grid on;
    dist_to_goal = norm(uav_goal - uav_pos);
    
    cla;
    axis off;
    text(0.1, 0.95, '\bfSimulation Metrics', 'FontSize', 12);
    text(0.1, 0.85, sprintf('Time: %.1f / %.1f s', t, total_time), 'FontSize', 10);
    text(0.1, 0.75, sprintf('Distance to Goal: %.1f m', dist_to_goal), 'FontSize', 10);
    text(0.1, 0.65, sprintf('Closest Intruder: %.1f m', closest_dist), 'FontSize', 10);
    text(0.1, 0.55, sprintf('Min Distance: %.1f m', min_distance), 'FontSize', 10);
    text(0.1, 0.45, sprintf('Avoidance Events: %d', avoidance_count), 'FontSize', 10);
    
    text(0.1, 0.3, '\bfKalman Filter Stats', 'FontSize', 12);
    avg_error = mean(estimation_errors(time_idx-1, :));
    text(0.1, 0.2, sprintf('Avg Est. Error: %.2f m', avg_error), 'FontSize', 10);
    text(0.1, 0.1, sprintf('Max Uncertainty: %.2f m', sqrt(max(covariances))), 'FontSize', 10);
    
    if avoid
        text(0.1, 0.02, '⚠ AVOIDING', 'FontSize', 11, 'Color', 'red', 'FontWeight', 'bold');
    else
        text(0.1, 0.02, '✓ NOMINAL', 'FontSize', 11, 'Color', 'green', 'FontWeight', 'bold');
    end
    
    % Estimation error plot
    subplot(2, 2, 4);
    hold on; grid on;
    for k = 1:n_intruders
        plot(time_vec(1:time_idx-1), estimation_errors(1:time_idx-1, k), ...
             'LineWidth', 1.5, 'DisplayName', sprintf('Intruder %d', k));
    end
    xlabel('Time (s)');
    ylabel('Position Error (m)');
    title('Kalman Filter Estimation Error');
    legend('Location', 'best');
    xlim([0 total_time]);
    
    drawnow;
    pause(0.01);
end

%% -------- Final Report -------- %%
fprintf('\n========== Simulation Complete ==========\n');
fprintf('Total time: %.1f s\n', t);
fprintf('Total distance traveled: %.1f m\n', total_distance);
fprintf('Minimum distance to intruders: %.1f m\n', min_distance);
fprintf('Avoidance maneuvers: %d\n', avoidance_count);
fprintf('Final distance to goal: %.1f m\n', norm(uav_goal - uav_pos));
fprintf('\n--- Kalman Filter Performance ---\n');
fprintf('Average estimation error: %.2f m\n', mean(estimation_errors(:)));
fprintf('Max estimation error: %.2f m\n', max(estimation_errors(:)));
fprintf('=========================================\n');
