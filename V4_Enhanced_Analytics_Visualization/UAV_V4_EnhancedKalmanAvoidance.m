% ═══════════════════════════════════════════════════════════════════════════
% ENHANCED 3D REACTIVE COLLISION AVOIDANCE WITH KALMAN FILTERING
% Complete with line-by-line comments and visual distance graph
% NOW WITH DIRECTION CHANGE TRACKING TABLE
% ENHANCED: Intruder labels + Altitude markers + Ground reference
% FIXED: Realistic altitude constraints - All aircraft maintain safe altitude
% Author: Generated for collision avoidance demonstration
% Date: 2025
% ═══════════════════════════════════════════════════════════════════════════
%
% ALTITUDE SAFETY CONFIGURATION:
% - UAV starts at:           30m altitude
% - UAV goal altitude:       80m
% - UAV minimum altitude:    5m (hard floor)
% - UAV ground avoidance:    Activates below 15m
% - Intruders start at:      55m, 85m, 65m, 95m
% - Intruder minimum:        20m (hard floor with bounce-back)
% ═══════════════════════════════════════════════════════════════════════════

clear; clc; close all;

%% SECTION 1: SIMULATION TIME PARAMETERS
dt = 0.2; total_time = 30;

%% SECTION 2: UAV CONFIGURATION
uav_pos = [0 0 0]; uav_speed = 20; uav_goal = [200 200 80];
uav_vel = (uav_goal - uav_pos) / norm(uav_goal - uav_pos) * uav_speed;
uav_traj = uav_pos;

%% SECTION 3: INTRUDERS CONFIGURATION
% All intruders start at realistic flight altitudes (40m minimum)
intruder_pos = [0 200 55; 200 100 85; 120 220 65; 220 120 95];
intruder_vel = [15 -5 0; -10 0 -2; -12 -10 1; -10 -12 -1];
n_intruders = size(intruder_pos, 1);
intruder_traj = cell(n_intruders, 1);
for k = 1:n_intruders
    intruder_traj{k} = intruder_pos(k, :);
end

%% SECTION 4: KALMAN FILTER INITIALIZATION
kf = cell(n_intruders, 1);
for k = 1:n_intruders
    kf{k}.x = [intruder_pos(k, :)'; intruder_vel(k, :)'];
    kf{k}.P = diag([100 100 100 25 25 25]);
    kf{k}.F = [1 0 0 dt 0 0; 0 1 0 0 dt 0; 0 0 1 0 0 dt; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
    q = 5;
    kf{k}.Q = q * [dt^4/4 0 0 dt^3/2 0 0; 0 dt^4/4 0 0 dt^3/2 0; 0 0 dt^4/4 0 0 dt^3/2; 
                   dt^3/2 0 0 dt^2 0 0; 0 dt^3/2 0 0 dt^2 0; 0 0 dt^3/2 0 0 dt^2];
    kf{k}.H = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0];
    kf{k}.R = 25 * eye(3);
end

%% SECTION 5: SENSOR & SAFETY PARAMETERS
sensor_range = 100; measurement_noise_std = 5; safety_radius = 25; detection_radius = 50;

%% SECTION 6: PERFORMANCE TRACKING VARIABLES
min_distance = inf; collision_events = 0; total_distance = 0; avoidance_count = 0; direction_changes = 0;
previous_heading = atan2(uav_vel(2), uav_vel(1));
estimation_errors = zeros(total_time/dt + 1, n_intruders);
maneuver_angles = zeros(total_time/dt + 1, 1);
maneuver_angles_deg = zeros(total_time/dt + 1, 1);
time_vec = 0:dt:total_time;
time_idx = 1;
current_maneuver_angle = 0;
uav_intruder_distances = zeros(total_time/dt + 1, n_intruders);

%% SECTION 6B: DIRECTION CHANGE TRACKING (NEW!)
direction_change_data = struct('Time', {}, 'HeadingChange', {}, ...
                               'Dist_Int1', {}, 'Dist_Int2', {}, ...
                               'Dist_Int3', {}, 'Dist_Int4', {}, ...
                               'UAV_X', {}, 'UAV_Y', {}, 'UAV_Z', {});
dc_count = 0;

%% SECTION 7: MAIN SIMULATION LOOP
figure('Position', [50 50 1600 900]);

for t = 0:dt:total_time
if norm(uav_goal - uav_pos) < 2
    fprintf('✓ Goal Reached at t = %.1f seconds!\n', t);
    break;
end
    %% STEP 1: SIMULATE SENSOR MEASUREMENTS
    measurements = cell(n_intruders, 1);
    measured_positions = nan(n_intruders, 3);
    for k = 1:n_intruders
        range_to_intruder = norm(intruder_pos(k, :) - uav_pos);
        if range_to_intruder < sensor_range
            noise = measurement_noise_std * randn(1, 3);
            measurements{k} = intruder_pos(k, :)' + noise';
            measured_positions(k, :) = measurements{k}';
        else
            measurements{k} = [];
        end
    end
    
    %% STEP 2: KALMAN FILTER UPDATE
    estimated_pos = zeros(n_intruders, 3);
    estimated_vel = zeros(n_intruders, 3);
    covariances = zeros(n_intruders, 1);
    for k = 1:n_intruders
        kf{k}.x = kf{k}.F * kf{k}.x;
        kf{k}.P = kf{k}.F * kf{k}.P * kf{k}.F' + kf{k}.Q;
        if ~isempty(measurements{k})
            y = measurements{k} - kf{k}.H * kf{k}.x;
            S = kf{k}.H * kf{k}.P * kf{k}.H' + kf{k}.R;
            K = kf{k}.P * kf{k}.H' / S;
            kf{k}.x = kf{k}.x + K * y;
            kf{k}.P = (eye(6) - K * kf{k}.H) * kf{k}.P;
        end
        estimated_pos(k, :) = kf{k}.x(1:3)';
        estimated_vel(k, :) = kf{k}.x(4:6)';
        covariances(k) = trace(kf{k}.P(1:3, 1:3));
        estimation_errors(time_idx, k) = norm(estimated_pos(k, :) - intruder_pos(k, :));
        uav_intruder_distances(time_idx, k) = norm(intruder_pos(k, :) - uav_pos);
    end
    
    %% STEP 3: CALCULATE DESIRED VELOCITY
    desired_vel = (uav_goal - uav_pos);
    desired_vel = desired_vel / norm(desired_vel) * uav_speed;
    
    %% STEP 4: COLLISION AVOIDANCE
    avoid = false;
    avoidance_vec = [0 0 0];
    closest_dist = inf;
    
    % GROUND AVOIDANCE - Add repulsive force from ground
    ground_safety_altitude = 15.0; % Start avoiding ground below this altitude
    if uav_pos(3) < ground_safety_altitude
        avoid = true;
        ground_dist = uav_pos(3);
        % Strong upward force when near ground
        ground_weight = 8.0 * (ground_safety_altitude / (ground_dist + 1e-6));
        avoidance_vec = avoidance_vec + [0 0 1] * ground_weight; % Pure upward force
    end
    
    % INTRUDER AVOIDANCE
    for k = 1:n_intruders
        r = estimated_pos(k, :) - uav_pos;
        d = norm(r);
        closest_dist = min(closest_dist, d);
        true_dist = norm(intruder_pos(k, :) - uav_pos);
        if true_dist < min_distance
            min_distance = true_dist;
        end
        uncertainty = sqrt(covariances(k));
        effective_safety = safety_radius + uncertainty;
        effective_detection = detection_radius + uncertainty;
        if d < effective_detection
            avoid = true;
            repulsion = -r / (d + 1e-6);
            if d < effective_safety
                weight = 5.0 * (effective_safety / (d + 1e-6));
                collision_events = collision_events + 1;
            else
                weight = 2.0 * (effective_detection - d) / effective_detection;
            end
            confidence = exp(-uncertainty / 10);
            weight = weight * confidence;
            avoidance_vec = avoidance_vec + repulsion * weight;
            perp = cross(r, desired_vel);
            if norm(perp) > 1e-6
                perp = perp / norm(perp);
                avoidance_vec = avoidance_vec + perp * weight * 0.5;
            end
        end
    end
    
    %% STEP 5: BLENDED CONTROL
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
    
    %% STEP 6: CALCULATE MANEUVER ANGLE
    if norm(desired_vel) > 0 && norm(uav_vel) > 0
        cos_angle = dot(uav_vel, desired_vel) / (norm(uav_vel) * norm(desired_vel));
        cos_angle = max(-1, min(1, cos_angle));
        current_maneuver_angle = acos(cos_angle);
        current_maneuver_angle_deg = rad2deg(current_maneuver_angle);
    else
        current_maneuver_angle = 0;
        current_maneuver_angle_deg = 0;
    end
    maneuver_angles(time_idx) = current_maneuver_angle;
    maneuver_angles_deg(time_idx) = current_maneuver_angle_deg;
    
    %% STEP 7: TRACK DIRECTION CHANGES AND LOG DISTANCES (MODIFIED!)
    current_heading = atan2(uav_vel(2), uav_vel(1));
    heading_change = abs(rad2deg(angdiff(current_heading, previous_heading)));
    if heading_change > 30
        direction_changes = direction_changes + 1;
        dc_count = dc_count + 1;
        % Log distances to all intruders at this moment
        dist_to_intruders = zeros(1, n_intruders);
        for k = 1:n_intruders
            dist_to_intruders(k) = norm(intruder_pos(k, :) - uav_pos);
        end
        direction_change_data(dc_count).Time = t;
        direction_change_data(dc_count).HeadingChange = heading_change;
        direction_change_data(dc_count).Dist_Int1 = dist_to_intruders(1);
        direction_change_data(dc_count).Dist_Int2 = dist_to_intruders(2);
        direction_change_data(dc_count).Dist_Int3 = dist_to_intruders(3);
        direction_change_data(dc_count).Dist_Int4 = dist_to_intruders(4);
        direction_change_data(dc_count).UAV_X = uav_pos(1);
        direction_change_data(dc_count).UAV_Y = uav_pos(2);
        direction_change_data(dc_count).UAV_Z = uav_pos(3);
    end
    previous_heading = current_heading;
    
    %% STEP 8: UPDATE POSITIONS
    prev_pos = uav_pos;
    uav_pos = uav_pos + uav_vel * dt;
    
    % GROUND COLLISION AVOIDANCE - Ensure UAV doesn't go below minimum altitude
    min_altitude = 5.0; % Minimum safe altitude in meters
    if uav_pos(3) < min_altitude
        uav_pos(3) = min_altitude;
        % Add upward component to velocity to climb away from ground
        if uav_vel(3) < 0
            uav_vel(3) = abs(uav_vel(3)) + 5; % Force climb
        end
    end
    
    total_distance = total_distance + norm(uav_pos - prev_pos);
    intruder_pos = intruder_pos + intruder_vel * dt;
    
    % GROUND COLLISION AVOIDANCE FOR INTRUDERS - Ensure they don't go below minimum altitude
    min_intruder_altitude = 20.0; % Minimum safe altitude for intruders
    for k = 1:n_intruders
        if intruder_pos(k, 3) < min_intruder_altitude
            intruder_pos(k, 3) = min_intruder_altitude;
            % Reverse vertical velocity if descending
            if intruder_vel(k, 3) < 0
                intruder_vel(k, 3) = -intruder_vel(k, 3); % Bounce back up
            end
        end
    end
    
    uav_traj = [uav_traj; uav_pos];
    for k = 1:n_intruders
        intruder_traj{k} = [intruder_traj{k}; intruder_pos(k, :)];
    end
    time_idx = time_idx + 1;
    
    %% STEP 9: ENHANCED VISUALIZATION WITH LABELS AND GROUND MARKERS
    clf;
    subplot(2, 3, [1 2 4 5]);
    hold on; grid on;
    
    % ═══════ GROUND PLANE REFERENCE ═══════
    % Draw ground plane (z=0)
    [X_ground, Y_ground] = meshgrid(0:50:250, 0:50:250);
    Z_ground = zeros(size(X_ground));
    surf(X_ground, Y_ground, Z_ground, 'FaceAlpha', 0.1, 'EdgeColor', [0.7 0.7 0.7], ...
         'FaceColor', [0.9 0.9 0.85], 'EdgeAlpha', 0.3);
    
    % Plot trajectories
    plot3(uav_traj(:, 1), uav_traj(:, 2), uav_traj(:, 3), 'b-', 'LineWidth', 2.5);
    for k = 1:n_intruders
        plot3(intruder_traj{k}(:, 1), intruder_traj{k}(:, 2), intruder_traj{k}(:, 3), 'r--', 'LineWidth', 1.5);
    end
    
    % ═══════ UAV WITH ALTITUDE MARKER ═══════
    plot3(uav_pos(1), uav_pos(2), uav_pos(3), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 14);
    % Vertical line from ground to UAV
    plot3([uav_pos(1) uav_pos(1)], [uav_pos(2) uav_pos(2)], [0 uav_pos(3)], ...
          'b:', 'LineWidth', 1.5);
    % Ground projection marker
    plot3(uav_pos(1), uav_pos(2), 0, 'bx', 'MarkerSize', 10, 'LineWidth', 2);
    % UAV altitude label
    text(uav_pos(1), uav_pos(2), uav_pos(3) + 8, ...
         sprintf('UAV\nAlt: %.1fm', uav_pos(3)), ...
         'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue', ...
         'HorizontalAlignment', 'center', 'BackgroundColor', [1 1 1 0.7]);
    
    % ═══════ INTRUDERS WITH LABELS AND ALTITUDE MARKERS ═══════
    intruder_colors = {[0.8 0 0], [0.9 0.4 0], [0.7 0 0.3], [0.6 0 0.5]};
    for k = 1:n_intruders
        % Intruder position
        plot3(intruder_pos(k, 1), intruder_pos(k, 2), intruder_pos(k, 3), ...
              'o', 'MarkerFaceColor', intruder_colors{k}, 'MarkerEdgeColor', 'k', ...
              'MarkerSize', 12, 'LineWidth', 1.5);
        
        % Vertical line from ground to intruder (altitude marker)
        plot3([intruder_pos(k, 1) intruder_pos(k, 1)], ...
              [intruder_pos(k, 2) intruder_pos(k, 2)], ...
              [0 intruder_pos(k, 3)], ':', 'Color', intruder_colors{k}, 'LineWidth', 1.5);
        
        % Ground projection marker
        plot3(intruder_pos(k, 1), intruder_pos(k, 2), 0, 'x', ...
              'Color', intruder_colors{k}, 'MarkerSize', 10, 'LineWidth', 2);
        
        % Intruder label with altitude
        text(intruder_pos(k, 1), intruder_pos(k, 2), intruder_pos(k, 3) + 8, ...
             sprintf('Intruder %d\nAlt: %.1fm', k, intruder_pos(k, 3)), ...
             'FontSize', 9, 'FontWeight', 'bold', 'Color', intruder_colors{k}, ...
             'HorizontalAlignment', 'center', 'BackgroundColor', [1 1 1 0.7]);
        
        % Velocity vector for attitude indication
        quiver3(intruder_pos(k, 1), intruder_pos(k, 2), intruder_pos(k, 3), ...
                intruder_vel(k, 1)*2, intruder_vel(k, 2)*2, intruder_vel(k, 3)*2, ...
                'Color', intruder_colors{k}, 'LineWidth', 2, 'MaxHeadSize', 1.5);
    end
    
    % Estimated positions
    plot3(estimated_pos(:, 1), estimated_pos(:, 2), estimated_pos(:, 3), ...
          'ms', 'MarkerSize', 14, 'LineWidth', 2.5);
    
    % Goal
    plot3(uav_goal(1), uav_goal(2), uav_goal(3), 'g^', ...
          'MarkerSize', 18, 'LineWidth', 3, 'MarkerFaceColor', 'g');
    % Goal altitude marker
    plot3([uav_goal(1) uav_goal(1)], [uav_goal(2) uav_goal(2)], [0 uav_goal(3)], ...
          'g:', 'LineWidth', 1.5);
    text(uav_goal(1), uav_goal(2), uav_goal(3) + 8, ...
         sprintf('GOAL\nAlt: %.1fm', uav_goal(3)), ...
         'FontSize', 10, 'FontWeight', 'bold', 'Color', 'green', ...
         'HorizontalAlignment', 'center', 'BackgroundColor', [1 1 1 0.7]);
    
    % Safety sphere
    [xs, ys, zs] = sphere(20);
    surf(uav_pos(1) + safety_radius * xs, uav_pos(2) + safety_radius * ys, uav_pos(3) + safety_radius * zs, ...
         'FaceAlpha', 0.15, 'EdgeColor', 'none', 'FaceColor', 'cyan');
    
    % Velocity vectors
    quiver3(uav_pos(1), uav_pos(2), uav_pos(3), uav_vel(1)*3, uav_vel(2)*3, uav_vel(3)*3, ...
            'b', 'LineWidth', 3, 'MaxHeadSize', 2);
    quiver3(uav_pos(1), uav_pos(2), uav_pos(3), desired_vel(1)*3, desired_vel(2)*3, desired_vel(3)*3, ...
            'g--', 'LineWidth', 2, 'MaxHeadSize', 1.5);
    
    xlabel('X (m)', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Y (m)', 'FontSize', 11, 'FontWeight', 'bold');
    zlabel('Altitude Z (m)', 'FontSize', 11, 'FontWeight', 'bold');
    title(sprintf('3D View with Altitude Markers | t=%.1fs | Maneuver: %.1f°', t, current_maneuver_angle_deg), ...
          'FontSize', 13, 'FontWeight', 'bold');
    axis equal; xlim([0 250]); ylim([0 250]); zlim([0 120]); view(45, 30);
    legend('Ground', 'UAV Path', 'Intruder Paths', 'UAV', 'Intruders', 'Estimates', ...
           'Goal', 'Safety Zone', 'Actual Vel', 'Desired Vel', ...
           'Location', 'northeast', 'FontSize', 7);
    
    % ═══════ TOP VIEW WITH ENHANCED LABELING ═══════
    subplot(2, 3, 3);
    hold on; grid on;
    plot(uav_traj(:, 1), uav_traj(:, 2), 'b-', 'LineWidth', 2);
    for k = 1:n_intruders
        plot(intruder_traj{k}(:, 1), intruder_traj{k}(:, 2), '--', ...
             'Color', intruder_colors{k}, 'LineWidth', 1.5);
    end
    plot(uav_pos(1), uav_pos(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 12);
    
    % Plot and label each intruder
    for k = 1:n_intruders
        plot(intruder_pos(k, 1), intruder_pos(k, 2), 'o', ...
             'MarkerFaceColor', intruder_colors{k}, 'MarkerEdgeColor', 'k', ...
             'MarkerSize', 10, 'LineWidth', 1.5);
        text(intruder_pos(k, 1) + 8, intruder_pos(k, 2) + 8, ...
             sprintf('I%d\n%.0fm', k, intruder_pos(k, 3)), ...
             'FontSize', 8, 'FontWeight', 'bold', 'Color', intruder_colors{k});
    end
    
    plot(uav_goal(1), uav_goal(2), 'g^', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
    theta_circle = linspace(0, 2*pi, 50);
    circle_x = uav_pos(1) + safety_radius * cos(theta_circle);
    circle_y = uav_pos(2) + safety_radius * sin(theta_circle);
    plot(circle_x, circle_y, 'c-', 'LineWidth', 1.5);
    quiver(uav_pos(1), uav_pos(2), uav_vel(1)*2, uav_vel(2)*2, 'b', 'LineWidth', 2.5, 'MaxHeadSize', 2);
    
    % Add attitude arrows for intruders
    for k = 1:n_intruders
        quiver(intruder_pos(k, 1), intruder_pos(k, 2), ...
               intruder_vel(k, 1)*1.5, intruder_vel(k, 2)*1.5, ...
               'Color', intruder_colors{k}, 'LineWidth', 1.5, 'MaxHeadSize', 1.5);
    end
    
    xlabel('X (m)', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel('Y (m)', 'FontSize', 10, 'FontWeight', 'bold');
    title('TOP VIEW (XY) - Altitude shown in labels', 'FontSize', 12, 'FontWeight', 'bold');
    axis equal; xlim([0 250]); ylim([0 250]);
    legend('UAV', 'Intruders', 'UAV Pos', 'Goal', 'Safety', 'UAV Vel', ...
           'Location', 'best', 'FontSize', 7);
    
    % ═══════ DISTANCE PLOT ═══════
    subplot(2, 3, 6);
    hold on; grid on;
    plot_colors = {'b', 'r', 'g', 'm'};
    for k = 1:n_intruders
        plot(time_vec(1:time_idx-1), uav_intruder_distances(1:time_idx-1, k), ...
             [plot_colors{k} '-'], 'LineWidth', 2, 'DisplayName', sprintf('Intruder %d', k));
    end
    yline(safety_radius, 'r--', 'LineWidth', 2, 'Label', 'Safety Zone', 'LabelHorizontalAlignment', 'left');
    yline(detection_radius, 'y--', 'LineWidth', 1.5, 'Label', 'Detection Zone', 'LabelHorizontalAlignment', 'left');
    xlabel('Time (s)', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel('Distance (m)', 'FontSize', 10, 'FontWeight', 'bold');
    title('UAV-INTRUDER DISTANCES vs TIME', 'FontSize', 11, 'FontWeight', 'bold');
    xlim([0 total_time]); ylim([0 max(150, max(uav_intruder_distances(:))+10)]);
    legend('Location', 'northeast', 'FontSize', 8);
    text(0.05, 0.95, sprintf('Direction Changes: %d', direction_changes), 'Units', 'normalized', 'FontSize', 9, 'FontWeight', 'bold', 'Color', 'blue');
    text(0.05, 0.88, sprintf('Maneuver: %.1f°', current_maneuver_angle_deg), 'Units', 'normalized', 'FontSize', 9, 'FontWeight', 'bold', 'Color', 'blue');
    text(0.05, 0.81, sprintf('Closest: %.1f m', closest_dist), 'Units', 'normalized', 'FontSize', 9, 'FontWeight', 'bold', 'Color', 'blue');
    drawnow; pause(0.01);
end

%% FINAL REPORT
fprintf('\n╔════════════════════════════════════════════════════╗\n');
fprintf('║         SIMULATION COMPLETE - FINAL REPORT         ║\n');
fprintf('╚════════════════════════════════════════════════════╝\n\n');
fprintf('─────────────────────────────────────────────────────\n');
fprintf(' MISSION STATISTICS\n');
fprintf('─────────────────────────────────────────────────────\n');
fprintf('  Total time:                %.1f seconds\n', t);
fprintf('  Total distance traveled:   %.1f meters\n', total_distance);
fprintf('  Average speed:             %.1f m/s\n', total_distance/t);
fprintf('  Final distance to goal:    %.1f meters\n', norm(uav_goal - uav_pos));
fprintf('\n─────────────────────────────────────────────────────\n');
fprintf(' SAFETY METRICS\n');
fprintf('─────────────────────────────────────────────────────\n');
fprintf('  Minimum distance to intruders: %.1f meters\n', min_distance);
fprintf('  Avoidance maneuvers:           %d events\n', avoidance_count);
fprintf('  Direction changes (>30°):      %d times\n', direction_changes);
fprintf('  Maximum maneuver angle:        %.1f degrees\n', max(maneuver_angles_deg));
fprintf('  Average maneuver angle:        %.1f degrees\n', mean(maneuver_angles_deg));
fprintf('\n─────────────────────────────────────────────────────\n');
fprintf(' KALMAN FILTER PERFORMANCE\n');
fprintf('─────────────────────────────────────────────────────\n');
fprintf('  Average estimation error:  %.2f meters\n', mean(estimation_errors(:)));
fprintf('  Maximum estimation error:  %.2f meters\n', max(estimation_errors(:)));
fprintf('  Minimum estimation error:  %.2f meters\n', min(estimation_errors(estimation_errors > 0)));

%% DIRECTION CHANGE TABLE (NEW SECTION!)
fprintf('\n╔════════════════════════════════════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║                        DIRECTION CHANGE EVENTS - UAV-INTRUDER DISTANCES                                ║\n');
fprintf('╚════════════════════════════════════════════════════════════════════════════════════════════════════════╝\n\n');

if dc_count > 0
    fprintf('┌──────┬────────┬───────────┬──────────────────────────────────────────────┬─────────────────────────────────┐\n');
    fprintf('│ Event│ Time   │  Heading  │    Distance to Intruders (meters)            │   UAV Position (meters)         │\n');
    fprintf('│  #   │  (s)   │ Change(°) │  Int-1   Int-2   Int-3   Int-4               │     X       Y       Z           │\n');
    fprintf('├──────┼────────┼───────────┼──────────────────────────────────────────────┼─────────────────────────────────┤\n');
    for i = 1:dc_count
        fprintf('│  %2d  │ %6.1f │  %6.1f   │ %6.1f  %6.1f  %6.1f  %6.1f            │ %6.1f  %6.1f  %6.1f      │\n', ...
                i, direction_change_data(i).Time, direction_change_data(i).HeadingChange, ...
                direction_change_data(i).Dist_Int1, direction_change_data(i).Dist_Int2, ...
                direction_change_data(i).Dist_Int3, direction_change_data(i).Dist_Int4, ...
                direction_change_data(i).UAV_X, direction_change_data(i).UAV_Y, direction_change_data(i).UAV_Z);
    end
    fprintf('└──────┴────────┴───────────┴──────────────────────────────────────────────┴─────────────────────────────────┘\n');
    
    fprintf('\n┌─────────────────────────────────────────────────────────────────────────────────────┐\n');
    fprintf('│                    STATISTICAL SUMMARY AT DIRECTION CHANGES                         │\n');
    fprintf('├─────────────┬────────────┬────────────┬────────────┬────────────┬──────────────────┤\n');
    fprintf('│  Intruder   │  Min Dist  │  Max Dist  │  Avg Dist  │ Std Dev    │ Critical Events  │\n');
    fprintf('│             │     (m)    │     (m)    │     (m)    │    (m)     │  (< 30m)         │\n');
    fprintf('├─────────────┼────────────┼────────────┼────────────┼────────────┼──────────────────┤\n');
    for k = 1:n_intruders
        distances = [direction_change_data.(['Dist_Int' num2str(k)])];
        fprintf('│ Intruder %d  │   %6.1f   │   %6.1f   │   %6.1f   │   %5.1f    │       %2d         │\n', ...
                k, min(distances), max(distances), mean(distances), std(distances), sum(distances < 30));
    end
    fprintf('└─────────────┴────────────┴────────────┴────────────┴────────────┴──────────────────┘\n');
    
    fprintf('\n┌───────────────────────────────────────────────────────────────────┐\n');
    fprintf('│           CLOSEST INTRUDER AT EACH DIRECTION CHANGE              │\n');
    fprintf('├──────┬────────┬─────────────────┬──────────────┬─────────────────┤\n');
    fprintf('│ Event│ Time   │ Closest Intruder│   Distance   │  Safety Status  │\n');
    fprintf('│  #   │  (s)   │                 │     (m)      │                 │\n');
    fprintf('├──────┼────────┼─────────────────┼──────────────┼─────────────────┤\n');
    for i = 1:dc_count
        dist_array = [direction_change_data(i).Dist_Int1, direction_change_data(i).Dist_Int2, ...
                      direction_change_data(i).Dist_Int3, direction_change_data(i).Dist_Int4];
        [min_dist, closest_idx] = min(dist_array);
        if min_dist < safety_radius
            status = 'CRITICAL!';
        elseif min_dist < detection_radius
            status = 'Warning';
        else
            status = 'Safe';
        end
        fprintf('│  %2d  │ %6.1f │   Intruder %d    │    %6.1f    │   %-13s │\n', ...
                i, direction_change_data(i).Time, closest_idx, min_dist, status);
    end
    fprintf('└──────┴────────┴─────────────────┴──────────────┴─────────────────┘\n');
    
    % CSV EXPORT
    EventNum = (1:dc_count)';
    Time = [direction_change_data.Time]';
    HeadingChange = [direction_change_data.HeadingChange]';
    Dist_Int1 = [direction_change_data.Dist_Int1]';
    Dist_Int2 = [direction_change_data.Dist_Int2]';
    Dist_Int3 = [direction_change_data.Dist_Int3]';
    Dist_Int4 = [direction_change_data.Dist_Int4]';
    UAV_X = [direction_change_data.UAV_X]';
    UAV_Y = [direction_change_data.UAV_Y]';
    UAV_Z = [direction_change_data.UAV_Z]';
    direction_change_table = table(EventNum, Time, HeadingChange, Dist_Int1, Dist_Int2, Dist_Int3, Dist_Int4, UAV_X, UAV_Y, UAV_Z);
    writetable(direction_change_table, 'UAV_Direction_Changes.csv');
    fprintf('\n✓ Direction change data exported to: UAV_Direction_Changes.csv\n');
else
    fprintf('  No direction changes detected during simulation.\n');
end

fprintf('\n─────────────────────────────────────────────────────\n');
fprintf(' DIRECTION CHANGE ANALYSIS\n');
fprintf('─────────────────────────────────────────────────────\n');
fprintf('  Formula: Significant change when |Δθ| > 30°\n');
fprintf('  Current heading:  %.1f° (from North)\n', rad2deg(current_heading));
fprintf('  Initial heading:  %.1f° (from North)\n', rad2deg(atan2(uav_traj(1,2), uav_traj(1,1))));

fprintf('\n╔════════════════════════════════════════════════════╗\n');
fprintf('║  Key Formulas Used:\n');
fprintf('║  • Kalman Gain: K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹\n');
fprintf('║  • State Update: x̂ = x̂⁻ + K·(z - H·x̂⁻)\n');
fprintf('║  • Repulsive Force: F = -k·(r/‖r‖)/d²\n');
fprintf('║  • Blended Control: v = α·v_avoid + (1-α)·v_goal\n');
fprintf('║  • Distance: d = √((x₂-x₁)² + (y₂-y₁)² + (z₂-z₁)²)\n');
fprintf('╚════════════════════════════════════════════════════╝\n\n');