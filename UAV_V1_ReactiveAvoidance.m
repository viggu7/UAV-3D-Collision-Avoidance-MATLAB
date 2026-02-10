% Enhanced 3D Reactive Collision Avoidance Simulation
clear; clc; close all;

%---------------- Simulation Parameters ----------------%
dt = 0.2;
total_time = 30;

%---------------- UAV States ---------------------%
uav_pos = [0 0 0];
uav_speed = 20;
uav_goal = [200 200 100];
uav_vel = (uav_goal - uav_pos) / norm(uav_goal - uav_pos) * uav_speed;

% Trajectory history
uav_traj = uav_pos;

%---------------- Multiple Intruders -------------%
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

%---------------- Safety Parameters ---------------%
safety_radius = 25;
detection_radius = 50;  % Larger radius for early detection
min_distance = inf;
collision_events = 0;

%---------------- Performance Metrics ---------------%
total_distance = 0;
avoidance_count = 0;

%---------------- Simulation Loop ----------------%
figure('Position', [100 100 1200 700]);

for t = 0:dt:total_time
    %% -------- Check Goal Reached -------- %%
    if norm(uav_goal - uav_pos) < 10
        disp('Goal Reached!');
        break;
    end
    
    %% -------- Nominal motion -------- %%
    desired_vel = (uav_goal - uav_pos);
    desired_vel = desired_vel / norm(desired_vel) * uav_speed;
    
    %% -------- Advanced Reactive Safety -------- %%
    avoid = false;
    avoidance_vec = [0 0 0];
    closest_dist = inf;
    
    for k = 1:n_intruders
        r = intruder_pos(k, :) - uav_pos;
        d = norm(r);
        closest_dist = min(closest_dist, d);
        
        % Track minimum distance
        if d < min_distance
            min_distance = d;
        end
        
        % Strong repulsive force when inside detection radius
        if d < detection_radius
            avoid = true;
            
            % Repulsive vector (push away from intruder)
            repulsion = -r / (d + 1e-6);
            
            % Weight increases as intruder gets closer
            if d < safety_radius
                % Critical zone - very strong avoidance
                weight = 5.0 * (safety_radius / (d + 1e-6));
                collision_events = collision_events + 1;
            else
                % Warning zone - moderate avoidance
                weight = 2.0 * (detection_radius - d) / detection_radius;
            end
            
            avoidance_vec = avoidance_vec + repulsion * weight;
            
            % Add perpendicular component for smoother avoidance
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
        
        % Normalize avoidance vector
        avoidance_vec = avoidance_vec / (norm(avoidance_vec) + 1e-6);
        
        % Determine blend ratio based on proximity
        if closest_dist < safety_radius
            % Emergency: 95% avoidance, 5% goal
            alpha = 0.95;
        elseif closest_dist < detection_radius
            % Warning: 70% avoidance, 30% goal
            alpha = 0.70;
        else
            alpha = 0.5;
        end
        
        % Blended velocity
        uav_vel = alpha * avoidance_vec * uav_speed + (1 - alpha) * desired_vel;
        
        % Ensure speed constraint
        vel_mag = norm(uav_vel);
        if vel_mag > 0
            uav_vel = uav_vel / vel_mag * uav_speed;
        end
    else
        uav_vel = desired_vel;
        % Ensure speed constraint
        vel_mag = norm(uav_vel);
        if vel_mag > uav_speed
            uav_vel = uav_vel / vel_mag * uav_speed;
        end
    end
    
    %% -------- State Update -------- %%
    prev_pos = uav_pos;
    uav_pos = uav_pos + uav_vel * dt;
    total_distance = total_distance + norm(uav_pos - prev_pos);
    
    intruder_pos = intruder_pos + intruder_vel * dt;
    
    % Store trajectories
    uav_traj = [uav_traj; uav_pos];
    for k = 1:n_intruders
        intruder_traj{k} = [intruder_traj{k}; intruder_pos(k, :)];
    end
    
    %% -------- Visualization -------- %%
    clf;
    
    % Main 3D plot
    subplot(1, 2, 1);
    hold on; grid on;
    
    % Plot trajectories
    plot3(uav_traj(:, 1), uav_traj(:, 2), uav_traj(:, 3), ...
          'b-', 'LineWidth', 1.5);
    for k = 1:n_intruders
        plot3(intruder_traj{k}(:, 1), intruder_traj{k}(:, 2), ...
              intruder_traj{k}(:, 3), 'r--', 'LineWidth', 1);
    end
    
    % Current positions
    plot3(uav_pos(1), uav_pos(2), uav_pos(3), ...
          'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 12);
    plot3(intruder_pos(:, 1), intruder_pos(:, 2), intruder_pos(:, 3), ...
          'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
    plot3(uav_goal(1), uav_goal(2), uav_goal(3), ...
          'g^', 'MarkerSize', 15, 'LineWidth', 2, 'MarkerFaceColor', 'g');
    
    % Safety sphere
    [xs, ys, zs] = sphere(20);
    surf(uav_pos(1) + safety_radius * xs, ...
         uav_pos(2) + safety_radius * ys, ...
         uav_pos(3) + safety_radius * zs, ...
         'FaceAlpha', 0.15, 'EdgeColor', 'none', 'FaceColor', 'cyan');
    
    % Detection sphere
    surf(uav_pos(1) + detection_radius * xs, ...
         uav_pos(2) + detection_radius * ys, ...
         uav_pos(3) + detection_radius * zs, ...
         'FaceAlpha', 0.05, 'EdgeColor', 'none', 'FaceColor', 'yellow');
    
    % Velocity vector
    quiver3(uav_pos(1), uav_pos(2), uav_pos(3), ...
            uav_vel(1) * 2, uav_vel(2) * 2, uav_vel(3) * 2, ...
            'b', 'LineWidth', 2, 'MaxHeadSize', 2);
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title(sprintf('3D Reactive Collision Avoidance | t = %.1f s', t));
    axis equal;
    xlim([0 250]); ylim([0 250]); zlim([0 120]);
    view(45, 30);
    legend('UAV Traj', 'Intruder Traj', 'UAV', 'Intruders', 'Goal', ...
           'Safety', 'Detection', 'Velocity', 'Location', 'best');
    
    % Metrics panel
    subplot(1, 2, 2);
    hold on; grid on;
    
    % Distance to goal over time
    dist_to_goal = norm(uav_goal - uav_pos);
    
    % Create metrics display
    cla;
    axis off;
    text(0.1, 0.9, '\bfSimulation Metrics', 'FontSize', 14);
    text(0.1, 0.8, sprintf('Time: %.1f / %.1f s', t, total_time), 'FontSize', 11);
    text(0.1, 0.7, sprintf('Distance to Goal: %.1f m', dist_to_goal), 'FontSize', 11);
    text(0.1, 0.6, sprintf('Closest Intruder: %.1f m', closest_dist), 'FontSize', 11);
    text(0.1, 0.5, sprintf('Min Distance: %.1f m', min_distance), 'FontSize', 11);
    text(0.1, 0.4, sprintf('Total Distance: %.1f m', total_distance), 'FontSize', 11);
    text(0.1, 0.3, sprintf('Avoidance Events: %d', avoidance_count), 'FontSize', 11);
    
    % Status indicator
    if avoid
        text(0.1, 0.2, '⚠ AVOIDING', 'FontSize', 12, 'Color', 'red', 'FontWeight', 'bold');
    else
        text(0.1, 0.2, '✓ NOMINAL', 'FontSize', 12, 'Color', 'green', 'FontWeight', 'bold');
    end
    
    % Safety status
    if min_distance < safety_radius * 0.8
        safety_status = 'CRITICAL';
        safety_color = 'red';
    elseif min_distance < safety_radius
        safety_status = 'WARNING';
        safety_color = [1 0.5 0];
    else
        safety_status = 'SAFE';
        safety_color = 'green';
    end
    text(0.1, 0.1, sprintf('Safety: %s', safety_status), ...
         'FontSize', 11, 'Color', safety_color, 'FontWeight', 'bold');
    
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
fprintf('=========================================\n');