function plot_simulation_results(formation_error, robot_distance, leader_trajectory, ...
    formation_mode, bezier_active, waypoints, obstacles, h_diamond, h_platoon, edges)
% PLOT_SIMULATION_RESULTS
% Generates comprehensive statistical graphs at the end of the simulation.
%
% Args:
%   formation_error: 1 x iterations - E(t) formation error over time
%   robot_distance: (N+1) x iterations - inter-robot distances + timestamps
%   leader_trajectory: 2 x iterations - leader (x,y) positions
%   formation_mode: 1 x iterations - 0=diamond, 1=platoon
%   bezier_active: 1 x iterations - 1 if in Bezier zone
%   waypoints: 2 x num_waypoints - waypoint positions
%   obstacles: num_obs x 2 - obstacle positions
%   h_diamond: N x 2 - diamond formation offsets
%   h_platoon: N x 2 - platoon formation offsets
%   edges: num_edges x 2 - edge definitions for formation

    iterations = length(formation_error);
    N_robots = size(h_diamond, 1);  % Nombre de robots (5)
    N_distances = N_robots - 1;     % Nombre de paires consécutives (4)
    num_edges = size(edges, 1);

    % Time vector (use timestamps from robot_distance)
    time_vec = robot_distance(end, :);  % Last row is timestamps

    % Steady-state analysis (ignore first 200 iterations)
    steady_start = 200;
    steady_state = formation_error(steady_start:end);

    %% ========== FIGURE 1: Formation Error ==========
    figure('Name', 'Formation Error Analysis', 'NumberTitle', 'off', ...
           'Position', [100, 100, 800, 600]);

    % Subplot 1: E(t) over time
    subplot(2, 1, 1);
    hold on;

    % Shade platoon zones
    platoon_zones = find(formation_mode == 1);
    if ~isempty(platoon_zones)
        % Find contiguous zones
        zone_starts = [platoon_zones(1)];
        zone_ends = [];
        for i = 2:length(platoon_zones)
            if platoon_zones(i) > platoon_zones(i-1) + 1
                zone_ends = [zone_ends, platoon_zones(i-1)];
                zone_starts = [zone_starts, platoon_zones(i)];
            end
        end
        zone_ends = [zone_ends, platoon_zones(end)];

        % Draw shaded regions
        y_max = max(formation_error) * 1.1;
        for z = 1:length(zone_starts)
            fill([time_vec(zone_starts(z)), time_vec(zone_ends(z)), ...
                  time_vec(zone_ends(z)), time_vec(zone_starts(z))], ...
                 [0, 0, y_max, y_max], ...
                 [1, 0.9, 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
        end
    end

    % Plot formation error
    plot(time_vec, formation_error, 'b-', 'LineWidth', 1.5);

    % Mean line
    mean_val = mean(steady_state);
    yline(mean_val, 'r--', sprintf('Mean = %.4f', mean_val), 'LineWidth', 1.5);

    xlabel('Time (s)');
    ylabel('E(t) = \Sigma(d_{actual} - d_{desired})^2');
    title('Formation Error Over Time');
    legend('Platoon Zone', 'E(t)', 'Mean (steady-state)', 'Location', 'best');
    grid on;
    hold off;

    % Subplot 2: Rolling RMSE
    subplot(2, 1, 2);
    window_size = 100;
    rmse_rolling = zeros(1, iterations - window_size + 1);
    for i = 1:(iterations - window_size + 1)
        window = formation_error(i:i+window_size-1);
        rmse_rolling(i) = sqrt(mean(window) / num_edges);
    end

    plot(time_vec(window_size:end), rmse_rolling, 'Color', [0.8, 0.4, 0], 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('RMSE (m)');
    title(sprintf('Rolling RMSE (window = %d iterations)', window_size));
    grid on;

    %% ========== FIGURE 2: Inter-Robot Distances ==========
    figure('Name', 'Inter-Robot Distances', 'NumberTitle', 'off', ...
           'Position', [150, 150, 900, 500]);

    hold on;

    % Shade platoon zones
    if ~isempty(platoon_zones)
        y_max = max(robot_distance(1:N_distances, :), [], 'all') * 1.2;
        y_min = min(robot_distance(1:N_distances, :), [], 'all') * 0.8;
        for z = 1:length(zone_starts)
            fill([time_vec(zone_starts(z)), time_vec(zone_ends(z)), ...
                  time_vec(zone_ends(z)), time_vec(zone_starts(z))], ...
                 [y_min, y_min, y_max, y_max], ...
                 [1, 0.9, 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
        end
    end

    % Colors for each distance
    colors = lines(N_distances);

    % Plot distances
    for d = 1:N_distances
        plot(time_vec, robot_distance(d, :), 'Color', colors(d,:), 'LineWidth', 1.2);
    end

    % Desired distances (diamond formation)
    desired_dists = zeros(1, N_distances);
    for d = 1:N_distances
        desired_dists(d) = norm(h_diamond(d,:) - h_diamond(d+1,:));
    end

    % Plot desired as dashed lines
    for d = 1:N_distances
        yline(desired_dists(d), '--', 'Color', colors(d,:), 'LineWidth', 1, 'Alpha', 0.6);
    end

    xlabel('Time (s)');
    ylabel('Distance (m)');
    title('Inter-Robot Distances Over Time');

    % Create legend
    legend_entries = cell(1, 2*N_distances + 1);
    legend_entries{1} = 'Platoon Zone';
    for d = 1:N_distances
        legend_entries{d+1} = sprintf('d(%d-%d)', d, d+1);
    end
    for d = 1:N_distances
        legend_entries{N_distances+d+1} = sprintf('d*(%d-%d)', d, d+1);
    end
    legend(legend_entries, 'Location', 'eastoutside');

    grid on;
    hold off;

    %% ========== FIGURE 3: Leader Trajectory ==========
    figure('Name', 'Leader Trajectory', 'NumberTitle', 'off', ...
           'Position', [200, 200, 700, 600]);

    hold on;

    % Plot trajectory with color coding for mode
    % Diamond = blue, Platoon = red, Bezier = green
    for t = 1:(iterations-1)
        if bezier_active(t)
            color = [0, 0.7, 0];  % Green for Bezier
        elseif formation_mode(t)
            color = [0.8, 0.2, 0.2];  % Red for platoon
        else
            color = [0.2, 0.2, 0.8];  % Blue for diamond
        end
        plot(leader_trajectory(1, t:t+1), leader_trajectory(2, t:t+1), ...
             'Color', color, 'LineWidth', 1.5);
    end

    % Plot waypoints
    waypoint_colors = ['k', 'b', 'r', 'g'];
    for i = 1:size(waypoints, 2)
        plot(waypoints(1, i), waypoints(2, i), 's', 'MarkerSize', 15, ...
             'LineWidth', 3, 'Color', waypoint_colors(i), 'MarkerFaceColor', 'w');
        text(waypoints(1, i) + 0.08, waypoints(2, i), sprintf('G%d', i), ...
             'FontSize', 12, 'FontWeight', 'bold');
    end

    % Plot obstacles
    if ~isempty(obstacles)
        for k = 1:size(obstacles, 1)
            plot(obstacles(k, 1), obstacles(k, 2), 'ko', 'MarkerSize', 20, ...
                 'LineWidth', 3, 'MarkerFaceColor', [0.3, 0.3, 0.3]);
        end
    end

    % Start and end markers
    plot(leader_trajectory(1, 1), leader_trajectory(2, 1), 'g^', ...
         'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'g');
    plot(leader_trajectory(1, end), leader_trajectory(2, end), 'rv', ...
         'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'r');

    xlabel('X (m)');
    ylabel('Y (m)');
    title('Leader Trajectory');
    legend('Diamond mode', 'Platoon mode', 'Bezier zone', ...
           'Waypoints', 'Obstacles', 'Start', 'End', 'Location', 'eastoutside');
    axis equal;
    xlim([-1.6, 1.6]);
    ylim([-1.0, 1.0]);
    grid on;
    hold off;

    %% ========== FIGURE 4: Summary Statistics ==========
    figure('Name', 'Summary Statistics', 'NumberTitle', 'off', ...
           'Position', [250, 250, 800, 500]);

    % Subplot 1: Bar chart of edge errors
    subplot(1, 2, 1);

    % Calculate mean error per edge (approximate from formation_error)
    edge_labels = cell(1, num_edges);
    for e = 1:num_edges
        edge_labels{e} = sprintf('%d-%d', edges(e,1), edges(e,2));
    end

    % Use formation error stats
    mean_error = mean(steady_state);
    std_error = std(steady_state);
    max_error = max(steady_state);
    min_error = min(steady_state);
    rmse = sqrt(mean_error / num_edges);

    % Bar chart of statistics
    stats_names = {'Mean E(t)', 'Std E(t)', 'Max E(t)', 'Min E(t)'};
    stats_values = [mean_error, std_error, max_error, min_error];

    bar(stats_values, 'FaceColor', [0.3, 0.5, 0.8]);
    set(gca, 'XTickLabel', stats_names);
    ylabel('Value');
    title('Formation Error Statistics (Steady-State)');
    grid on;

    % Subplot 2: Text summary
    subplot(1, 2, 2);
    axis off;

    % Calculate additional stats
    total_time = time_vec(end);
    platoon_time = sum(formation_mode) / iterations * total_time;
    bezier_time = sum(bezier_active) / iterations * total_time;

    % Create summary text
    summary_text = {
        '=== SIMULATION SUMMARY ===', ...
        '', ...
        sprintf('Total Duration: %.1f s', total_time), ...
        sprintf('Iterations: %d', iterations), ...
        '', ...
        '--- Formation Error ---', ...
        sprintf('Mean E(t): %.6f', mean_error), ...
        sprintf('Std E(t): %.6f', std_error), ...
        sprintf('RMSE: %.4f m', rmse), ...
        sprintf('Max E(t): %.6f', max_error), ...
        sprintf('Min E(t): %.6f', min_error), ...
        '', ...
        '--- Mode Statistics ---', ...
        sprintf('Diamond Time: %.1f s (%.1f%%)', total_time - platoon_time, (1 - platoon_time/total_time)*100), ...
        sprintf('Platoon Time: %.1f s (%.1f%%)', platoon_time, platoon_time/total_time*100), ...
        sprintf('Bezier Active: %.1f s (%.1f%%)', bezier_time, bezier_time/total_time*100), ...
    };

    text(0.1, 0.9, summary_text, 'FontName', 'FixedWidth', 'FontSize', 10, ...
         'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');
    title('Simulation Summary');

    fprintf('\n=== GRAPHIQUES GÉNÉRÉS ===\n');
    fprintf('Figure 1: Formation Error Analysis\n');
    fprintf('Figure 2: Inter-Robot Distances\n');
    fprintf('Figure 3: Leader Trajectory\n');
    fprintf('Figure 4: Summary Statistics\n');
end
