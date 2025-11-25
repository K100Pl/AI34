% SY28 Project - Version complète avec Obstacles et Reconfiguration
% Correction: Position des obstacles ajustée pour éviter les waypoints.

%% Experiment Constants
iterations = 3000;

%% Set up the Robotarium object
N = 4;
initial_positions = generate_initial_conditions(N, 'Width', 1.2, 'Height', 0.8, 'Spacing', 0.4);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

%% Topologie Rigide (Diamant) - Q2
L = zeros(N, N);
L(2, 2) = 1; L(2, 1) = -1;
L(3, 3) = 1; L(3, 1) = -1;
L(4, 4) = 2; L(4, 2) = -1; L(4, 3) = -1; 

dxi = zeros(2, N);
state = 1;

formation_control_gain = 3; 

% Offsets Diamant
h_diamond = [ 0,    0;      
             -0.3, -0.3;    
              0.3, -0.3;    
              0,   -0.6];   

% Offsets Platoon (Q4)
h_platoon = [ 0,    0;
              0,   -0.35;
              0,   -0.7;
              0,   -1.05];
              
current_offsets = h_diamond; 

% --- PARAMÈTRES D'OBSTACLES AJUSTÉS ---
% WP1: (-0.9, 0.5), WP2: (0.9, 0.5), WP3: (0.9, -0.5), WP4: (-0.9, -0.5)
% O1: Central (0, 0), O2: Déplacé à (-0.3, 0.0) pour éviter les coins
obstacles = [0.0, 0.0; -0.3, 0.5]; 
detect_radius = 0.6;    
safe_radius = 0.25;     
repulsion_gain = 0.05;  

%% Tools
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.8);
uni_barrier_cert = create_uni_barrier_certificate_with_boundary();
leader_controller = create_si_position_controller('XVelocityGain', 0.8, 'YVelocityGain', 0.8, 'VelocityMagnitudeLimit', 0.15);

waypoints = [-0.9 0.5; 0.9 0.5; 0.9 -0.5; -0.9 -0.5]';
close_enough = 0.2; 

%% Plotting Setup
CM = ['k','b','r','g'];
marker_size_goal = determine_marker_size(r, 0.20);
font_size = determine_font_size(r, 0.05);
line_width = 3;

% Waypoints & Obstacles 
for i = 1:length(waypoints)
    goal_caption = sprintf('G%d', i);
    plot(waypoints(1,i), waypoints(2,i),'s','MarkerSize',marker_size_goal,'LineWidth',line_width,'Color',CM(i));
    text(waypoints(1,i)-0.05, waypoints(2,i), goal_caption, 'FontSize', font_size, 'FontWeight', 'bold');
end
for k=1:size(obstacles,1)
    plot(obstacles(k,1), obstacles(k,2), 'ko', 'MarkerSize', marker_size_goal, 'LineWidth', 3, 'MarkerFaceColor','k');
end

x=r.get_poses();
[rows, cols] = find(L == -1); 
for k = 1:length(rows)
   lf(k) = line([x(1,rows(k)), x(1,cols(k))],[x(2,rows(k)), x(2,cols(k))], 'LineWidth', line_width, 'Color', 'b'); 
end
leader_label = text(500, 500, 'Leader', 'FontSize', font_size, 'FontWeight', 'bold', 'Color', 'r');
for j = 1:N-1    
    follower_labels{j} = text(500, 500, sprintf('F%d', j), 'FontSize', font_size, 'FontWeight', 'bold');
end

%% Data Saving Setup (Identique)
robot_distance = zeros(5,iterations); 
goal_distance = []; 
start_time = tic;

dxi(:, 1) = [0.01; 0.01];
r.step();

%% BOUCLE PRINCIPALE
for t = 1:iterations
    
    x = r.get_poses();
    
    %% 1. RECONFIGURATION (Q4)
    dist_to_obs = min(sqrt(sum((x(1:2,1) - obstacles').^2)));
    
    if dist_to_obs < detect_radius
        current_offsets = h_platoon; 
        leader_label.String = 'Leader (PLATOON)';
    else
        current_offsets = h_diamond; 
        leader_label.String = 'Leader (DIAMOND)';
    end

    %% 2. COMMANDE LEADER (Q1)
    waypoint = waypoints(:, state);
    dxi(:, 1) = leader_controller(x(1:2, 1), waypoint);
    
    if(norm(x(1:2, 1) - waypoint) < close_enough)
        state = mod(state, 4) + 1; 
    end
    
    %% 3. COMMANDE FOLLOWERS (Q2)
    for i = 2:N
        dxi(:, i) = [0 ; 0];
        neighbors = find(L(i,:) == -1);
        
        for j = neighbors
            pos_error = x(1:2, j) - x(1:2, i);
            offset_error = current_offsets(j,:)' - current_offsets(i,:)';
            dxi(:, i) = dxi(:, i) + formation_control_gain * (pos_error - offset_error);
        end
        dxi(:, i) = dxi(:, i) + dxi(:, 1); 
    end
    
    %% 4. ÉVITEMENT D'OBSTACLES (Q3)
    for i = 1:N
        for k = 1:size(obstacles, 1)
            vect = x(1:2, i) - obstacles(k,:)';
            dist = norm(vect);
            if dist < safe_radius
                 dxi(:, i) = dxi(:, i) + repulsion_gain * (vect / dist^3);
            end
        end
    end

    %% Limits & Dynamics
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
    
    dxu = si_to_uni_dyn(dxi, x);
    dxu = uni_barrier_cert(dxu, x);
    
    r.set_velocities(1:N, dxu);
    
    %% Update Plots
    for q = 1:N-1
        follower_labels{q}.Position = x(1:2, q+1) + [-0.1;0.1];     
    end
    
    idx_line = 1;
    for r_idx = 2:N 
         neighbors = find(L(r_idx,:) == -1);
         for c_idx = neighbors
             if idx_line <= length(lf)
                 lf(idx_line).XData = [x(1,r_idx), x(1,c_idx)];
                 lf(idx_line).YData = [x(2,r_idx), x(2,c_idx)];
                 idx_line = idx_line + 1;
             end
         end
    end
    leader_label.Position = x(1:2, 1) + [-0.1;0.1];
    
    % Data Saving
    robot_distance(1,t) = norm([x(1:2,1) - x(1:2,2)],2);
    robot_distance(5,t) = toc(start_time);
    if(norm(x(1:2, 1) - waypoint) < close_enough)
        goal_distance = [goal_distance [norm(x(1:2, 1) - waypoint);toc(start_time)]];
    end
    
    r.step();
end

save('DistanceData.mat', 'robot_distance');
save('GoalData.mat', 'goal_distance');
r.debug();

%% Helper Functions (Identique)
function marker_size = determine_marker_size(robotarium_instance, marker_size_meters)
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Points');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);
marker_ratio = (marker_size_meters)/(robotarium_instance.boundaries(2) -...
    robotarium_instance.boundaries(1));
marker_size = cursize(3) * marker_ratio;
end

function font_size = determine_font_size(robotarium_instance, font_height_meters)
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Points');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);
font_ratio = (font_height_meters)/(robotarium_instance.boundaries(4) -...
    robotarium_instance.boundaries(3));
font_size = cursize(4) * font_ratio;
end