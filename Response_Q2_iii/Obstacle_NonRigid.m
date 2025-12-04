% SY28 Project - Version NON-RIGIDE pour comparaison Q2.iii
% Topologie en CHAINE (4 edges) vs Diamant rigide (7 edges)
% Objectif: Montrer l'importance de la rigidité

%% Experiment Constants
iterations = 3000;

%% Set up the Robotarium object
N = 5;  % 1 leader + 4 followers
initial_positions = generate_initial_conditions(N, 'Width', 1.5, 'Height', 1.0, 'Spacing', 0.3);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

%% Topologie NON-RIGIDE (Chaîne) - Q2.iii
% 4 edges < 2N-3 = 7 -> PAS RIGIDE
% Chaque follower regarde seulement celui devant lui

L = zeros(N, N);
L(2, 2) = 1; L(2, 1) = -1;  % F1 regarde Leader seulement
L(3, 3) = 1; L(3, 2) = -1;  % F2 regarde F1 seulement
L(4, 4) = 1; L(4, 3) = -1;  % F3 regarde F2 seulement
L(5, 5) = 1; L(5, 4) = -1;  % F4 regarde F3 seulement

dxi = zeros(2, N);
state = 1;

formation_control_gain = 4;

% Offsets Diamant (memes que version rigide pour comparaison juste)
h_diamond = [ 0,      0;       % Leader
             -0.20,  -0.20;    % F1 (gauche-haut)
              0.20,  -0.20;    % F2 (droite-haut)
              0,     -0.40;    % F3 (centre)
              0,     -0.60];   % F4 (queue)

% Offsets Platoon
h_platoon = [ 0,      0;       % Leader
              0,     -0.22;    % F1
              0,     -0.44;    % F2
              0,     -0.66;    % F3
              0,     -0.88];   % F4

current_offsets = h_diamond;

% --- OBSTACLES ---
obstacles = [-0.5, 0.0;
              0.5, 0.0];
detect_radius = 0.6;
safe_radius = 0.25;
repulsion_gain = 0.05;

%% Tools
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.8);
uni_barrier_cert = create_uni_barrier_certificate_with_boundary();

smooth_leader_controller = create_smooth_waypoint_controller(...
    'LinearVelocityGain', 0.5, ...
    'PositionError', 0.2, ...
    'LookAheadGain', 0.6, ...
    'MinimumVelocity', 0.08, ...
    'VelocityMagnitudeLimit', 0.15);

waypoints = [-0.9 0.5; 0.9 0.5; 0.9 -0.5; -0.9 -0.5]';
close_enough = 0.2;

%% Plotting Setup
CM = ['k','b','r','g','m'];
marker_size_goal = determine_marker_size(r, 0.20);
font_size = determine_font_size(r, 0.05);
line_width = 3;

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
   lf(k) = line([x(1,rows(k)), x(1,cols(k))],[x(2,rows(k)), x(2,cols(k))], 'LineWidth', line_width, 'Color', 'r');
end
leader_label = text(500, 500, 'Leader (NON-RIGIDE)', 'FontSize', font_size, 'FontWeight', 'bold', 'Color', 'r');
for j = 1:N-1
    follower_labels{j} = text(500, 500, sprintf('F%d', j), 'FontSize', font_size, 'FontWeight', 'bold');
end

%% Edges pour metrique - FORME DIAMANT (7 edges)
% On mesure les MEMES 7 edges que le rigide pour comparer la FORME
% Le controle utilise 4 edges, mais on evalue si la forme est maintenue
edges = [1,2; 1,3; 1,4; 2,3; 2,4; 3,4; 4,5];
num_edges = size(edges, 1);

%% Data Saving Setup
robot_distance = zeros(N+1, iterations);
formation_error = zeros(1, iterations);
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
        leader_label.String = 'Leader (NON-RIGIDE PLATOON)';
    else
        current_offsets = h_diamond;
        leader_label.String = 'Leader (NON-RIGIDE DIAMOND)';
    end

    %% 2. COMMANDE LEADER
    dxi(:, 1) = smooth_leader_controller(x(:, 1), waypoints, state);

    waypoint = waypoints(:, state);
    if(norm(x(1:2, 1) - waypoint) < close_enough)
        state = mod(state, size(waypoints, 2)) + 1;
    end

    %% 3. COMMANDE FOLLOWERS (topologie chaine)
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

    %% 4. EVITEMENT D'OBSTACLES
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

    %% 5. METRIQUE DE FORMATION
    E_t = 0;
    for e = 1:num_edges
        i_node = edges(e, 1);
        j_node = edges(e, 2);
        d_actual = norm(x(1:2, i_node) - x(1:2, j_node));
        d_desired = norm(current_offsets(i_node,:) - current_offsets(j_node,:));
        E_t = E_t + (d_actual - d_desired)^2;
    end
    formation_error(t) = E_t;

    % Data Saving
    for d = 1:N-1
        robot_distance(d,t) = norm(x(1:2,d) - x(1:2,d+1), 2);
    end
    robot_distance(N+1,t) = toc(start_time);
    if(norm(x(1:2, 1) - waypoint) < close_enough)
        goal_distance = [goal_distance [norm(x(1:2, 1) - waypoint);toc(start_time)]];
    end

    r.step();
end

%% Sauvegarde
save('DistanceData_NonRigid.mat', 'robot_distance');
save('GoalData_NonRigid.mat', 'goal_distance');
save('FormationError_NonRigid.mat', 'formation_error', 'edges');

%% Affichage des statistiques
fprintf('\n=== METRIQUE DE FORMATION - NON-RIGIDE (Q2.iii) ===\n');
fprintf('Topologie: CHAINE (controle 4 edges, mesure 7 edges)\n');
fprintf('E(t) = Σ(d_actual - d_desired)² sur 7 edges de la forme\n\n');

steady_state = formation_error(200:end);

fprintf('Statistiques sur 7 edges [1-2, 1-3, 1-4, 2-3, 2-4, 3-4, 4-5]:\n');
fprintf('  Mean E(t):     %.6f\n', mean(steady_state));
fprintf('  Std E(t):      %.6f\n', std(steady_state));
fprintf('  Max E(t):      %.6f\n', max(steady_state));
fprintf('  Min E(t):      %.6f\n', min(steady_state));
fprintf('  RMSE:          %.4f m\n', sqrt(mean(steady_state)/num_edges));
fprintf('  Erreur finale: %.6f\n', formation_error(end));

%% Verification de FORME - edges NON controles
% Ces distances DEVRAIENT etre fixes si la forme diamant est maintenue
% Mais le non-rigide ne les controle pas !
fprintf('\n=== VERIFICATION DE FORME (edges NON controles) ===\n');
x_final = r.get_poses();

% Distances desirees dans le diamant
d_13_desired = norm(h_diamond(1,:) - h_diamond(3,:));  % Leader-F2
d_14_desired = norm(h_diamond(1,:) - h_diamond(4,:));  % Leader-F3
d_24_desired = norm(h_diamond(2,:) - h_diamond(4,:));  % F1-F3

% Distances actuelles
d_13_actual = norm(x_final(1:2,1) - x_final(1:2,3));
d_14_actual = norm(x_final(1:2,1) - x_final(1:2,4));
d_24_actual = norm(x_final(1:2,2) - x_final(1:2,4));

fprintf('Distance Leader-F2 (1-3): desire=%.3f, actuel=%.3f, erreur=%.3f m\n', d_13_desired, d_13_actual, abs(d_13_actual-d_13_desired));
fprintf('Distance Leader-F3 (1-4): desire=%.3f, actuel=%.3f, erreur=%.3f m\n', d_14_desired, d_14_actual, abs(d_14_actual-d_14_desired));
fprintf('Distance F1-F3 (2-4):     desire=%.3f, actuel=%.3f, erreur=%.3f m\n', d_24_desired, d_24_actual, abs(d_24_actual-d_24_desired));

total_shape_error = (d_13_actual-d_13_desired)^2 + (d_14_actual-d_14_desired)^2 + (d_24_actual-d_24_desired)^2;
fprintf('\nErreur de FORME (edges non controles): %.6f\n', total_shape_error);

r.debug();

%% Helper Functions
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
