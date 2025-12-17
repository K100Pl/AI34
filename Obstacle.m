% SY28 Project - Version complète avec Smooth Navigation, Obstacles et Reconfiguration
% Q1: Smooth navigation (Vilca 2015)
% Q2: Formation diamant rigide avec 5 robots (1 leader + 4 followers)
% Q3: Évitement d'obstacles (champ potentiel)
% Q4: Reconfiguration diamant <-> platoon

%% Experiment Constants
iterations = 15000;  % Plus long pour bien observer

%% Set up the Robotarium object
N = 5;  % 1 leader + 4 followers (Q2 requirement)

% Position initiale: entre G4(-1.2,-0.7) et G1(-1.2,0.7), orienté vers G1
% Leader au centre, followers en formation diamant derrière
initial_positions = [-1.2, -1.0, -1.0, -1.2, -1.4;   % x: tous à gauche
                      0.0, -0.15,  0.15, -0.30, -0.45; % y: entre G4 et G1
                      pi/2, pi/2, pi/2, pi/2, pi/2];   % theta: orientés vers le haut (vers G1)

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

%% Topologie Rigide (Diamant) - Q2
% 7 edges = 2N-3 pour N=5 -> Minimalement RIGIDE

L = zeros(N, N);
% F1(2) regarde Leader(1) et F2(3)
L(2, 2) = 2; L(2, 1) = -1; L(2, 3) = -1;
% F2(3) regarde Leader(1) et F1(2)
L(3, 3) = 2; L(3, 1) = -1; L(3, 2) = -1;
% F3(4) regarde Leader(1), F1(2), F2(3)
L(4, 4) = 3; L(4, 1) = -1; L(4, 2) = -1; L(4, 3) = -1;
% F4(5) regarde F3(4)
L(5, 5) = 1; L(5, 4) = -1;

dxi = zeros(2, N);
state = 1;

formation_control_gain = 4;  % Augmenté pour 5 robots

% Offsets Diamant (5 robots) - EN COORDONNÉES LOCALES (relatives au leader)
% X positif = DEVANT le leader, X négatif = DERRIÈRE
% Y positif = GAUCHE, Y négatif = DROITE
h_diamond_local = [ 0,      0;       % Leader (origine)
                   -0.20,   0.20;    % F1 (derrière-gauche)
                   -0.20,  -0.20;    % F2 (derrière-droite)
                   -0.40,   0;       % F3 (directement derrière)
                   -0.60,   0];      % F4 (queue)

% Offsets Platoon (5 robots) - Q4 - EN COORDONNÉES LOCALES
h_platoon_local = [ 0,      0;       % Leader
                   -0.22,   0;       % F1 (derrière)
                   -0.44,   0;       % F2 (derrière)
                   -0.66,   0;       % F3 (derrière)
                   -0.88,   0];      % F4 (derrière)

current_offsets_local = h_diamond_local; 

% --- OBSTACLES ---
% Placés PILE sur les trajectoires entre waypoints
%   WP1(-1.2,0.7) → WP2(1.2,0.7)  : trajectoire horizontale haut, y=0.7
%   WP2(1.2,0.7)  → WP3(1.2,-0.7) : trajectoire verticale droite, x=1.2
%   WP3(1.2,-0.7) → WP4(-1.2,-0.7): trajectoire horizontale bas, y=-0.7
%   WP4(-1.2,-0.7)→ WP1(-1.2,0.7) : trajectoire verticale gauche, x=-1.2
obstacles = [ 0.0,  0.7;   % Sur trajectoire WP1→WP2 (haut)
              0.0, -0.7];  % Sur trajectoire WP3→WP4 (bas)
detect_radius = 0.6;
safe_radius = 0.25;
repulsion_gain = 0.05;  

%% Tools
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.8);
uni_barrier_cert = create_uni_barrier_certificate_with_boundary();

% Smooth waypoint controller with Bezier cornering
smooth_leader_controller = create_smooth_waypoint_controller(...
    'LinearVelocityGain', 0.20, ...
    'PositionError', 0.2, ...
    'LookAheadGain', 0.6, ...
    'MinimumVelocity', 0.02, ...
    'VelocityMagnitudeLimit', 0.06, ...
    'BezierZoneRadius', 0.5, ...
    'BezierExitLength', 0.35, ...
    'BezierVelocity', 0.06);

% Waypoints agrandis (plus proches des bords du Robotarium)
waypoints = [-1.2 0.7; 1.2 0.7; 1.2 -0.7; -1.2 -0.7]';
close_enough = 0.2; 

%% Plotting Setup
CM = ['k','b','r','g','m'];  % 5 couleurs pour 5 robots
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

%% DEBUG BEZIER - Marqueurs visuels (initialisés hors écran)
bezier_markers = struct();
bezier_markers.zone_marker = plot(10, 10, 'ro', 'MarkerSize', 15, 'LineWidth', 3, 'MarkerFaceColor', 'r');
bezier_markers.P0_marker = plot(10, 10, 'gs', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'g');
bezier_markers.P1_marker = plot(10, 10, 'rs', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'r');
bezier_markers.P2_marker = plot(10, 10, 'bs', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'b');
bezier_markers.bezier_curve = plot(10, 10, 'm-', 'LineWidth', 2);
bezier_markers.target_marker = plot(10, 10, 'c*', 'MarkerSize', 15, 'LineWidth', 2);

%% Définition des edges pour la métrique de formation (Q2.ii & Q2.iii)
% 7 edges définissant la forme DIAMANT (2N-3 pour N=5)
% On mesure la FORME, pas seulement les edges de contrôle
edges = [1,2; 1,3; 1,4; 2,3; 2,4; 3,4; 4,5];
num_edges = size(edges, 1);

%% Data Saving Setup
robot_distance = zeros(N+1, iterations);  % N distances + timestamp
formation_error = zeros(1, iterations);   % E(t) = Σ(d - d*)² (Q2.ii)
goal_distance = [];
start_time = tic;

dxi(:, 1) = [0.01; 0.01];
r.step();

%% BOUCLE PRINCIPALE
for t = 1:iterations
    
    x = r.get_poses();
    
    %% 1. RECONFIGURATION (Q4)
    if ~isempty(obstacles)
        dist_to_obs = min(sqrt(sum((x(1:2,1) - obstacles').^2)));
    else
        dist_to_obs = inf;  % Pas d'obstacles = distance infinie
    end

    if dist_to_obs < detect_radius
        current_offsets_local = h_platoon_local;
        leader_label.String = 'Leader (PLATOON)';
    else
        current_offsets_local = h_diamond_local;
        leader_label.String = 'Leader (DIAMOND)';
    end

    %% 1b. ROTATION DES OFFSETS selon orientation du leader
    % Transforme les offsets locaux (relatifs au leader) en coordonnées globales
    theta_leader = x(3, 1);
    R = [cos(theta_leader), -sin(theta_leader);
         sin(theta_leader),  cos(theta_leader)];

    current_offsets = zeros(N, 2);
    for i = 1:N
        current_offsets(i,:) = (R * current_offsets_local(i,:)')';
    end

    %% 2. COMMANDE LEADER - Smooth Navigation avec Bézier (Q1)
    leader_pos = x(1:2, 1);

    % Appel au contrôleur (toute la logique Bézier est encapsulée)
    [dxi(:, 1), bezier_info] = smooth_leader_controller(x(:, 1), waypoints, state);

    % Mise à jour des marqueurs debug
    update_bezier_debug_markers(bezier_markers, bezier_info, leader_pos);

    % Switch waypoint si courbe terminée
    if bezier_info.bezier_completed
        state = mod(state, size(waypoints, 2)) + 1;
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
    
    %% 5. MÉTRIQUE DE FORMATION (Q2.ii)
    % E(t) = Σ(d_actual - d_desired)² pour tous les edges
    E_t = 0;
    for e = 1:num_edges
        i_node = edges(e, 1);
        j_node = edges(e, 2);
        % Distance actuelle
        d_actual = norm(x(1:2, i_node) - x(1:2, j_node));
        % Distance désirée (calculée depuis les offsets courants)
        d_desired = norm(current_offsets(i_node,:) - current_offsets(j_node,:));
        % Erreur au carré
        E_t = E_t + (d_actual - d_desired)^2;
    end
    formation_error(t) = E_t;

    % Data Saving - distances entre robots consécutifs
    for d = 1:N-1
        robot_distance(d,t) = norm(x(1:2,d) - x(1:2,d+1), 2);
    end
    robot_distance(N+1,t) = toc(start_time);  % Timestamp
    current_waypoint = waypoints(:, state);
    if(norm(x(1:2, 1) - current_waypoint) < close_enough)
        goal_distance = [goal_distance [norm(x(1:2, 1) - current_waypoint);toc(start_time)]];
    end
    
    r.step();
end

%% Sauvegarde des données
save('DistanceData.mat', 'robot_distance');
save('GoalData.mat', 'goal_distance');
save('FormationError.mat', 'formation_error', 'edges');

%% Affichage des statistiques de formation (Q2.ii)
fprintf('\n=== MÉTRIQUE DE FORMATION - RIGIDE (Q2.iii) ===\n');
fprintf('Topologie: DIAMANT RIGIDE (controle 7 edges, mesure 7 edges)\n');
fprintf('E(t) = Σ(d_actual - d_desired)² sur 7 edges de la forme\n\n');

% Ignorer les 200 premières itérations (transitoire)
steady_state = formation_error(200:end);

fprintf('Statistiques sur 7 edges [1-2, 1-3, 1-4, 2-3, 2-4, 3-4, 4-5]:\n');
fprintf('  Mean E(t):     %.6f\n', mean(steady_state));
fprintf('  Std E(t):      %.6f\n', std(steady_state));
fprintf('  Max E(t):      %.6f\n', max(steady_state));
fprintf('  Min E(t):      %.6f\n', min(steady_state));
fprintf('  RMSE:          %.4f m\n', sqrt(mean(steady_state)/num_edges));
fprintf('  Erreur finale: %.6f\n', formation_error(end));

%% Verification de FORME (pour comparaison avec non-rigide)
fprintf('\n=== VERIFICATION DE FORME (memes distances que non-rigide) ===\n');
x_final = r.get_poses();

% Distances desirees dans le diamant
d_13_desired = norm(h_diamond_local(1,:) - h_diamond_local(3,:));  % Leader-F2
d_14_desired = norm(h_diamond_local(1,:) - h_diamond_local(4,:));  % Leader-F3
d_24_desired = norm(h_diamond_local(2,:) - h_diamond_local(4,:));  % F1-F3

% Distances actuelles
d_13_actual = norm(x_final(1:2,1) - x_final(1:2,3));
d_14_actual = norm(x_final(1:2,1) - x_final(1:2,4));
d_24_actual = norm(x_final(1:2,2) - x_final(1:2,4));

fprintf('Distance Leader-F2 (1-3): desire=%.3f, actuel=%.3f, erreur=%.3f m\n', d_13_desired, d_13_actual, abs(d_13_actual-d_13_desired));
fprintf('Distance Leader-F3 (1-4): desire=%.3f, actuel=%.3f, erreur=%.3f m\n', d_14_desired, d_14_actual, abs(d_14_actual-d_14_desired));
fprintf('Distance F1-F3 (2-4):     desire=%.3f, actuel=%.3f, erreur=%.3f m\n', d_24_desired, d_24_actual, abs(d_24_actual-d_24_desired));

total_shape_error = (d_13_actual-d_13_desired)^2 + (d_14_actual-d_14_desired)^2 + (d_24_actual-d_24_desired)^2;
fprintf('\nErreur de FORME (3 distances de reference): %.6f\n', total_shape_error);

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