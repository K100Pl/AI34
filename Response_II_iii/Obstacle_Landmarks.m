% SY28 Project - Section II.iii: Observation de Landmarks
% Les robots corrigent leur pose estimee en observant des landmarks fixes
% Objectif: Montrer que les landmarks permettent de limiter le drift

%% Experiment Constants
iterations = 3000;
dt = 0.033;  % Robotarium timestep (~30Hz)

%% PARAMETRES DE BRUIT ODOMETRIE (II.ii)
sigma_v = 0.01;      % Ecart-type bruit vitesse lineaire (m/s)
sigma_omega = 0.02;  % Ecart-type bruit vitesse angulaire (rad/s)

%% LANDMARKS (II.iii)
% 2 landmarks aux coins opposes (moins de corrections)
landmarks = [-0.8,  0.4;    % Proche waypoint 1
              0.8, -0.4];   % Proche waypoint 3
observation_radius = 0.25;  % Distance max pour observer un landmark

%% Set up the Robotarium object
N = 5;  % 1 leader + 4 followers (Q2 requirement)
rng(42);  % Seed fixe pour reproductibilite
initial_positions = generate_initial_conditions(N, 'Width', 1.5, 'Height', 1.0, 'Spacing', 0.3);
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

% Offsets Diamant (5 robots) - formation plus compacte
h_diamond = [ 0,      0;       % Leader
             -0.20,  -0.20;    % F1 (gauche-haut)
              0.20,  -0.20;    % F2 (droite-haut)
              0,     -0.40;    % F3 (centre)
              0,     -0.60];   % F4 (queue)

% Offsets Platoon (5 robots) - Q4 - plus compact pour rester dans les limites
h_platoon = [ 0,      0;       % Leader
              0,     -0.22;    % F1
              0,     -0.44;    % F2
              0,     -0.66;    % F3
              0,     -0.88];   % F4 (total: 0.88m au lieu de 1.12m)
              
current_offsets = h_diamond; 

% --- OBSTACLES ---
% Positionnés loin du spawn (centre) mais sur le chemin entre waypoints
obstacles = [-0.5, 0.0;   % Côté gauche, entre WP1/WP4
              0.5, 0.0];  % Côté droit, entre WP2/WP3 
detect_radius = 0.6;    
safe_radius = 0.25;     
repulsion_gain = 0.05;  

%% Tools
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.8);
uni_barrier_cert = create_uni_barrier_certificate_with_boundary();

% Smooth waypoint controller based on Vilca et al. (2015)
smooth_leader_controller = create_smooth_waypoint_controller(...
    'LinearVelocityGain', 0.5, ...
    'PositionError', 0.2, ...
    'LookAheadGain', 0.6, ...
    'MinimumVelocity', 0.08, ...
    'VelocityMagnitudeLimit', 0.15);

waypoints = [-0.9 0.5; 0.9 0.5; 0.9 -0.5; -0.9 -0.5]';
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

%% ODOMETRIE - Initialisation (II.i)
% Position initiale connue parfaitement (x deja obtenu pour plotting)
x_estimated = x;  % Pose estimee = pose reelle au depart (x de ligne 88)

% Stockage erreur odometrie
odometry_error = zeros(N, iterations);  % Erreur par robot
total_odom_error = zeros(1, iterations);  % Erreur totale

% Compteur de corrections par landmark (II.iii)
landmark_corrections = zeros(N, 1);  % Nombre de corrections par robot

dxi(:, 1) = [0.01; 0.01];
r.step();  % Reset le flag get_poses

%% BOUCLE PRINCIPALE
for t = 1:iterations

    x_real = r.get_poses();  % Position REELLE (seul appel autorise par step)

    %% Calcul erreur odometrie AVANT mise a jour
    for i = 1:N
        odometry_error(i, t) = norm(x_estimated(1:2, i) - x_real(1:2, i));
    end
    total_odom_error(t) = sum(odometry_error(:, t));

    x = x_estimated;         % On utilise la position ESTIMEE pour le controle

    %% 1. RECONFIGURATION (Q4) - utilise x_estimated
    dist_to_obs = min(sqrt(sum((x(1:2,1) - obstacles').^2)));
    
    if dist_to_obs < detect_radius
        current_offsets = h_platoon; 
        leader_label.String = 'Leader (PLATOON)';
    else
        current_offsets = h_diamond; 
        leader_label.String = 'Leader (DIAMOND)';
    end

    %% 2. COMMANDE LEADER - Smooth Navigation (Q1)
    % Use smooth controller with full pose (x, y, theta) and look-ahead
    dxi(:, 1) = smooth_leader_controller(x(:, 1), waypoints, state);

    waypoint = waypoints(:, state);
    if(norm(x(1:2, 1) - waypoint) < close_enough)
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

    %% ODOMETRIE BRUITEE - Integration des vitesses avec bruit (II.ii)
    % Modele unicycle: x' = v*cos(theta), y' = v*sin(theta), theta' = omega
    % Bruit gaussien ajoute sur les mesures de vitesse
    for i = 1:N
        v = dxu(1, i);      % Vitesse lineaire commandee
        omega = dxu(2, i);  % Vitesse angulaire commandee
        theta = x_estimated(3, i);

        % Ajout du bruit sur les vitesses mesurees
        v_noisy = v + sigma_v * randn();
        omega_noisy = omega + sigma_omega * randn();

        % Integration avec vitesses bruitees (Euler)
        x_estimated(1, i) = x_estimated(1, i) + v_noisy * cos(theta) * dt;
        x_estimated(2, i) = x_estimated(2, i) + v_noisy * sin(theta) * dt;
        x_estimated(3, i) = x_estimated(3, i) + omega_noisy * dt;
    end

    %% OBSERVATION DE LANDMARKS (II.iii)
    % Quand un robot voit un landmark, il corrige sa position estimee
    for i = 1:N
        for k = 1:size(landmarks, 1)
            dist_to_landmark = norm(x_real(1:2, i) - landmarks(k,:)');

            if dist_to_landmark < observation_radius
                % OBSERVATION PARFAITE: le robot voit le landmark
                % et corrige sa position estimee
                x_estimated(1:2, i) = x_real(1:2, i);  % Reset position
                x_estimated(3, i) = x_real(3, i);      % Reset orientation
                landmark_corrections(i) = landmark_corrections(i) + 1;
            end
        end
    end

    % Stocker x_real pour comparaison finale
    x_real_last = x_real;

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
    if(norm(x(1:2, 1) - waypoint) < close_enough)
        goal_distance = [goal_distance [norm(x(1:2, 1) - waypoint);toc(start_time)]];
    end
    
    r.step();
end

%% Sauvegarde des donnees
save('OdometryError_Landmarks.mat', 'odometry_error', 'total_odom_error', 'sigma_v', 'sigma_omega', 'landmark_corrections');

%% Affichage des statistiques ODOMETRIE (II.iii)
fprintf('\n=== SECTION II.iii: OBSERVATION DE LANDMARKS ===\n');
fprintf('Odometrie bruitee + correction par landmarks\n');
fprintf('sigma_v = %.3f m/s, sigma_omega = %.3f rad/s\n', sigma_v, sigma_omega);
fprintf('Landmarks: %d points, rayon observation = %.2f m\n', size(landmarks,1), observation_radius);
fprintf('Erreur = ||x_estimated - x_real||\n\n');

fprintf('Corrections par landmark (par robot):\n');
for i = 1:N
    fprintf('  Robot %d: %d corrections\n', i, landmark_corrections(i));
end
fprintf('  Total:   %d corrections\n', sum(landmark_corrections));

fprintf('\nErreur odometrie par robot (moyenne sur simulation):\n');
for i = 1:N
    fprintf('  Robot %d: %.6f m\n', i, mean(odometry_error(i, :)));
end

fprintf('\nStatistiques erreur totale:\n');
fprintf('  Mean:     %.6f m\n', mean(total_odom_error));
fprintf('  Std:      %.6f m\n', std(total_odom_error));
fprintf('  Max:      %.6f m\n', max(total_odom_error));
fprintf('  Min:      %.6f m\n', min(total_odom_error));
fprintf('  Finale:   %.6f m\n', total_odom_error(end));

fprintf('\nEvolution de l''erreur dans le temps:\n');
fprintf('  t=500:    %.6f m\n', total_odom_error(500));
fprintf('  t=1000:   %.6f m\n', total_odom_error(1000));
fprintf('  t=1500:   %.6f m\n', total_odom_error(1500));
fprintf('  t=2000:   %.6f m\n', total_odom_error(2000));
fprintf('  t=2500:   %.6f m\n', total_odom_error(2500));
fprintf('  t=3000:   %.6f m\n', total_odom_error(3000));

fprintf('\nCOMPARAISON II.ii vs II.iii:\n');
fprintf('  II.ii (sans landmarks): Mean ~0.114m, Finale ~0.175m\n');
fprintf('  II.iii (avec landmarks): Mean %.3fm, Finale %.3fm\n', mean(total_odom_error), total_odom_error(end));

fprintf('\nComparaison position finale:\n');
for i = 1:N
    err = norm(x_estimated(1:2, i) - x_real_last(1:2, i));
    fprintf('  Robot %d: estimated=(%.3f, %.3f), real=(%.3f, %.3f), erreur=%.4f m\n', ...
        i, x_estimated(1, i), x_estimated(2, i), x_real_last(1, i), x_real_last(2, i), err);
end

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