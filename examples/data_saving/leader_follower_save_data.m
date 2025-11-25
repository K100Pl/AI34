% Script leader_follower - Version Sans Obstacles
% Focus: Q1 (Smooth Leader) & Q2 (Rigid Diamond Formation)

%% Experiment Constants
iterations = 3000;

%% Set up the Robotarium object
N = 4;
% Initialisation avec espacement suffisant pour éviter le blocage au départ
initial_positions = generate_initial_conditions(N, 'Width', 1.2, 'Height', 0.8, 'Spacing', 0.4);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

%% Topologie Rigide (Diamant) - Q2
% 1=Leader, 2=F1, 3=F2, 4=F3 (Queue)
% Le Robot 4 est connecté à 2 ET 3 pour verrouiller la forme
L = zeros(N, N);
L(2, 2) = 1; L(2, 1) = -1;       % 2 suit 1
L(3, 3) = 1; L(3, 1) = -1;       % 3 suit 1
L(4, 4) = 2; L(4, 2) = -1; L(4, 3) = -1; % 4 suit 2 et 3

dxi = zeros(2, N);
state = 1;

% Gain modéré pour éviter que les followers ne doublent le leader
formation_control_gain = 3; 

% Offsets pour la formation Diamant (Position relative vs Leader)
h_diamond = [ 0,    0;      
             -0.3, -0.3;    
              0.3, -0.3;    
              0,   -0.6];   
              
current_offsets = h_diamond; 

%% Tools
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.8);
uni_barrier_cert = create_uni_barrier_certificate_with_boundary();
% Leader limité à 0.15 m/s pour laisser le temps aux followers de suivre
leader_controller = create_si_position_controller('XVelocityGain', 0.8, 'YVelocityGain', 0.8, 'VelocityMagnitudeLimit', 0.15);

% Waypoints resserrés pour ne pas sortir du cadre (cadre approx [-1.6, 1.6])
waypoints = [-0.9 0.5; 0.9 0.5; 0.9 -0.5; -0.9 -0.5]';
close_enough = 0.2; % Rayon de transition

%% Plotting Setup
CM = ['k','b','r','g'];
marker_size_goal = determine_marker_size(r, 0.20);
font_size = determine_font_size(r, 0.05);
line_width = 3;

% Affichage des Waypoints
for i = 1:length(waypoints)
    goal_caption = sprintf('G%d', i);
    plot(waypoints(1,i), waypoints(2,i),'s','MarkerSize',marker_size_goal,'LineWidth',line_width,'Color',CM(i));
    text(waypoints(1,i)-0.05, waypoints(2,i), goal_caption, 'FontSize', font_size, 'FontWeight', 'bold');
end

x=r.get_poses();
% Lignes bleues de connexion entre robots
[rows, cols] = find(L == -1); 
for k = 1:length(rows)
   lf(k) = line([x(1,rows(k)), x(1,cols(k))],[x(2,rows(k)), x(2,cols(k))], 'LineWidth', line_width, 'Color', 'b'); 
end

leader_label = text(500, 500, 'Leader', 'FontSize', font_size, 'FontWeight', 'bold', 'Color', 'r');
for j = 1:N-1    
    follower_labels{j} = text(500, 500, sprintf('F%d', j), 'FontSize', font_size, 'FontWeight', 'bold');
end

%% Data Saving Setup
robot_distance = zeros(5,iterations); 
goal_distance = []; 
start_time = tic;

% Petite impulsion initiale pour initialiser les matrices
dxi(:, 1) = [0.01; 0.01];
r.step();

%% BOUCLE PRINCIPALE
for t = 1:iterations
    
    x = r.get_poses();
    
    %% 1. COMMANDE LEADER (Q1 - Smooth Navigation)
    waypoint = waypoints(:, state);
    
    % Calcul vitesse leader vers le waypoint
    dxi(:, 1) = leader_controller(x(1:2, 1), waypoint);
    
    % Logique de changement de waypoint fluide (sans arrêt)
    if(norm(x(1:2, 1) - waypoint) < close_enough)
        state = mod(state, 4) + 1; 
    end
    
    %% 2. COMMANDE FOLLOWERS (Q2 - Rigid Formation)
    for i = 2:N
        dxi(:, i) = [0 ; 0];
        
        % Trouver qui je dois suivre (voisins dans la matrice L)
        neighbors = find(L(i,:) == -1);
        
        for j = neighbors
            % Consensus : On veut maintenir la distance relative définie par les offsets
            pos_error = x(1:2, j) - x(1:2, i);
            offset_error = current_offsets(j,:)' - current_offsets(i,:)';
            
            % Correction proportionnelle à l'erreur
            dxi(:, i) = dxi(:, i) + formation_control_gain * (pos_error - offset_error);
        end
        
        % Feedforward : On ajoute la vitesse du leader pour anticiper le mouvement
        dxi(:, i) = dxi(:, i) + dxi(:, 1); 
    end
    
    %% Limits & Dynamics
    % Limiter la vitesse max pour éviter les mouvements brusques
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
    
    % Conversion en dynamique Unicycle (v, omega)
    dxu = si_to_uni_dyn(dxi, x);
    
    % Barrière de sécurité (Anti-collision entre robots)
    dxu = uni_barrier_cert(dxu, x);
    
    % Envoi des commandes
    r.set_velocities(1:N, dxu);
    
    %% Update Plots
    for q = 1:N-1
        follower_labels{q}.Position = x(1:2, q+1) + [-0.1;0.1];     
    end
    
    % Mise à jour des lignes du graphe
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
    
    %% Data Saving
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

%% Helper Functions (NE PAS TOUCHER)
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