% SY28 Project - Hybrid Control (Real QCar Leader + Sim Followers)
% Q1: Smooth navigation (Vilca 2015) sur QCar
% Q2: Formation diamant rigide (QCar + 4 Virtual Agents)
% Q3: Évitement d'obstacles
% Q4: Reconfiguration

clear; clc; close all;

%% 1. ROS 2 CONFIGURATION (Connexion au QCar)
% Assurez-vous que le DOMAIN_ID correspond à votre QCar
try
    ros2node_obj = ros2node("/matlab_central_node", 0);
catch
    % Si le noeud existe déjà
    ros2node_obj = ros2node("/matlab_central_node_restart", 0); 
end

% Publisher : Commande vitesse pour le QCar
% Adapter le topic si besoin (ex: '/qcar/cmd_vel' ou '/cmd_vel')
cmd_pub = ros2publisher(ros2node_obj, "/qcar/cmd_vel", "geometry_msgs/Twist");
cmd_msg = ros2message(cmd_pub);

% Subscriber : Odométrie ou Optitrack du QCar
% Adapter le topic (ex: '/qcar/odom', '/optitrack/pose', etc.)
odom_sub = ros2subscriber(ros2node_obj, "/qcar/odom", "nav_msgs/Odometry");

pause(1); % Attente connexion

%% 2. Experiment Constants & Setup
iterations = 5000;
dt = 0.05; % Pas de temps (20Hz)

N = 5;  % 1 Réel (Leader) + 4 Virtuels
state = 1; % Etat pour les waypoints
formation_control_gain = 4;

% Initialisation des positions virtuelles (Followers uniquement)
% Le Leader (x(:,1)) sera écrasé par la vraie position
x = zeros(3, N); 
% Position initiale arbitraire pour les followers (autour de 0,0)
x(:, 2) = [-0.5; -0.5; 0];
x(:, 3) = [-0.5; 0.5; 0];
x(:, 4) = [-1.0; 0; 0];
x(:, 5) = [-1.5; 0; 0];

%% 3. Topologie (Identique version simu)
L = zeros(N, N);
L(2, 2) = 2; L(2, 1) = -1; L(2, 3) = -1;
L(3, 3) = 2; L(3, 1) = -1; L(3, 2) = -1;
L(4, 4) = 3; L(4, 1) = -1; L(4, 2) = -1; L(4, 3) = -1;
L(5, 5) = 1; L(5, 4) = -1;

% Offsets
h_diamond = [ 0, 0; -0.4, -0.4; 0.4, -0.4; 0, -0.8; 0, -1.2]; % Écarté un peu pour le vrai robot
h_platoon = [ 0, 0; 0, -0.5; 0, -1.0; 0, -1.5; 0, -2.0];
current_offsets = h_diamond;

% Obstacles & Waypoints
obstacles = [-0.5, 0.0; 0.5, 0.0];
detect_radius = 0.8;    
safe_radius = 0.35; % Un peu plus large pour le vrai robot
repulsion_gain = 0.1;

% Waypoints (Adaptés à l'espace physique du labo ?)
waypoints = [1.5 0.5; 1.5 -0.5; -0.5 -0.5; -0.5 0.5]'; 
close_enough = 0.3;

%% 4. Visualization Setup
figure('Name', 'Hybrid Control: Real QCar + Sim Followers', 'Color', 'w');
hold on; axis equal; grid on;
xlim([-2 3]); ylim([-2 2]);
xlabel('X (m)'); ylabel('Y (m)');

% Plot Waypoints & Obstacles
plot(waypoints(1,:), waypoints(2,:), 'gs', 'MarkerSize', 10, 'LineWidth', 2);
plot(obstacles(:,1), obstacles(:,2), 'ko', 'MarkerSize', 20, 'MarkerFaceColor', 'k');

% Robots Handles
colors = ['r', 'b', 'b', 'b', 'b']; % Rouge = Vrai QCar
for i = 1:N
    h_robots(i) = plot(x(1,i), x(2,i), 'o', 'MarkerSize', 12, 'MarkerFaceColor', colors(i));
    h_text(i) = text(x(1,i), x(2,i), sprintf('  R%d', i));
end
h_arrow = quiver(x(1,1), x(2,1), cos(x(3,1)), sin(x(3,1)), 0.5, 'r', 'LineWidth', 2); % Cap du QCar

%% 5. BOUCLE DE CONTRÔLE PRINCIPALE
rate = ros2rate(ros2node_obj, 1/dt); % Maintient la fréquence
dxi = zeros(2, N);

disp('Démarrage du contrôle... Appuyez sur Ctrl+C pour arrêter.');

try
    for t = 1:iterations
        
        %% A. Acquisition Données Réelles (QCar)
        latest_odom = odom_sub.LatestMessage;
        if isempty(latest_odom)
            warning('Pas de données odométrie reçues du QCar !');
            continue; 
        end
        
        % Extraction position/orientation (Quaternion -> Euler)
        pos = latest_odom.pose.pose.position;
        quat = latest_odom.pose.pose.orientation;
        eul = quat2eul([quat.w quat.x quat.y quat.z]); % [Z Y X]
        theta_real = eul(1);
        
        % MISE A JOUR ETAT LEADER (Ecrasement par le réel)
        x(1,1) = pos.x;
        x(2,1) = pos.y;
        x(3,1) = theta_real;
        
        %% B. Logique de Reconfiguration
        dist_to_obs = min(sqrt(sum((x(1:2,1) - obstacles').^2)));
        if dist_to_obs < detect_radius
            current_offsets = h_platoon;
            title('Mode: PLATOON (Obstacle Detected)');
        else
            current_offsets = h_diamond;
            title('Mode: DIAMOND');
        end
        
        %% C. Calcul Commande LEADER (QCar)
        % Algorithme Smooth Waypoint (Vilca)
        err_pos = waypoints(:, state) - x(1:2, 1);
        dist_target = norm(err_pos);
        
        if dist_target < close_enough
            state = mod(state, size(waypoints, 2)) + 1;
        end
        
        % Commande simple P (ou Smooth Vilca simplifié pour ROS)
        v_des = 0.3; % Vitesse constante désirée (m/s)
        desired_heading = atan2(err_pos(2), err_pos(1));
        heading_error = angdiff(x(3,1), desired_heading);
        
        omega_des = 1.5 * heading_error; % Gain P angulaire
        
        % Saturation pour sécurité
        v_cmd = min(max(v_des, 0), 0.5); 
        w_cmd = min(max(omega_des, -1.0), 1.0);
        
        % Envoi ROS au QCar
        cmd_msg.linear.x = v_cmd;
        cmd_msg.angular.z = w_cmd;
        send(cmd_pub, cmd_msg);
        
        % Note: On stocke la vitesse linéaire demandée dans dxi pour les followers
        dxi(:, 1) = [v_cmd * cos(x(3,1)); v_cmd * sin(x(3,1))];
        
        %% D. Calcul Commande FOLLOWERS (Simulés)
        for i = 2:N
            dxi(:, i) = [0 ; 0];
            neighbors = find(L(i,:) == -1);
            
            for j = neighbors
                pos_error = x(1:2, j) - x(1:2, i);
                offset_error = current_offsets(j,:)' - current_offsets(i,:)';
                % Terme consensus
                dxi(:, i) = dxi(:, i) + formation_control_gain * (pos_error - offset_error);
            end
            
            % Feedforward du leader (pour suivre le mouvement)
            dxi(:, i) = dxi(:, i) + dxi(:, 1); 
            
            % Evitement obstacles (Followers)
            for k = 1:size(obstacles, 1)
                vect = x(1:2, i) - obstacles(k,:)';
                dist = norm(vect);
                if dist < safe_radius
                     dxi(:, i) = dxi(:, i) + repulsion_gain * (vect / dist^3);
                end
            end
            
            % Saturation vitesse followers
            v_max_sim = 0.6;
            if norm(dxi(:, i)) > v_max_sim
                dxi(:, i) = dxi(:, i) / norm(dxi(:, i)) * v_max_sim;
            end
        end
        
        %% E. Intégration Dynamique (Mise à jour Simulée)
        % Euler simple pour les robots virtuels (Single Integrator)
        x(1:2, 2:N) = x(1:2, 2:N) + dxi(:, 2:N) * dt;
        % (Optionnel) Mise à jour theta virtuel si on veut simuler Unicycle
        % x(3, i) = atan2(dxi(2,i), dxi(1,i));
        
        %% F. Update Plot
        for i = 1:N
            h_robots(i).XData = x(1,i);
            h_robots(i).YData = x(2,i);
            h_text(i).Position = [x(1,i)+0.1, x(2,i)+0.1];
        end
        % Flèche direction du QCar
        h_arrow.XData = x(1,1); h_arrow.YData = x(2,1);
        h_arrow.UData = cos(x(3,1))*0.5; h_arrow.VData = sin(x(3,1))*0.5;
        
        drawnow limitrate;
        waitfor(rate);
    end

catch ME
    disp('Erreur rencontrée, arrêt d urgence.');
    disp(ME.message);
end

%% Arrêt d'urgence final
disp('Arrêt du QCar.');
cmd_msg.linear.x = 0;
cmd_msg.angular.z = 0;
send(cmd_pub, cmd_msg);