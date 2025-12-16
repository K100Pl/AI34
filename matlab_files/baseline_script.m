%% INITIAL SETUP
clear all; clc;

% Note: Assurez-vous que les messages ROS personnalisés sont compilés
% avant de lancer ce script (ros2genmsg).

% % ======== Robots to work with ========== % %
% On initialise seulement la QCar 1 pour cet exercice
try
    qcar_01 = ros2Setup(1);
    disp('QCar 1 Initialized successfully.');
catch ME
    disp('Error initializing QCar. Make sure ROS 2 is running and ID is correct.');
    rethrow(ME);
end

pause(2); % Wait for the Optitrack messages to arrive and stabilize.

%% COMMUNICATION TESTS
% Affiche la pose actuelle pour vérifier que l'Optitrack fonctionne
fprintf('Initial Pose -> X: %.2f | Y: %.2f | Theta: %.2f \n', ...
    qcar_01.pose.x, qcar_01.pose.y, qcar_01.pose.theta);

%% EXAMPLE FOR ONE SINGLE QCAR TRACKING A SINGLE WAYPOINT 
% Configuration de la boucle de contrôle

dt = 0.05; % Fréquence de contrôle (20Hz)

% --- VARIABLES LOCALES & PARAMÈTRES --- %

% 1. Définir le point cible (Waypoint) en mètres
% CHANGE CES VALEURS selon la taille de ton arène !
target_x = 1.5; 
target_y = 0.0; 

% 2. Gains du contrôleur (Proportionnel)
Kv = 0.8;  % Gain de vitesse linéaire
Kw = 1.2;  % Gain de braquage (steering)

% 3. Limites de sécurité du QCar
v_max = 0.5;        % Vitesse max (m/s) - pour la sécurité
steering_max = 0.5; % Braquage max (rad) - env. 30 degrés
dist_tol = 0.15;    % Tolérance d'arrivée (15 cm)

disp('Starting Control Loop...');

while(1)
    
    % --- 1. ACQUISITION DES DONNÉES --- %
    current_x = qcar_01.pose.x;
    current_y = qcar_01.pose.y;
    current_th = qcar_01.pose.theta;
    
    % Sécurité : Si l'Optitrack perd le robot (NaN), on arrête tout
    if isnan(current_x) || isnan(current_y)
        qcar_01.sendCommand(0, 0);
        disp('Warning: Lost Optitrack tracking! Stopping.');
        pause(dt);
        continue;
    end

    % --- 2. CALCUL DES ERREURS --- %
    % Vecteur vers la cible
    dx = target_x - current_x;
    dy = target_y - current_y;
    
    % Distance Euclidienne (Erreur de position)
    dist_error = sqrt(dx^2 + dy^2);
    
    % Cap désiré (Angle vers la cible)
    desired_heading = atan2(dy, dx);
    
    % Erreur de cap (Orientation)
    heading_error = desired_heading - current_th;
    
    % Normalisation de l'angle entre [-pi, pi] (très important !)
    heading_error = atan2(sin(heading_error), cos(heading_error));

    % --- 3. CONDITION D'ARRÊT --- %
    if(dist_error <= dist_tol)
        qcar_01.sendCommand(0, 0);   % Stop the car
        disp('TARGET REACHED! Stopping program.');
        break;
    end

    % --- 4. LOIS DE COMMANDE (P-Controller) --- %
    
    % Vitesse : proportionnelle à la distance
    cmd_velocity = Kv * dist_error;
    
    % Direction : proportionnelle à l'erreur d'angle
    cmd_steering = Kw * heading_error;
    
    % --- 5. SATURATIONS (Sécurité physique) --- %
    
    % Limiter la vitesse (entre 0 et v_max)
    cmd_velocity = min(max(cmd_velocity, 0), v_max);
    
    % Inverser la marche si la cible est derrière ? (Optionnel, ici on avance seulement)
    % Pour une conduite simple type voiture, si l'angle est > 90deg, 
    % on peut s'arrêter et tourner, mais ici on suppose une trajectoire fluide.
    
    % Limiter le braquage (entre -steering_max et +steering_max)
    cmd_steering = min(max(cmd_steering, -steering_max), steering_max);
    
    % --- 6. ENVOI DE LA COMMANDE --- %
    qcar_01.sendCommand(cmd_steering, cmd_velocity);

    % Affichage pour débug (optionnel)
    % fprintf('Dist: %.2f | HedErr: %.2f | CmdV: %.2f | CmdS: %.2f\n', ...
    %         dist_error, heading_error, cmd_velocity, cmd_steering);

    pause(dt);

end