%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file defines the QCar class
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef QCar < handle
    properties
        id          % QCar ID (1,2,3)
        namespace   % ROS2 namespace (/qcar_01, etc.)
        node        % ros2node
        publisher   % ros2publisher
        subscriber  % ros2subscriber
        pose        % struct with the 2D pose of the robot (x, y, theta)

        % % ===== Physical Properties ===== % %
        mass        
        wheelBase   
        trackWidth

        % % ===== IF NEEDED, ADD MORE PROPERTIES HERE ===== % %
        

    

    end
    
    methods
        function obj = QCar(id, namespace, node, publisher)

            obj.id        = id;
            obj.namespace = namespace;
            obj.node      = node;
            obj.publisher = publisher;
            
            obj.pose.x     = NaN;
            obj.pose.y     = NaN;
            obj.pose.theta = NaN;

            obj.mass = 2.7; % Kg
            obj.wheelBase = 0.39; % m
            obj.trackWidth = 0.20; % m
        end

        function sendCommand(obj, steering, velocity)
            
            msg = ros2message("qcar2_interfaces/MotorCommands");
            msg.motor_names  = ({'steering_angle', 'motor_throttle'});
            msg.values = [steering, velocity];
            
            send(obj.publisher, msg);    % Publishing commands
        end

        % % ===== IF NEEDED, ADD MORE METHODS HERE ===== % %





    end
end
