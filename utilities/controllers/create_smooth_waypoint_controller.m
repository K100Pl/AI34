function [smooth_waypoint_controller] = create_smooth_waypoint_controller(varargin)
% CREATE_SMOOTH_WAYPOINT_CONTROLLER
% Creates a controller for smooth waypoint navigation without stop-and-turn behavior.
% Based on "A novel safe and flexible control strategy based on target reaching
% for the navigation of urban vehicles" by Vilca, Adouane, Mezouar (2015).
%
% This controller enables smooth navigation between waypoints by:
%   - Anticipating the next waypoint direction (look-ahead)
%   - Maintaining non-zero velocity during waypoint transitions
%   - Smooth orientation control without stopping to turn
%
% Args:
%   LinearVelocityGain: Gain for linear velocity control (default: 0.8)
%   AngularVelocityGain: Gain for angular velocity control (default: 2.0)
%   PositionError: Distance threshold to switch to next waypoint (default: 0.1)
%   LookAheadGain: Weight for look-ahead direction (default: 0.5)
%   MinimumVelocity: Minimum linear velocity to maintain (default: 0.05)
%   VelocityMagnitudeLimit: Maximum velocity magnitude (default: 0.2)
%
% Returns:
%   smooth_waypoint_controller: Function handle for the controller
%
% Usage:
%   controller = create_smooth_waypoint_controller();
%   dxu = controller(current_pose, waypoints, current_waypoint_idx);

    p = inputParser;
    addOptional(p, 'LinearVelocityGain', 0.8);
    addOptional(p, 'AngularVelocityGain', 2.0);
    addOptional(p, 'PositionError', 0.1);
    addOptional(p, 'LookAheadGain', 0.5);
    addOptional(p, 'MinimumVelocity', 0.05);
    addOptional(p, 'VelocityMagnitudeLimit', 0.2);
    parse(p, varargin{:});

    k_v = p.Results.LinearVelocityGain;
    k_w = p.Results.AngularVelocityGain;
    pos_err = p.Results.PositionError;
    look_ahead_gain = p.Results.LookAheadGain;
    v_min = p.Results.MinimumVelocity;
    v_max = p.Results.VelocityMagnitudeLimit;

    smooth_waypoint_controller = @controller;

    function dxi = controller(current_pose, waypoints, waypoint_idx)
        % CONTROLLER Smooth waypoint following control law
        %
        % Inputs:
        %   current_pose: [x; y; theta] current robot pose (3x1)
        %   waypoints: [x1 x2 ...; y1 y2 ...] waypoint positions (2xN)
        %   waypoint_idx: index of current target waypoint
        %
        % Outputs:
        %   dxi: [vx; vy] single-integrator velocity commands (2x1)

        % Extract current state
        x = current_pose(1);
        y = current_pose(2);
        theta = current_pose(3);

        % Get current waypoint
        num_waypoints = size(waypoints, 2);
        current_wp = waypoints(:, waypoint_idx);

        % Get next waypoint (circular indexing)
        next_idx = mod(waypoint_idx, num_waypoints) + 1;
        next_wp = waypoints(:, next_idx);

        % Compute position errors with respect to current waypoint
        ex = current_wp(1) - x;
        ey = current_wp(2) - y;
        distance_to_wp = sqrt(ex^2 + ey^2);

        % Compute direction to current waypoint
        direction_current = atan2(ey, ex);

        % Compute desired direction considering look-ahead
        if distance_to_wp < pos_err * 3
            % Close to waypoint: blend current and next waypoint direction
            direction_next = atan2(next_wp(2) - current_wp(2), ...
                                    next_wp(1) - current_wp(1));

            % Smooth blending: more look-ahead as we get closer
            blend_factor = 1.0 - (distance_to_wp / (pos_err * 3));
            blend_factor = blend_factor * look_ahead_gain;

            % Blend the angles properly (handle angle wrapping)
            angle_diff = direction_next - direction_current;
            angle_diff = atan2(sin(angle_diff), cos(angle_diff));
            theta_desired = direction_current + blend_factor * angle_diff;
        else
            % Far from waypoint: head directly towards it
            theta_desired = direction_current;
        end

        % Compute velocity magnitude (proportional to distance)
        v_magnitude = k_v * distance_to_wp;

        % Apply smooth saturation
        if v_magnitude > v_max
            v_magnitude = v_max;
        elseif v_magnitude < v_min
            % Maintain minimum velocity for smooth motion
            v_magnitude = v_min;
        end

        % Compute orientation error (wrap to [-pi, pi])
        e_theta = theta_desired - theta;
        e_theta = atan2(sin(e_theta), cos(e_theta));

        % Reduce velocity when orientation error is large
        % This creates smooth turns without stopping
        orientation_factor = (1.0 + cos(e_theta)) / 2.0; % Range [0, 1]
        v_magnitude = v_magnitude * (0.4 + 0.6 * orientation_factor);

        % Ensure minimum velocity to avoid stopping
        v_magnitude = max(v_min, v_magnitude);

        % Compute single-integrator velocity in desired direction
        vx = v_magnitude * cos(theta_desired);
        vy = v_magnitude * sin(theta_desired);

        % Output single-integrator commands
        dxi = [vx; vy];
    end
end
