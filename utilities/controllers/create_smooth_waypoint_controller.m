function [smooth_waypoint_controller] = create_smooth_waypoint_controller(varargin)
% CREATE_SMOOTH_WAYPOINT_CONTROLLER
% Creates a smooth waypoint controller with Bezier curve cornering.
% Based on quadratic Bezier curves for smooth trajectory generation
% at waypoint corners.
%
% The controller operates in two modes:
%   1. STRAIGHT SEGMENT: Standard proportional control toward waypoint
%   2. BEZIER ZONE: Follows tangent of quadratic Bezier curve near corners
%
% Bezier curve: B(t) = (1-t)^2*P0 + 2*(1-t)*t*P1 + t^2*P2
%   P0 = robot position when entering the zone
%   P1 = waypoint (corner control point)
%   P2 = exit point on next segment
%
% Args:
%   LinearVelocityGain: Gain for linear velocity (default: 0.8)
%   AngularVelocityGain: Gain for angular velocity (default: 2.0)
%   PositionError: Distance threshold for waypoint switch (default: 0.2)
%   LookAheadGain: Weight for look-ahead (default: 0.5)
%   MinimumVelocity: Minimum linear velocity (default: 0.05)
%   VelocityMagnitudeLimit: Maximum velocity (default: 0.2)
%   BezierZoneRadius: Radius to activate Bezier mode (default: 0.5)
%   BezierExitLength: Distance from corner to P2 (default: 0.35)
%   BezierVelocity: Velocity in Bezier zone (default: 0.06)
%
% Returns:
%   smooth_waypoint_controller: Function handle [dxi, info] = controller(pose, waypoints, idx)

    p = inputParser;
    addOptional(p, 'LinearVelocityGain', 0.8);
    addOptional(p, 'AngularVelocityGain', 2.0);
    addOptional(p, 'PositionError', 0.2);
    addOptional(p, 'LookAheadGain', 0.5);
    addOptional(p, 'MinimumVelocity', 0.05);
    addOptional(p, 'VelocityMagnitudeLimit', 0.2);
    addOptional(p, 'BezierZoneRadius', 0.5);
    addOptional(p, 'BezierExitLength', 0.35);
    addOptional(p, 'BezierVelocity', 0.06);
    parse(p, varargin{:});

    k_v = p.Results.LinearVelocityGain;
    k_w = p.Results.AngularVelocityGain;
    pos_err = p.Results.PositionError;
    look_ahead_gain = p.Results.LookAheadGain;
    v_min = p.Results.MinimumVelocity;
    v_max = p.Results.VelocityMagnitudeLimit;
    bezier_zone_radius = p.Results.BezierZoneRadius;
    bezier_L = p.Results.BezierExitLength;
    bezier_velocity = p.Results.BezierVelocity;

    % Persistent state for Bezier mode
    persistent in_bezier_zone saved_P0 current_bezier_state;
    if isempty(in_bezier_zone)
        in_bezier_zone = false;
        saved_P0 = [0; 0];
        current_bezier_state = 0;
    end

    smooth_waypoint_controller = @controller;

    function [dxi, info] = controller(current_pose, waypoints, waypoint_idx)
        % CONTROLLER Smooth waypoint following with Bezier cornering
        %
        % Inputs:
        %   current_pose: [x; y; theta] current robot pose (3x1)
        %   waypoints: [x1 x2 ...; y1 y2 ...] waypoint positions (2xN)
        %   waypoint_idx: index of current target waypoint
        %
        % Outputs:
        %   dxi: [vx; vy] single-integrator velocity commands (2x1)
        %   info: struct with debug/visualization data

        % Extract current position
        robot_pos = current_pose(1:2);
        theta = current_pose(3);

        % Get current and next waypoints
        num_waypoints = size(waypoints, 2);
        wp_curr = waypoints(:, waypoint_idx);
        next_idx = mod(waypoint_idx, num_waypoints) + 1;
        wp_next = waypoints(:, next_idx);

        % Distance to current waypoint
        dist_to_wp = norm(wp_curr - robot_pos);

        % Initialize info struct
        info = struct();
        info.in_bezier_zone = false;
        info.bezier_completed = false;
        info.P0 = [NaN; NaN];
        info.P1 = [NaN; NaN];
        info.P2 = [NaN; NaN];
        info.t_robot = NaN;
        info.curve_points = [];
        info.target_point = [NaN; NaN];
        info.dist_to_wp = dist_to_wp;

        if dist_to_wp < bezier_zone_radius
            % === BEZIER ZONE MODE ===

            % Detect ENTRY into zone (new waypoint or first time)
            if ~in_bezier_zone || current_bezier_state ~= waypoint_idx
                % Just entered! Save current position as P0
                saved_P0 = robot_pos;
                in_bezier_zone = true;
                current_bezier_state = waypoint_idx;
            end

            % Direction toward next waypoint (exit direction)
            dir_out = (wp_next - wp_curr) / norm(wp_next - wp_curr);

            % Bezier control points
            P0 = saved_P0;                       % Entry point (where robot WAS)
            P1 = wp_curr;                        % Corner (control point)
            P2 = wp_curr + bezier_L * dir_out;   % Exit point (on next segment)

            % Calculate t (robot progress along curve)
            vec_P0_P2 = P2 - P0;
            vec_P0_robot = robot_pos - P0;
            t_robot = dot(vec_P0_robot, vec_P0_P2) / dot(vec_P0_P2, vec_P0_P2);
            t_robot = max(0, min(1.2, t_robot));  % Allow slight overshoot

            % TANGENT of Bezier curve at point t
            % B'(t) = 2(1-t)(P1-P0) + 2t(P2-P1)
            t_clamped = min(1, t_robot);
            tangent = 2*(1-t_clamped)*(P1-P0) + 2*t_clamped*(P2-P1);
            tangent_norm = tangent / norm(tangent);

            % === COMMAND: FOLLOW TANGENT ===
            dxi = bezier_velocity * tangent_norm;

            % Check if curve is completed (t > 0.95)
            bezier_completed = (t_robot > 0.95);

            % Generate curve points for visualization (20 points)
            t_vals = linspace(0, 1, 20);
            curve_points = zeros(2, 20);
            for k = 1:20
                tv = t_vals(k);
                % B(t) = (1-t)^2*P0 + 2*(1-t)*t*P1 + t^2*P2
                curve_points(:, k) = (1-tv)^2 * P0 + 2*(1-tv)*tv * P1 + tv^2 * P2;
            end

            % Fill info struct
            info.in_bezier_zone = true;
            info.bezier_completed = bezier_completed;
            info.P0 = P0;
            info.P1 = P1;
            info.P2 = P2;
            info.t_robot = t_robot;
            info.curve_points = curve_points;
            info.target_point = robot_pos + 0.15 * tangent_norm;

        else
            % === STRAIGHT SEGMENT MODE ===
            in_bezier_zone = false;

            % Standard proportional control toward waypoint
            x = robot_pos(1);
            y = robot_pos(2);

            % Compute position errors
            ex = wp_curr(1) - x;
            ey = wp_curr(2) - y;
            distance_to_wp = sqrt(ex^2 + ey^2);

            % Direction to current waypoint
            direction_current = atan2(ey, ex);

            % Look-ahead blending when close to waypoint
            if distance_to_wp < pos_err * 3
                direction_next = atan2(wp_next(2) - wp_curr(2), ...
                                        wp_next(1) - wp_curr(1));
                blend_factor = 1.0 - (distance_to_wp / (pos_err * 3));
                blend_factor = blend_factor * look_ahead_gain;
                angle_diff = direction_next - direction_current;
                angle_diff = atan2(sin(angle_diff), cos(angle_diff));
                theta_desired = direction_current + blend_factor * angle_diff;
            else
                theta_desired = direction_current;
            end

            % Velocity magnitude (proportional to distance)
            v_magnitude = k_v * distance_to_wp;

            % Apply saturation
            if v_magnitude > v_max
                v_magnitude = v_max;
            elseif v_magnitude < v_min
                v_magnitude = v_min;
            end

            % Reduce velocity when orientation error is large
            e_theta = theta_desired - theta;
            e_theta = atan2(sin(e_theta), cos(e_theta));
            orientation_factor = (1.0 + cos(e_theta)) / 2.0;
            v_magnitude = v_magnitude * (0.4 + 0.6 * orientation_factor);
            v_magnitude = max(v_min, v_magnitude);

            % Compute velocity in desired direction
            vx = v_magnitude * cos(theta_desired);
            vy = v_magnitude * sin(theta_desired);
            dxi = [vx; vy];

            % Check for waypoint switch (fallback, should rarely trigger)
            if dist_to_wp < pos_err
                info.bezier_completed = true;
            end
        end
    end
end
