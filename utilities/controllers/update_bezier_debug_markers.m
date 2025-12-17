function update_bezier_debug_markers(markers, info, leader_pos)
% UPDATE_BEZIER_DEBUG_MARKERS
% Updates visual debug markers for Bezier curve visualization.
%
% Args:
%   markers: struct with graphics handles
%       .zone_marker: red circle on leader when in zone
%       .P0_marker: green square for P0 (entry point)
%       .P1_marker: red square for P1 (waypoint/corner)
%       .P2_marker: blue square for P2 (exit point)
%       .bezier_curve: magenta line for the curve
%       .target_marker: cyan star for tangent direction
%   info: struct from controller with Bezier data
%       .in_bezier_zone: boolean
%       .P0, .P1, .P2: control points (2x1)
%       .curve_points: curve coordinates (2x20)
%       .target_point: target visualization point (2x1)
%   leader_pos: current leader position (2x1)

    if info.in_bezier_zone
        % Show zone marker on leader
        markers.zone_marker.XData = leader_pos(1);
        markers.zone_marker.YData = leader_pos(2);

        % Show control points P0, P1, P2
        markers.P0_marker.XData = info.P0(1);
        markers.P0_marker.YData = info.P0(2);
        markers.P1_marker.XData = info.P1(1);
        markers.P1_marker.YData = info.P1(2);
        markers.P2_marker.XData = info.P2(1);
        markers.P2_marker.YData = info.P2(2);

        % Show Bezier curve
        markers.bezier_curve.XData = info.curve_points(1, :);
        markers.bezier_curve.YData = info.curve_points(2, :);

        % Show target direction
        markers.target_marker.XData = info.target_point(1);
        markers.target_marker.YData = info.target_point(2);
    else
        % Hide all markers (move off-screen)
        markers.zone_marker.XData = 10;
        markers.zone_marker.YData = 10;
        markers.P0_marker.XData = 10;
        markers.P0_marker.YData = 10;
        markers.P1_marker.XData = 10;
        markers.P1_marker.YData = 10;
        markers.P2_marker.XData = 10;
        markers.P2_marker.YData = 10;
        markers.bezier_curve.XData = 10;
        markers.bezier_curve.YData = 10;
        markers.target_marker.XData = 10;
        markers.target_marker.YData = 10;
    end
end
