function mocapCallback(msg, qcar)
% This function updates the pose of the robot via Optitrack feedback

    % % ==== Position Data in Meters ==== % %
    x_position =  msg.pose.position.x;
    y_position = -msg.pose.position.z;

    % % ==== Orientation data in Quaternion Form ==== % %
    q = msg.pose.orientation;
    % % ==== Changing from quaternion to Euler angles in radians ==== % %
    [yaw, pitch, roll] = quat2angle([q.w, q.x, q.y, q.z], "YZX"); 
    heading = yaw;

    % % ==== Robot's Pose Update ==== % %
    qcar.pose.x     = x_position;
    qcar.pose.y     = y_position;
    qcar.pose.theta = heading;

end
