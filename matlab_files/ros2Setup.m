function qcar = ros2Setup(carID)
% carID is a scalar denoting the ID number of the QCar to use.
%%%%%%%%%%%%%%%%%%%%%%%%% EXAMPLE %%%%%%%%%%%%%%%%%%%%%%%%
% qcar_01 = ros2Setup(1);
% qcar_02 = ros2Setup(2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This process is run for ROS_DOMAIN_ID = 123.

    setenv("ROS_DOMAIN_ID", "22")

    if ~isscalar(carID)
        error('carID must be a scalar');
    end

    id = carID;

    if ~ismember(id, [1 2 3])
        error('Invalid car ID: %d. Valid IDs are 1, 2, or 3.', id);
    end
        
    % % ==== Namespace ==== % %
    namespace = sprintf('/qcar_0%d', id);
    
    % % ==== ROS2 Node Creation ==== % %
    nodeName = sprintf('matlab_qcar0%d_node', id);
    node = ros2node(nodeName, 22);   % 123 = ROS_DOMAIN_ID
    
    % % ==== ROS2 Publisher Setup ==== % %
    topic2Pub   = namespace + "/qcar2_motor_speed_cmd";
    pubMsgType  = "qcar2_interfaces/MotorCommands";
    motorCommandsPub = ros2publisher(node, topic2Pub, pubMsgType);
    
    % % ==== Definition of our robot as a QCar object ==== % %
    qcar = QCar(id, namespace, node, motorCommandsPub);
    
    % % ==== ROS2 Subscriber Setup ==== % %
    topic2Subs  = "/vrpn_mocap" + namespace + "/pose";
    subMsgType  = "geometry_msgs/PoseStamped";
    
    % Aquí viene la magia: pasamos el HANDLE qcar al callback
    mocapSub = ros2subscriber( ...
        node, topic2Subs, subMsgType, ...
        @(msg)mocapCallback(msg, qcar), ...   % <--- importante
        "Reliability","besteffort", ...
        "Durability","volatile");
    
    % % ==== Declaring the subscriber in the QCar object ==== % %
    qcar.subscriber = mocapSub;
end
