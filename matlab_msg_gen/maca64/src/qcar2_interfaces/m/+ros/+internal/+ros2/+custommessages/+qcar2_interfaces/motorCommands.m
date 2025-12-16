function [data, info] = motorCommands
%MotorCommands gives an empty data for qcar2_interfaces/MotorCommands
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'qcar2_interfaces/MotorCommands';
[data.motor_names, info.motor_names] = ros.internal.ros2.messages.ros2.char('string',NaN,NaN,0);
[data.values, info.values] = ros.internal.ros2.messages.ros2.default_type('double',NaN,0);
info.MessageType = 'qcar2_interfaces/MotorCommands';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'motor_names';
info.MatPath{2} = 'values';
