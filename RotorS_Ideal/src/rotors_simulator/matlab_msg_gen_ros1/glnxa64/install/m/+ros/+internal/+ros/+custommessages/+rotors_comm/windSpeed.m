function [data, info] = windSpeed
%WindSpeed gives an empty data for rotors_comm/WindSpeed
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rotors_comm/WindSpeed';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Velocity, info.Velocity] = ros.internal.ros.messages.geometry_msgs.vector3;
info.Velocity.MLdataType = 'struct';
info.MessageType = 'rotors_comm/WindSpeed';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'velocity';
info.MatPath{8} = 'velocity.x';
info.MatPath{9} = 'velocity.y';
info.MatPath{10} = 'velocity.z';
