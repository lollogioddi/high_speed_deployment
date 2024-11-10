function [data, info] = octomapResponse
%Octomap gives an empty data for rotors_comm/OctomapResponse
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rotors_comm/OctomapResponse';
[data.Map, info.Map] = ros.internal.ros.messages.octomap_msgs.octomap;
info.Map.MLdataType = 'struct';
[data.OriginLatitude, info.OriginLatitude] = ros.internal.ros.messages.ros.default_type('double',1);
[data.OriginLongitude, info.OriginLongitude] = ros.internal.ros.messages.ros.default_type('double',1);
[data.OriginAltitude, info.OriginAltitude] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'rotors_comm/OctomapResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'map';
info.MatPath{2} = 'map.header';
info.MatPath{3} = 'map.header.seq';
info.MatPath{4} = 'map.header.stamp';
info.MatPath{5} = 'map.header.stamp.sec';
info.MatPath{6} = 'map.header.stamp.nsec';
info.MatPath{7} = 'map.header.frame_id';
info.MatPath{8} = 'map.binary';
info.MatPath{9} = 'map.id';
info.MatPath{10} = 'map.resolution';
info.MatPath{11} = 'map.data';
info.MatPath{12} = 'origin_latitude';
info.MatPath{13} = 'origin_longitude';
info.MatPath{14} = 'origin_altitude';
