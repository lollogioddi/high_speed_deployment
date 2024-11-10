function [data, info] = octomapRequest
%Octomap gives an empty data for rotors_comm/OctomapRequest
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rotors_comm/OctomapRequest';
[data.BoundingBoxOrigin, info.BoundingBoxOrigin] = ros.internal.ros.messages.geometry_msgs.point;
info.BoundingBoxOrigin.MLdataType = 'struct';
[data.BoundingBoxLengths, info.BoundingBoxLengths] = ros.internal.ros.messages.geometry_msgs.point;
info.BoundingBoxLengths.MLdataType = 'struct';
[data.LeafSize, info.LeafSize] = ros.internal.ros.messages.ros.default_type('double',1);
[data.PublishOctomap, info.PublishOctomap] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Filename, info.Filename] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'rotors_comm/OctomapRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'bounding_box_origin';
info.MatPath{2} = 'bounding_box_origin.x';
info.MatPath{3} = 'bounding_box_origin.y';
info.MatPath{4} = 'bounding_box_origin.z';
info.MatPath{5} = 'bounding_box_lengths';
info.MatPath{6} = 'bounding_box_lengths.x';
info.MatPath{7} = 'bounding_box_lengths.y';
info.MatPath{8} = 'bounding_box_lengths.z';
info.MatPath{9} = 'leaf_size';
info.MatPath{10} = 'publish_octomap';
info.MatPath{11} = 'filename';
