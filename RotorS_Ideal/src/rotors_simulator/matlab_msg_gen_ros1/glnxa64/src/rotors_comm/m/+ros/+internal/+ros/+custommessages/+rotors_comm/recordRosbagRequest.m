function [data, info] = recordRosbagRequest
%RecordRosbag gives an empty data for rotors_comm/RecordRosbagRequest
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rotors_comm/RecordRosbagRequest';
[data.Record, info.Record] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'rotors_comm/RecordRosbagRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'record';
