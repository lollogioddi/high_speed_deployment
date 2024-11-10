function [data, info] = reset_simulationRequest
%reset_simulation gives an empty data for rotors_comm/reset_simulationRequest
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rotors_comm/reset_simulationRequest';
info.MessageType = 'rotors_comm/reset_simulationRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
