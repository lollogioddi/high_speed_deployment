function [data, info] = performanceMetricsRequest
%PerformanceMetrics gives an empty data for rotors_comm/PerformanceMetricsRequest
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rotors_comm/PerformanceMetricsRequest';
info.MessageType = 'rotors_comm/PerformanceMetricsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
