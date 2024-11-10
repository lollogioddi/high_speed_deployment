function [data, info] = performanceMetricsResponse
%PerformanceMetrics gives an empty data for rotors_comm/PerformanceMetricsResponse
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rotors_comm/PerformanceMetricsResponse';
[data.Overshoot1, info.Overshoot1] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Overshoot2, info.Overshoot2] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Overshoot3, info.Overshoot3] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Overshoot4, info.Overshoot4] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Overshoot5, info.Overshoot5] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Overshoot6, info.Overshoot6] = ros.internal.ros.messages.ros.default_type('double',1);
[data.SettlingTime1, info.SettlingTime1] = ros.internal.ros.messages.ros.default_type('double',1);
[data.SettlingTime2, info.SettlingTime2] = ros.internal.ros.messages.ros.default_type('double',1);
[data.SettlingTime3, info.SettlingTime3] = ros.internal.ros.messages.ros.default_type('double',1);
[data.SettlingTime4, info.SettlingTime4] = ros.internal.ros.messages.ros.default_type('double',1);
[data.SettlingTime5, info.SettlingTime5] = ros.internal.ros.messages.ros.default_type('double',1);
[data.SettlingTime6, info.SettlingTime6] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'rotors_comm/PerformanceMetricsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'overshoot1';
info.MatPath{2} = 'overshoot2';
info.MatPath{3} = 'overshoot3';
info.MatPath{4} = 'overshoot4';
info.MatPath{5} = 'overshoot5';
info.MatPath{6} = 'overshoot6';
info.MatPath{7} = 'settling_time1';
info.MatPath{8} = 'settling_time2';
info.MatPath{9} = 'settling_time3';
info.MatPath{10} = 'settling_time4';
info.MatPath{11} = 'settling_time5';
info.MatPath{12} = 'settling_time6';
