function T = outlierTable
%% Outliers
% Remove certain sensors from analysis (from clipping or other recording
% issues). These always come in a subject/sensor pair and specify which
% conditions.

badSu = [8;7];
badSe = {'mdm64E1';'mdm652E'};
badCo = {{'all'};{'og_jog_sss'}};
T = table(badSu,badSe,badCo,'VariableNames',{'Subject','Sensor','Condition'});