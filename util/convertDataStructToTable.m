function [T,estTime] = convertDataStructToTable(O)

TB = struct2table(O);

parms = [];
for i = 1 : length(TB.iParameters)
    pp = num2cell(TB.iParameters{i},2);
    parms = [parms;pp];
end

T = table(cell2mat(TB.iCosts),parms,cell2mat(TB.subject),vertcat(TB.condition{:}),vertcat(TB.sensor{:}),vertcat(TB.location{:}),vertcat(TB.samplingFrequency{:}),vertcat(TB.algorithm{:}));
T.Properties.VariableNames = {'Cost' 'Parameters' 'Subject' 'Condition' 'Sensor' 'SensorLocation' 'SamplingFrequency' 'Algorithm'};

estTime = [];
if any(strcmp(TB.Properties.VariableNames,'estTime'))
    estTime = vertcat(TB.estTime{:});
end