function [cost] = AHRScost2(z,AHRS,data,fs,opts,optimOpts)
% Set Parameters before estimating error in attitude estimation
ahrsClass = class(AHRS);
parmNames = optimOpts.(ahrsClass).parameters.Names;
for i = 1 : length(parmNames)
   AHRS.(parmNames{i}) = z(i); 
end

[~,cost] = estimateAttitude_WuDataset(AHRS,[],[],fs,data,opts);