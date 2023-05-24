function [cost,AHRS,q,p] = AHRScost(z,AHRS,data,opts)
% Set Parameters before estimating error in attitude estimation
ahrsClass = class(AHRS);
parmNames = opts.(ahrsClass).parameters.Names;
for i = 1 : length(parmNames)
   AHRS.(parmNames{i}) = z(i); 
end

[cost,q,p] = AHRSerror(AHRS,data,opts);