function [q,sse,rmse,euler_error] = estimateAttitude_WuDataset(AHRS,p,parmNames,fs,data,opts)
% Estimate attitude of sensor given certain parameters for the AHRS filter
% p.
%
% D must have the field SampleRate and
%
% D has a substructure "Data" which has
% ground_truth quaternion orientation estimates from motion capture gyro
% (gyroscope measurements), mag (magnetometer measurements, and acc
% (accelerometer measurements)
%
% opts is a structure specifying various options (see defaultDataOpts.m)

% if isfield(data,'bads')
%     bads = data.bads;
% else
%     bads = false(size(data.time));
% end
bads = data.bads;

interpCost = opts.interpError;

% Set Parameters
if ~isempty(p) && ~isempty(parmNames)
    for i = 1 : length(parmNames)
        AHRS.(parmNames{i}) = p(i);
    end
end

% Set Sampling Rate
AHRS.SamplePeriod = 1 / fs;

% whether to use magnetometer
AHRS.UseMagnetometer = opts.useMagnetometer;

if isprop(AHRS,'normStepSize')
    AHRS.normStepSize = opts.GradDescent.normStepSize;
end

if isfield(data,'q0')
    q0 = data.q0;
else
    q0 = data.ground_truth(1,:);
end
% q0 = [1 0 0 0];
% Estimate orientation

oclass = class(AHRS);

if isfield(opts,'correctAHRSindices') && opts.correctAHRSindices
    indexOffset = 0;
else
    indexOffset = -1;
end

dt = AHRS.SamplePeriod;
if isprop(AHRS,'normStepSize')
    normStepSize = AHRS.normStepSize;
else
    normStepSize = false;
end

intOrder = opts.integrationOrder;


gains = allParams(AHRS);
if AHRS.UseMagnetometer
    mag = data.mag;
else
    mag = NaN(size(data.mag));
end

if any(strcmp(oclass,{'RiddickAHRS' 'WilsonAHRS' 'AdmiraalAHRS' 'MadgwickAHRS3' 'JustaAHRS' 'FastKF'}))

    if strcmp(oclass,'FastKF')
        P = AHRS.P;
    else
        P = [];
    end
    %     q = estimateOrientation_Fast2(oclass,q0,data.gyro,data.acc,mag,gains,dt,normStepSize);
    %     %         q = estimateOrientation_Fast2_mex(oclass,q0,data.gyro,data.acc,data.mag,gains,dt,normStepSize,P);

        q = estimateOrientation_Fast2(oclass,q0,data.gyro,data.acc,mag,gains,dt,normStepSize,P,intOrder);
%     q = estimateOrientation_Fast2_mex(oclass,q0,data.gyro,data.acc,mag,gains,dt,normStepSize,P,intOrder);
else
    
    q = estimateOrientation(oclass,q0,data.gyro,data.acc,mag,gains,dt,normStepSize,indexOffset);
end

% q(:,1) = abs(q(:,1));

if nargout > 1

    if ~interpCost
        % Estimate cost (sse between estimated quaternion and mocap quaternion)
%         q_error = quaternProd(data.ground_truth,quaternConj(q));
%         euler_error = eulerd(quaternion(q_error),'ZYX','point');
%         euler_error(bads,:) = [];
%         sse = sum(rms(euler_error));

        [euler_error,rmse,sse] = calcFilterError2(q,data.ground_truth,bads);
%           [euler_error,rmse,sse] = calcFilterError_mex(q,data.ground_truth,bads);
    else

%         qI = interp1(data.time,q,data.timeOrig);
%         q_error = quaternProd(data.groundTruthOrig,quaternConj(qI));
%         euler_error = eulerd(quaternion(q_error),'ZYX','point');
%         bads = isnan(euler_error(:,1));
%         euler_error(bads,:) = [];
%         sse = sum(rms(euler_error));

        bads = isnan(data.groundTruthOrig(:,1));
%         [euler_error,rmse,sse] = calcFilterError(qI,data.groundTruthOrig,bads);
        [euler_error,rmse,sse] = calcFilterError_mex(qI,data.groundTruthOrig,bads);
    end

end

