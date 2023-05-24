function [q,sse] = estimateAttitude(AHRS,p,parmNames,fs,data,opts)
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

interpCost = false;
bads = data.bads;

% Set Parameters
for i = 1 : length(parmNames)
   AHRS.(parmNames{i}) = p(i); 
end

% Set Sampling Rate
AHRS.SamplePeriod = 1 / fs;

% whether to use magnetometer
AHRS.UseMagnetometer = opts.useMagnetometer;

if isprop(AHRS,'normStepSize')
    AHRS.normStepSize = opts.GradDescent.normStepSize;
end


q0 = data.ground_truth(1,:);
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

    
    if any(strcmp(oclass,{'RiddickAHRS' 'WilsonAHRS' 'AdmiraalAHRS' 'MadgwickAHRS3' 'JustaAHRS' 'FastKF'}))
        % Here we have fast versions of the orientation estimation compiled
        % to mex files
        gains = allParams(AHRS);
        if AHRS.UseMagnetometer
            mag = data.mag;
        else
            mag = NaN(size(data.mag));
        end
        if strcmp(oclass,'FastKF')
            P = AHRS.P;
        else
            P = [];
        end
%         q = estimateOrientation_Fast(oclass,q0,data.gyro,data.acc,mag,gains,dt,normStepSize,indexOffset,P);
        q = estimateOrientation_Fast_mex(oclass,q0,data.gyro,data.acc,data.mag,gains,dt,normStepSize,indexOffset,P);
    else
        % Otherwise use the matlab code which call the class objects a lot
        % which slows down everything quite a bit
        q = estimateOrientation(oclass,q0,data.gyro,data.acc,mag,gains,dt,normStepSize,indexOffset);
    end

if nargout > 1
    if ~interpCost
        % Estimate cost (sse between estimated quaternion and mocap quaternion)
        q_error = quaternProd(data.ground_truth,quaternConj(q));
        euler_error = eulerd(quaternion(q_error),'ZYX','point');
        euler_error(bads,:) = [];
        sse = sum(rms(euler_error));
    else

        qI = interp1(data.time,q,data.timeOrig);
        q_error = quaternProd(data.groundTruthOrig,quaternConj(qI));
        euler_error = eulerd(quaternion(q_error),'ZYX','point');
        bads = isnan(euler_error(:,1));
        euler_error(bads,:) = [];
        sse = sum(rms(euler_error));
    end

%     eaTruth = eulerd(quaternion(data.ground_truth),'ZYX','point');
%     eaEst = eulerd(quaternion(q),'ZYX','point');

%     [Blow,Alow] = butter(2, 0.1 / (fs/2),'low');
%     [Bhigh,Ahigh] = butter(2, 0.1 / (fs/2),'high');
%     [Blow,Alow] = fir1(100, 0.1 / (fs/2),'low');
%     [Bhigh,Ahigh] = fir1(100, 0.1 / (fs/2),'high');

%     eaTruthLow = filtfilt(Blow,Alow,eaTruth);
%     eaTruthHigh = filtfilt(Bhigh,Ahigh,eaTruth);

%         eaEstLow = filtfilt(Blow,Alow,eaEst);
%     eaEstHigh = filtfilt(Bhigh,Ahigh,eaEst);

%     figure
%     eaFused = eaTruthLow+eaTruthHigh;
% %     eaFused = eaFused - eaFused(1,:) + eaTruth(1,:);
%     plot(eaTruth,'k'); hold on; plot(eaFused,'r')
% 
%     figure
%     subplot(211)
%     plot(eaTruthLow,'k'); hold on; plot(eaEstLow,'r')
%     subplot(212)
%     plot(eaTruthHigh,'k'); hold on; plot(eaEstHigh,'r')

%     sse = sum(rms(eaTruthHigh-eaEstHigh));
% sse = sum(rms(eaTruthLow-eaEstLow));
end
            
