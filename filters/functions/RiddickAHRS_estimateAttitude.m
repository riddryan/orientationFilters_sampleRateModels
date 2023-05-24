function [q,sse] = RiddickAHRS_estimateAttitude(AHRS,p,parmNames,fs,data,opts)
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


        gains = [AHRS.c_gyroscope AHRS.c_accelerometer AHRS.c_magnetometer];
        dt = AHRS.SamplePeriod;
        normStepSize = AHRS.normStepSize;
        q = estimateOrientation_RiddickAHRS(q0,data.gyro,data.acc,data.mag,gains,dt,normStepSize);


if nargout > 1
    % Estimate cost (sse between estimated quaternion and mocap quaternion)
    q_error = quaternProd(data.ground_truth,quaternConj(q));
    euler_error = eulerd(quaternion(q_error),'ZYX','point');
%     bads = bads | isnan(euler_error(:,1));
    euler_error(bads,:) = [];
    sse = sum(rms(euler_error));

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
            
