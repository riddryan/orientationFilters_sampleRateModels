function [sse,q,p] = AHRSerror(AHRS,data,opts)
% Estimate Attitude of sensor
q0 = data.ground_truth(1,:);
q = estimateOrientation(AHRS,q0,data.gyro,data.acc,data.mag);

% Estimate cost (sse between estimated quaternion and mocap quaternion)
q_error = quaternProd(data.ground_truth,quaternConj(q));
euler_error = eulerd(quaternion(q_error),'ZYX','point');
sse = sum(rms(euler_error));

if nargout > 2
    ahrsClass = class(AHRS);
    parmNames = opts.(ahrsClass).parameters.Names;
    for i = 1 : length(parmNames)
        p(i) = AHRS.(parmNames{i});
    end
end