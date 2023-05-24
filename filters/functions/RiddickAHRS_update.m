function [qPredicted,qCorrected] = RiddickAHRS_update(q,gyro,acc,mag,gains,dt,normStepSize) %#codegen
c_gyro = gains(1);
c_acc = gains(2);
c_mag = gains(3);


%% Normalize
acc = acc / norm(acc);

% This is redundant since we later de-norm Vm, remove this line to speed up
% execution time. It is kept in here to make comparison to other GDF
% filters more similar in terms of execution time.
mag = mag / norm(mag); 
%% Measurement - acceleration
% Estimate of orientation from accelerometer
Fa = [-2*q(1)*q(3) + 2*q(2)*q(4)                             - acc(1)
    2*q(1)*q(2)   + 2*q(3)*q(4)                             - acc(2)
    q(1)*q(1)     - q(2)*q(2) - q(3)*q(3) + q(4)*q(4)       - acc(3)];

Ja = 2* [ -q(3)  q(2)  q(1)
    q(4)  q(1) -q(2)
    -q(1)  q(4) -q(3)
    q(2)  q(3)  q(4)];

qa = (Ja*Fa).';

if normStepSize
    qa = qa ./ norm(qa);
end

qCorrected = (q - c_acc * qa * dt);
%% Measurement - magnetometer


Fm = []; Jm = [];
if ~any(isnan(mag))
    % Estimate of orientation from magnetometer. Orthogonal to
    % accelerometer estimate
    Vm = cross(acc, mag);
    Vm = Vm / norm(Vm);

    Fm =                 [2*q(1)*q(4) + 2*q(2)*q(3)                               - Vm(1)
        q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2                       - Vm(2)
        2*q(3)*q(4) - 2*q(1)*q(2)                               - Vm(3)];

    Jm = 2 * [q(4)  q(1) -q(2)
        q(3) -q(2) -q(1)
        q(2)  q(3)  q(4)
        q(1) -q(4)  q(3)];
    qm = (Jm*Fm).';

    if normStepSize
        qm = qm ./ norm(qm);
    end

    qCorrected = (q - c_mag * qm * dt);
end

% qCorrected = q - dt * (c_acc*Ja*Fa - c_mag*Jm*Fm).';
%% Predict
qGyro = [0 gyro];
qDotMeasured = 0.5 * quaternProd(q,qGyro);
qPredicted = qCorrected + c_gyro * dt* qDotMeasured;

if nargout > 1
    qCorrected = qCorrected ./ norm(qCorrected);
end

qPredicted = qPredicted ./ norm(qPredicted);
qPredicted = qPredicted(1,:);