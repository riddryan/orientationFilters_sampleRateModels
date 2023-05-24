function [qPredicted,qCorrected,dqdtN,qa,qm] = RiddickAHRS_update_higherOrder(q,gyro,acc,mag,gains,dt,normStepSize,intOrder,dqdtPrev) %#codegen
% dqdtPrev are the time derivatives of q based on gyroscope measurements
% Where the ith represents the ith derivative of q at the previous time
% point
% dqdtPrev is Nx4, where N must be greater than or equal to intOrder
% size 4 because it is in quaternion form

if nargin < 8
    intOrder = 1;
end
if nargin < 9
    dqdtPrev = [];
end
c_gyro = gains(1);
c_acc = gains(2);
c_mag = gains(3);

if length(gains) < 4
    c_gyro2 = c_gyro;
else
    c_gyro2 = gains(4);
end

if length(gains) < 5
    c_gyro3 = c_gyro;
else
    c_gyro3 = gains(5);
end

dqdtN = zeros(intOrder,4);

qa = NaN(1,4);
qm = NaN(1,4);

%% Normalize
acc = acc / norm(acc);
mag = mag / norm(mag);
%% Measurement - acceleration
% Estimate of orientation from accelerometer

% faEx = [-2*q(1)*q(3) + 2*q(2)*q(4)
%     2*q(1)*q(2)   + 2*q(3)*q(4)
%     q(1)*q(1)     - q(2)*q(2) - q(3)*q(3) + q(4)*q(4)];

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

% if norm(qa-q) > norm(qa+q)
%     qa = - qa;
% end
% if norm(qa(1)-q(1)) > norm(qa(1)+q(1))
%     qa = - qa;
% end

qCorrected = (q - c_acc * qa * dt);
%% Measurement - magnetometer

% % Flexed
% % q = [0.7071 0 -0.7071 0];
% %  AA = [-1 0 0];
% %  MM = [-0.2 0 -0.1];
% 
% q = [0.7071 0 -0.7071 0];
%  AA = [-0.8 0 -0.07];
%  MM = [-0.26 0 -0.123];
% 
% % standing straight
% % q = [1 0 0 0];
% %  AA = [0 0 1];
% %  MM = [1 0 1];
% 
% 
%  MM = MM/norm(MM);
%  VM = cross(AA,MM);
%  VM = VM/norm(VM);
% 
%  Fm =                 [2*q(1)*q(4) + 2*q(2)*q(3)                               - VM(1)
%         q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2                       - VM(2)
%         2*q(3)*q(4) - 2*q(1)*q(2)                               - VM(3)];
%      Jm = 2 * [q(4)  q(1) -q(2)
%         q(3) -q(2) -q(1)
%         q(2)  q(3)  q(4)
%         q(1) -q(4)  q(3)];

if ~any(isnan(mag))
    % Estimate of orientation from magnetometer. Orthogonal to
    % accelerometer estimate
%     Vm  = mag;
    Vm = cross(acc, mag);
    Vm = Vm / norm(Vm);

    Fm =                 [2*q(1)*q(4) + 2*q(2)*q(3)                               - Vm(1)
        q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2                       - Vm(2)
        2*q(3)*q(4) - 2*q(1)*q(2)                               - Vm(3)];
    Jm = 2 * [q(4)  q(1) -q(2)
        q(3) -q(2) -q(1)
        q(2)  q(3)  q(4)
        q(1) -q(4)  q(3)];

% FLIPPED

%     Fm =                 [-2*q(1)*q(4) - 2*q(2)*q(3)                               - Vm(1)
%         -q(1)^2 + q(2)^2 - q(3)^2 + q(4)^2                       - Vm(2)
%         2*q(3)*q(4) - 2*q(1)*q(2)                               - Vm(3)];
% 
%     Jm = 2 * [-q(4)  -q(1) q(2)
%         -q(3) q(2) q(1)
%         -q(2)  -q(3)  -q(4)
%         -q(1) q(4)  -q(3)];

    qm = (Jm*Fm).';

    if normStepSize
        qm = qm ./ norm(qm);
    end

    if norm(qm-q) > norm(qm+q)
        qm = - qm;
    end

    qCorrected = qCorrected - c_mag * qm * dt;
end



%% Predict
qGyro = [0 gyro];
dqdt = 0.5 * quaternProd(q,qGyro);

% First order
predictedChange = c_gyro * dqdt * dt;

% Store 1st order derivative for output
dqdtN(1,:) = dqdt;

% Second order term
dqdt2 = zeros(1,4);
keepGoing = true;
if intOrder >= 2
    dqdt2 = (dqdt - dqdtPrev(1,:))/dt;
    if sum(dqdtPrev(1,:))==0
        keepGoing = false;
    else
        predictedChange = predictedChange + c_gyro2 * 1/2*dqdt2*dt^2;
    end
    dqdtN(2,:) = dqdt2;
end

% Third order term
dqdt3 = zeros(1,4);
if intOrder == 3 && keepGoing
    dqdt3 = (dqdt2 - dqdtPrev(2,:))/dt;
    if sum(dqdtPrev(2,:))==0
        keepGoing = false;
    else
        predictedChange = predictedChange + c_gyro3 * 1/6*dqdt3*dt^3;
    end
    dqdtN(3,:) = dqdt3;
end

qPredicted = qCorrected + predictedChange;

%% Normalize output
if nargout > 1
    qCorrected = qCorrected ./ norm(qCorrected);
end
qPredicted = qPredicted ./ norm(qPredicted);

% if norm(qPredicted-q) > norm(qPredicted+q)
%     qPredicted = - qPredicted;
% end
% if norm(qCorrected-q) > norm(qCorrected+q)
%     qCorrected = - qCorrected;
% end