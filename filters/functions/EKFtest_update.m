function [qPredicted,qCorrected,pPredicted,R,J,L,E] = EKFtest_update(q,gyro,acc,mag,Sigma_g,Sigma_a,Sigma_m,dt,P)
%% Set up
acc = acc / norm(acc);
mag = mag / norm(mag);


R = NaN(8,8);
J = NaN(8,6);
L = NaN(4,8);

% R = NaN(6,6);
% J = NaN(6,6);
% L = NaN(4,6);


%% Prediction


q0 = q;
[qPredicted,A] = stateFcn(q.',gyro,dt);
Q = stateCovariance(q,dt,Sigma_g);
P_ = A*P*A.' + Q;

% IMPORTANT PREVIOUS VERSIONS OF THIS MEASUREMENT MODEL DIDN'T USE THE
% PREDICTED Q (INTEGRATED GYRO) NEED TO UPDATE
q = qPredicted;

%% Measurement


Fa = [-2*q(1)*q(3) + 2*q(2)*q(4)                             - acc(1)
    2*q(1)*q(2)   + 2*q(3)*q(4)                             - acc(2)
    q(1)*q(1)     - q(2)*q(2) - q(3)*q(3) + q(4)*q(4)       - acc(3)];
Ja = 2* [ -q(3)  q(2)  q(1)
    q(4)  q(1) -q(2)
    -q(1)  q(4) -q(3)
    q(2)  q(3)  q(4)];




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


E = -[Fa;Fm];
H = [Ja.';Jm.'];
R = blkdiag(Sigma_a,Sigma_m);
S = H*P_*H.' + R;
% H = H*Qdx;
% S = H



%% Filter
% Kalman Gain
L = P_ * H.' / (S);

qEstimated = qPredicted + L*E;
qCorrected = (q0.'+L*E);
% Normalize quaternions
qEstimated = qEstimated./norm(qEstimated);
qCorrected = qCorrected ./ norm(qCorrected);

% qE = L*E;
% qE = qE ./ norm(qE);
% qEstimated = quatMult(qPredicted,qE);
% qCorrected = quatMult(q0.',qE);




% Correct state covariance
PEstimated = (eye(4) - L*H)*P_;


pPredicted = PEstimated;

qPredicted = qEstimated.';
% Store current estimate of orientation for next iteration
end


function [qPredicted,A] = stateFcn(q,gyro,dt)

% Linear state dynamics (A). The state is the integrated
% (across dt) gyroscope measurements. Returns predicted
% state at next time step (qPredicted) based on the state
% dynamics (A), which are linear dynamics in which the
% gyroscope measurement is integrated across the time step.
wx = gyro(1);
wy = gyro(2);
wz = gyro(3);

omega4=[0,-wx,-wy,-wz;
    wx,0,wz,-wy;
    wy,-wz,0,wx;
    wz,wy,-wx,0];
A = eye(4)+dt/2*omega4;


qPredicted = A*q;
end
function Q = stateCovariance(q,dt,Sigma_g)
qk_1=q.';
q0=qk_1(1);
q1=qk_1(2);
q2=qk_1(3);
q3=qk_1(4);
Dk=[q1 q2 q3;
    -q0 -q3 -q2;
    q2 -q0 -q1;
    -q2 q1 -q0];
Q = dt*dt/4*(Dk*Sigma_g*Dk.');
end
