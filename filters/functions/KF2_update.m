function [qPredicted,qCorrected,pPredicted] = KF2_update(q,gyro,acc,mag,gains,dt,P)
Sigma_g = reshape(gains(1:9),3,3);
Sigma_a = reshape(gains(10:18),3,3);
Sigma_m = reshape(gains(19:27),3,3);

%% Normalize
acc = acc / norm(acc);
mag = mag / norm(mag);

%%


q1 = q(1);q2 = q(2);q3 = q(3);q4 =q(4);
ax = acc(1); ay=acc(2); az= acc(3);



dqAcc = -[2*ay*q2 - 2*ax*q3 + q1*(2*az - 2)
2*ax*q4 + 2*ay*q1 - q2*(2*az + 2)
2*ay*q4 - 2*ax*q1 - q3*(2*az + 2)
2*ax*q2 + 2*ay*q3 + q4*(2*az - 2)];

% dqAcc = -[2*ay*q2 - 2*ax*q3 + q1*(2*az - 2)
% 2*ax*q4 + 2*ay*q1 - q2*(2*az + 2)
% 2*ay*q4 - 2*ax*q1 - q3*(2*az + 2)];

% Estimate of orientation from magnetometer. Orthogonal to
% accelerometer estimate
Vm = cross(acc, mag);
Vm = Vm / norm(Vm);

mx = Vm(1);my = Vm(2); mz = Vm(3);

dqMag = -[2*mx*q4 - 2*mz*q2 + q1*(2*my - 2)
2*mx*q3 - 2*mz*q1 - q2*(2*my + 2)
2*mx*q2 + 2*mz*q4 + q3*(2*my - 2)
2*mx*q1 + 2*mz*q3 - q4*(2*my + 2)];

% dqMag = -[2*mx*q4 - 2*mz*q2 + q1*(2*my - 2)
% %     2*mx*q2 + 2*mz*q4 + q3*(2*my - 2)
% 2*mx*q1 + 2*mz*q3 - q4*(2*my + 2)];

E = -[dqAcc;dqMag];
H = [eye(4);eye(4)];
% H([4 6 7],:) = [];
% R = blkdiag(Sigma_a,Sigma_m(1:2,1:2));

% E = -[dqAcc;dqMag];


[qPredicted,A] = stateFcn(q.',gyro,dt);


% E = E - H*qPredicted;

Q = stateCovariance(q,dt,Sigma_g);




J = [-2*q3, 2*q2,  2*q1,     0,     0,     0
 2*q4, 2*q1, -2*q2,     0,     0,     0
-2*q1, 2*q4, -2*q3,     0,     0,     0
 2*q2, 2*q3,  2*q4,     0,     0,     0
    0,    0,     0, -2*q4, -2*q1,  2*q2
    0,    0,     0, -2*q3,  2*q2,  2*q1
    0,    0,     0, -2*q2, -2*q3, -2*q4
    0,    0,     0, -2*q1,  2*q4, -2*q3];
R = J * blkdiag(Sigma_a,Sigma_a) * J.';
Ra = Sigma_a(1,1)^2 ./ R(1:4,1:4);
Rm = Sigma_m(1,1)^2 ./ R(5:8,5:8);
R = blkdiag(Ra,Rm);

P_ = A*P*A.' + Q;
S = H*P_*H.' + R;


% Kalman Gain
L = P_ * H.' / (S);
qEstimated = qPredicted + L*E;
qCorrected = (q.'+L*E);

% Normalize quaternion
qEstimated = qEstimated./norm(qEstimated);
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
