function [qPredicted,qCorrected,pPredicted] = esKF_update(q,gyro,acc,mag,gains,dt,P)
Sigma_g = reshape(gains(1:9),3,3);
Sigma_a = reshape(gains(10:18),3,3);
Sigma_m = reshape(gains(19:27),3,3);

%% Set up
acc = acc / norm(acc);
mag = mag / norm(mag);

%% Prediction


q0 = q;
% Nominal State
[qPredicted] = stateFcn(q.',gyro,dt);

% Error-state
qEw = vec2quat(gyro.',dt).';
Rw = q2R(qEw);

% Q = stateCovariance(q,dt,Sigma_g);
Q = Sigma_g;
P_ = Rw*P*Rw.' + Q*dt^2;

% IMPORTANT PREVIOUS VERSIONS OF THIS MEASUREMENT MODEL DIDN'T USE THE
% PREDICTED Q (INTEGRATED GYRO) NEED TO UPDATE
% q = qPredicted;

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


Qdx = 1/2*[-q(2) -q(3) -q(4)
    q(1) -q(4) q(3)
    q(3) q(1) -q(2)
    -q(3) q(2) q(1)];


E = -[Fa;Fm];
H = [Ja.';Jm.'];
H = H*Qdx;
R = blkdiag(Sigma_a,Sigma_m);
S = H*P_*H.' + R;
%% Measurement Model - 2nd order terms
Ha = zeros(4,4,3);
Hm = zeros(4,4,3);


% Accelerometer
Ha(:,:,1) = [ 0, 0, -2, 0
 0, 0,  0, 2
-2, 0,  0, 0
 0, 2,  0, 0];
Ha(:,:,2) = [0, 2, 0, 0
2, 0, 0, 0
0, 0, 0, 2
0, 0, 2, 0];
Ha(:,:,3) = [2,  0,  0, 0
0, -2,  0, 0
0,  0, -2, 0
0,  0,  0, 2];
% Magnetometer
Hm(:,:,1) = [0, 0, 0, 2
0, 0, 2, 0
0, 2, 0, 0
2, 0, 0, 0];
Hm(:,:,2) = [2,  0, 0,  0
0, -2, 0,  0
0,  0, 2,  0
0,  0, 0, -2];
Hm(:,:,3) = [ 0, -2, 0, 0
-2,  0, 0, 0
 0,  0, 0, 2
 0,  0, 2, 0];


hess = cat(3,Ha,Hm);

S2 = zeros(6);
E2 = zeros(6,1);
for i = 1 : 6
    for j = 1 : 6
        if j >= i
            S2(i,j) = 1/2 * trace(P_*Qdx.'*hess(:,:,i)*Qdx*P_*Qdx.'*hess(:,:,j)*Qdx);
        else
            S2(i,j) = S2(j,i);
        end
    end

    E2(i) = 1/2*trace(Qdx.'*hess(:,:,i)*Qdx*P_);
end
S = S + S2;
E = E + E2;
%% Filter
% Kalman Gain
L = P_ * H.' / (S);

dx = L*E;
qE = [1;0.5.*dx];
qE = qE ./ norm(qE);

qEstimated = quatMult(qPredicted,qE);
qCorrected = quatMult(q0.',qE);


PEstimated = (eye(3) - L*H)*P_;


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
