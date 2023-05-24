function [qPredicted,qCorrected] = ticcKFexp_update(q,gyro,acc,mag,gains,dt)
CA = gains(1);
LA = gains(2);
OA = gains(3);

CM = gains(4);
LM = gains(5);
OM = gains(6);

%% Normalize
acc = acc / norm(acc);
mag = mag / norm(mag);

%%


q1 = q(1);q2 = q(2);q3 = q(3);q4 =q(4);
ax = acc(1); ay=acc(2); az= acc(3);



dqAcc = [2*q1^3 + 2*q1*q2^2 + 2*q1*q3^2 + 2*q1*q4^2 - 2*az*q1 - 2*ay*q2 + 2*ax*q3
    2*q1^2*q2 - 2*ay*q1 + 2*q2^3 + 2*q2*q3^2 + 2*q2*q4^2 + 2*az*q2 - 2*ax*q4
    2*q1^2*q3 + 2*ax*q1 + 2*q2^2*q3 + 2*q3^3 + 2*q3*q4^2 + 2*az*q3 - 2*ay*q4
    2*q1^2*q4 + 2*q2^2*q4 - 2*ax*q2 + 2*q3^2*q4 - 2*ay*q3 + 2*q4^3 - 2*az*q4];

% Estimate of orientation from magnetometer. Orthogonal to
% accelerometer estimate
Vm = cross(acc, mag);
Vm = Vm / norm(Vm);

mx = Vm(1);my = Vm(2); mz = Vm(3);

dqMag =  [2*q1^3 + 2*q1*q2^2 + 2*q1*q3^2 + 2*q1*q4^2 - 2*my*q1 + 2*mz*q2 - 2*mx*q4
    2*q1^2*q2 + 2*mz*q1 + 2*q2^3 + 2*q2*q3^2 + 2*q2*q4^2 + 2*my*q2 - 2*mx*q3
    2*q1^2*q3 + 2*q2^2*q3 - 2*mx*q2 + 2*q3^3 + 2*q3*q4^2 - 2*my*q3 - 2*mz*q4
    2*q1^2*q4 - 2*mx*q1 + 2*q2^2*q4 + 2*q3^2*q4 - 2*mz*q3 + 2*q4^3 + 2*my*q4];


            accGain = CA * exp(-LA./dt) + OA;
            magGain = CM * exp(-LM./dt) + OM;

%             qCorrected = (q - obj.c_accelerometer * qa - obj.c_magnetometer * qm);
qCorrected = (q - accGain * dqAcc.' - magGain * dqMag.');

qDotMeasured = 0.5 * quaternProd(q,[0 gyro]);
qPredicted = qCorrected + dt* qDotMeasured;

qCorrected = qCorrected ./ norm(qCorrected);
qPredicted = qPredicted ./ norm(qPredicted);
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