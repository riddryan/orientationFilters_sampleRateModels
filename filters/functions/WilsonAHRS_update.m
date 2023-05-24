function qPredicted = WilsonAHRS_update(q,gyro,acc,mag,gains,dt,normStepSize)
beta = gains(1);

%% Normalize
acc = acc / norm(acc);
mag = mag / norm(mag);
%% Measurement
% Estimate of orientation from accelerometer
Fa = [-2*q(1)*q(3) + 2*q(2)*q(4)                             - acc(1)
    2*q(1)*q(2)   + 2*q(3)*q(4)                             - acc(2)
    q(1)*q(1)     - q(2)*q(2) - q(3)*q(3) + q(4)*q(4)       - acc(3)];

Ja = 2* [ -q(3)  q(2)  q(1)
    q(4)  q(1) -q(2)
    -q(1)  q(4) -q(3)
    q(2)  q(3)  q(4)];


Fm = []; Jm = [];
if ~any(isnan(mag))
Vm = cross(acc, mag);
Vm = Vm / norm(Vm);

Fm =                 [2*q(1)*q(4) + 2*q(2)*q(3)                               - Vm(1)
    q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2                       - Vm(2)
    2*q(3)*q(4) - 2*q(1)*q(2)                               - Vm(3)];

Jm = 2 * [q(4)  q(1) -q(2)
    q(3) -q(2) -q(1)
    q(2)  q(3)  q(4)
    q(1) -q(4)  q(3)];
end
F = [Fa;Fm];
J = [Ja Jm];


%%

qPredicted = gdfPredict(J,F,q,gyro,beta,dt,normStepSize);
