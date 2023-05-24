function qPredicted = MadgwickAHRS3_update(q,gyro,acc,mag,gains,dt,normStepSize)
beta = gains(1);

%% Normalize
acc = acc / norm(acc);
mag = mag / norm(mag);
%% Measurement

% Reference direction of Earth's magnetic feild
h = quaternProd(q, quaternProd([0 mag], quaternConj(q)));
b = [0 norm([h(2) h(3)]) 0 h(4)];
%             Fa = quaternProd(quaternConj(q),quaternProd([0 0 0 1],q)) - [0 Accelerometer]
%             Fm = quaternProd(quaternConj(q),quaternProd(b,q)) - [0 Magnetometer]
% Gradient decent algorithm corrective step
% (this is q-1 * vr * q -vm) --> F that needs to be minimised
Fa = [2*(q(2)*q(4) - q(1)*q(3)) - acc(1)
    2*(q(1)*q(2) + q(3)*q(4)) - acc(2)
    2*(0.5 - q(2)^2 - q(3)^2) - acc(3)];

% Jacobian are the partial derivatives of F (below is not the
% actual partial derivatives)
Ja = [-2*q(3),                 	2*q(4),                    -2*q(1),                         2*q(2)
    2*q(2),                 	2*q(1),                    	2*q(4),                         2*q(3)
    0,                         -4*q(2),                    -4*q(3),                         0];
Fm = []; Jm = [];
if ~any(isnan(mag))
    Fm = [2*b(2)*(0.5 - q(3)^2 - q(4)^2) + 2*b(4)*(q(2)*q(4) - q(1)*q(3)) - mag(1)
        2*b(2)*(q(2)*q(3) - q(1)*q(4)) + 2*b(4)*(q(1)*q(2) + q(3)*q(4)) - mag(2)
        2*b(2)*(q(1)*q(3) + q(2)*q(4)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2) - mag(3)];

    Jm = [-2*b(4)*q(3),               2*b(4)*q(4),               -4*b(2)*q(3)-2*b(4)*q(1),       -4*b(2)*q(4)+2*b(4)*q(2)
        -2*b(2)*q(4)+2*b(4)*q(2),	2*b(2)*q(3)+2*b(4)*q(1),	2*b(2)*q(2)+2*b(4)*q(4),       -2*b(2)*q(1)+2*b(4)*q(3)
        2*b(2)*q(3),                2*b(2)*q(4)-4*b(4)*q(2),	2*b(2)*q(1)-4*b(4)*q(3),       2*b(2)*q(2)];

end
F = [Fa;Fm];
J = [Ja;Jm].';

%%

qPredicted = gdfPredict(J,F,q,gyro,beta,dt,normStepSize);
