


%% Acc
syms ax ay az
syms q [1 4]
qAcc = [0 ax ay az];
qGrav = [0 0 0 1];
fAcc = q + qp(quaternConj(q),qp(qGrav,q)) - qAcc;
% fAcc = fAcc(2:4);
% fAcc = qp(quaternConj(q),qp(qGrav,q)) - qAcc; fAcc = fAcc(2:4);
fAcc = simplify(fAcc.')
JAcc = simplify(jacobian(fAcc,q.'))

%% Mag
syms mx my mz
qMag = [0 mx my mz];
qEarth = [0 0 1 0];
fMag = q + qp(quaternConj(q),qp(qEarth,q)) - qMag;
fMag = simplify(fMag.')
JMag = simplify(jacobian(fMag,q.'))
% J = simplify(jacobian(f(2:4),q.'))

%% Measurement Jacobian
qMeas = [fAcc;fMag];
syms aw mw
measVars = [mw mx my mz aw ax ay az].';
Jmeas = simplify(jacobian(qMeas,measVars));

syms ra rm
sigMeas = blkdiag(diag([ra ra ra ra]),diag([rm rm rm rm]));

%% Acc - q frame

% f = q + qp(quaternConj(q),qp(qGrav,q)) - qAcc;
fAcc2 = q + qGrav - qp(q,qp(qAcc,quaternConj(q)))
fAcc2 = simplify(fAcc2.')

% J = jacobian(f(2:4),q.')
JAcc2 = simplify(jacobian(fAcc2,q.'))

%% Mag - q frame
% syms mN mD
% qEarth = [0 mN 0 mD];
fMag2 = q + qEarth - qp(q,qp(qMag,quaternConj(q)))
fMag2 = simplify(fMag2.')

% J = jacobian(f(2:4),q.')
JMag2 = simplify(jacobian(fMag2,q.'))

%% Measurement model - magnetometer
% Measurement

% Reference field is cross product of earth's magnetic field and gravity.
% Since gravity is in the z direction, and earth's magnetic field has components in
% the x and z direction, the magnetic reference direction is in the
% y-direction.
Mr = [0;1;0];
% Mr = [0;-1;0];

P2 = [q4 q3 q2 q1
    q1 -q2 q3 -q4
    -q2 q1 q4 q3];
Mb = [mx;my;mz];

qm = (Mr(2)*P2).'*Mb;
Hmag = equationsToMatrix(qm,q.');

%% Measurement model = accelerometer
Mr = [0;0;1];
% From Jin Wu Fast KF
P3 = [-q3 q4 -q1 q2
    q2 q1 q4 q3
    q1 -q2 -q3 q4];
Ma = [ax;ay;az];
qa = (Mr(3)*P3).'*Ma;
Hacc = equationsToMatrix(qa,q.');
% Hacc = [az ay -ax 0
%     ay -az 0 ax
%     -ax 0 -az ay
%     0 ax ay az];
