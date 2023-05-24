


%% Acc
syms ax ay az
syms q [1 4]
qAcc = [0 ax ay az];
qGrav = [0 0 0 1];
% fAcc = q + qp(quaternConj(q),qp(qGrav,q)) - qAcc;
% fAcc = fAcc(2:4);
fAcc = qp(quaternConj(q),qp(qGrav,q)) - qAcc; fAcc = fAcc(2:4);
fAcc = simplify(fAcc.')
JAcc = simplify(jacobian(fAcc,q.'))

dqAcc = simplify(JAcc.'*fAcc)


JAQ = simplify(jacobian(dqAcc,q))

%% Mag
syms mx my mz
qMag = [0 mx my mz];
qEarth = [0 0 1 0];
% fMag = q + qp(quaternConj(q),qp(qEarth,q)) - qMag;
fMag = qp(quaternConj(q),qp(qEarth,q)) - qMag; fMag = fMag(2:4);
fMag = simplify(fMag.')
JMag = simplify(jacobian(fMag,q.'))
% J = simplify(jacobian(f(2:4),q.'))
dqMag = simplify(JMag.'*fMag)

JMQ = simplify(jacobian(dqMag,q))

%% Measurement Jacobian
qMeas = [fAcc;fMag];
syms aw mw
measVars = [mw mx my mz aw ax ay az].';
Jmeas = simplify(jacobian(qMeas,measVars));

syms ra rm
sigMeas = blkdiag(diag([ra ra ra ra]),diag([rm rm rm rm]));

%% Acc - q frame

% f = q + qp(quaternConj(q),qp(qGrav,q)) - qAcc;
fAcc2 = qGrav - qp(q,qp(qAcc,quaternConj(q)))
fAcc2 = simplify(fAcc2.')

% J = jacobian(f(2:4),q.')
JAcc2 = simplify(jacobian(fAcc2,q.'))

%% Mag - q frame
% syms mN mD
% qEarth = [0 mN 0 mD];
fMag2 = qEarth - qp(q,qp(qMag,quaternConj(q)))
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
    -q2 -q1 q4 q3];
Mb = [mx;my;mz];

qm = (Mr(2)*P2).'*Mb;
Hmag = equationsToMatrix(qm,q.');

QM = 1/2 * (Hmag + eye(4)) * q.';
JJM = jacobian(QM,[mx;my;mz]);

dqm = simplify(2 * (Hmag + eye(4)) * q.');
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

dqa = simplify(2 * (eye(4) + Hacc) * q.')

%% Meas model - mag (crossed with acc)

vm = cross(Mb,Ma);

qm2 = (Mr(2)*P2).'*vm;
Hmag2 = equationsToMatrix(qm2,q.');

QM2 = 1/2 * (Hmag2 + eye(4)) * q.';
JJM2 = jacobian(QM2,[mx;my;mz]);

%% Simplified wilson expressions
syms q1 q2 q3 q4
syms ax ay az mx my mz
Ma = [ax;ay;az];
Mb = [mx;my;mz];
q = [q1;q2;q3;q4];
% assume(q,'real');
% assume(Ma,{'real' 'positive'});
% assume(Mb,{'real' 'positive'});

dqAcc = [2*ay*q2 - 2*ax*q3 + q1*(2*az - 2)
2*ax*q4 + 2*ay*q1 - q2*(2*az + 2)
2*ay*q4 - 2*ax*q1 - q3*(2*az + 2)
2*ax*q2 + 2*ay*q3 + q4*(2*az - 2)];

vm = cross(Mb,Ma);
vmN = sqrt((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2);
vm = vm ./ vmN;
% vm = vm ./ norm(vm);

% dqMag = [2*mx*q4 - 2*mz*q2 + q1*(2*my - 2)
% 2*mx*q3 - 2*mz*q1 - q2*(2*my + 2)
% 2*mx*q2 + 2*mz*q4 + q3*(2*my - 2)
% 2*mx*q1 + 2*mz*q3 - q4*(2*my + 2)];

dqMag = [2*vm(1)*q4 - 2*vm(3)*q2 + q1*(2*vm(2) - 2)
2*vm(1)*q3 - 2*vm(3)*q1 - q2*(2*vm(2) + 2)
2*vm(1)*q2 + 2*vm(3)*q4 + q3*(2*vm(2) - 2)
2*vm(1)*q1 + 2*vm(3)*q3 - q4*(2*vm(2) + 2)];

JA = simplify(jacobian(dqAcc,[ax;ay;az;mx;my;mz]));
JM = -simplify(jacobian(dqMag,[ax;ay;az;mx;my;mz]));
J = [JA;JM];

% JA = jacobian(dqAcc,[ax;ay;az]);
% JM = jacobian(dqMag,[mx;my;mz]);

syms ra rm

% R = blkdiag(diag([ra ra ra]),diag([rm rm rm]));
% RT = simplify(J*R*J.');


%% Simplified wilson expressions
syms q1 q2 q3 q4
syms ax ay az mx my mz
Ma = [ax;ay;az];
Mb = [mx;my;mz];
q = [q1;q2;q3;q4];
% assume(q,'real');
% assume(Ma,{'real' 'positive'});
% assume(Mb,{'real' 'positive'});

dqAcc = [2*ay*q2 - 2*ax*q3 + q1*(2*az - 2)
2*ax*q4 + 2*ay*q1 - q2*(2*az + 2)
2*ay*q4 - 2*ax*q1 - q3*(2*az + 2)
2*ax*q2 + 2*ay*q3 + q4*(2*az - 2)];

vm = Mb;

% dqMag = [2*mx*q4 - 2*mz*q2 + q1*(2*my - 2)
% 2*mx*q3 - 2*mz*q1 - q2*(2*my + 2)
% 2*mx*q2 + 2*mz*q4 + q3*(2*my - 2)
% 2*mx*q1 + 2*mz*q3 - q4*(2*my + 2)];

dqMag = [2*vm(1)*q4 - 2*vm(3)*q2 + q1*(2*vm(2) - 2)
2*vm(1)*q3 - 2*vm(3)*q1 - q2*(2*vm(2) + 2)
2*vm(1)*q2 + 2*vm(3)*q4 + q3*(2*vm(2) - 2)
2*vm(1)*q1 + 2*vm(3)*q3 - q4*(2*vm(2) + 2)];

JA = simplify(jacobian(dqAcc,[ax;ay;az;mx;my;mz]));
JM = -simplify(jacobian(dqMag,[ax;ay;az;mx;my;mz]));
J = [JA;JM];

% JA = jacobian(dqAcc,[ax;ay;az]);
% JM = jacobian(dqMag,[mx;my;mz]);

syms ra rm

% R = blkdiag(diag([ra ra ra]),diag([rm rm rm]));
% RT = simplify(J*R*J.');

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
    -q2 -q1 q4 q3];
Mb = [mx;my;mz];

qm = (Mr(2)*P2).'*Mb;
Hmag = equationsToMatrix(qm,q.');

QM = 1/2 * (Hmag + eye(4)) * q.';
JJM = jacobian(QM,[mx;my;mz]);

dqm = simplify(2 * (Hmag + eye(4)) * q.');