%% Set up
syms ax ay az
syms mx my mz
syms Vmx Vmy Vmz
syms q [1 4]
%% Not taking into account cross product
% Measurements
% Accelereomter measurement
qAcc = [0 ax ay az];
% Earth gravity reference field
qGrav = [0 0 0 1];
% Error between accelerometer measurement and rotated earth field
Fa = qp(quaternConj(q),qp(qGrav,q)) - qAcc; 
Fa = simplify(Fa(2:4)).';

% Magnetometer measurement perpendicular to earth gravity field
qMag = [0 Vmx Vmy Vmz];
% Earth magnetic ref field (portion perpendicular to grav field)
qEarth = [0 0 1 0];
% Error between mag measurement and rotated mag earth field
Fm = qp(quaternConj(q),qp(qEarth,q)) - qMag; 
Fm = simplify(Fm(2:4)).';
% Measurement Model
% Measurement model as a function of orientation quaternion
Ja = simplify(jacobian(Fa,q)).';
Jm = simplify(jacobian(Fm,q)).';

Qdx = 1/2*[-q2 -q3 -q4
    q1 -q4 q3
    q3 q1 -q2
    -q3 q2 q1];

JTA = simplify(Ja.'*Qdx);
JTM = simplify(Jm.'*Qdx);
clear HTA HTM
for i = 1 : 3
HTA(:,:,i) = jacobian(JTA(:,i),q)*Qdx;
HTM(:,:,i) = jacobian(JTM(:,i),q)*Qdx;
end



% Measurement Hessians
clear Ha Hm
for i = 1 : 3
    Ha(:,:,i) = hessian(Fa(i),q);
    Hm(:,:,i) = hessian(Fm(i),q);
end
ff = [Fa;Fm];
for i = 1 : 6
    HH(:,:,i) = hessian(ff(i),q);
end

%% Take into account cross product
% Measurements
% Accelereomter measurement
qAcc = [0 ax ay az];
% Earth gravity reference field
qGrav = [0 0 0 1];
% Error between accelerometer measurement and rotated earth field
Fa = qp(quaternConj(q),qp(qGrav,q)) - qAcc; 
Fa = simplify(Fa(2:4)).';

% Magnetometer measurement perpendicular to earth gravity field
qMag = [0 cross([ax ay az],[mx my mz])];
% qMag = [0 Vmx Vmy Vmz];
% Earth magnetic ref field (portion perpendicular to grav field)
qEarth = [0 0 1 0];
% Error between mag measurement and rotated mag earth field
Fm = qp(quaternConj(q),qp(qEarth,q)) - qMag; 
Fm = simplify(Fm(2:4)).';
% Measurement Model
% Measurement model as a function of orientation quaternion
Ja = simplify(jacobian(Fa,q)).';
Jm = simplify(jacobian(Fm,q)).';

Qdx = 1/2*[-q2 -q3 -q4
    q1 -q4 q3
    q3 q1 -q2
    -q3 q2 q1];

JTA = simplify(Ja.'*Qdx);
JTM = simplify(Jm.'*Qdx);
clear HTA HTM
for i = 1 : 3
HTA(:,:,i) = jacobian(JTA(:,i),q)*Qdx;
HTM(:,:,i) = jacobian(JTM(:,i),q)*Qdx;
end



% Measurement Hessians
clear Ha Hm
for i = 1 : 3
    Ha(:,:,i) = hessian(Fa(i),q);
    Hm(:,:,i) = hessian(Fm(i),q);
end
ff = [Fa;Fm];
for i = 1 : 6
    HH(:,:,i) = hessian(ff(i),q);
end

