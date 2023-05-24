function [qPredicted,qCorrected,pPredicted,R,J,L,E] = KFtest_update(q,gyro,acc,mag,Sigma_g,Sigma_a,Sigma_m,dt,P)
%% Normalize
acc = acc / norm(acc);
mag = mag / norm(mag);


R = NaN(8,8);
J = NaN(8,6);
L = NaN(4,8);

% R = NaN(6,6);
% J = NaN(6,6);
% L = NaN(4,6);


%%


q1 = q(1);q2 = q(2);q3 = q(3);q4 =q(4);
ax = acc(1); ay=acc(2); az= acc(3);



% dqAcc = [2*q1^3 + 2*q1*q2^2 + 2*q1*q3^2 + 2*q1*q4^2 - 2*az*q1 - 2*ay*q2 + 2*ax*q3
%     2*q1^2*q2 - 2*ay*q1 + 2*q2^3 + 2*q2*q3^2 + 2*q2*q4^2 + 2*az*q2 - 2*ax*q4
%     2*q1^2*q3 + 2*ax*q1 + 2*q2^2*q3 + 2*q3^3 + 2*q3*q4^2 + 2*az*q3 - 2*ay*q4
%     2*q1^2*q4 + 2*q2^2*q4 - 2*ax*q2 + 2*q3^2*q4 - 2*ay*q3 + 2*q4^3 - 2*az*q4];

dqAcc = -[2*ay*q2 - 2*ax*q3 + q1*(2*az - 2)
2*ax*q4 + 2*ay*q1 - q2*(2*az + 2)
2*ay*q4 - 2*ax*q1 - q3*(2*az + 2)
2*ax*q2 + 2*ay*q3 + q4*(2*az - 2)];

% Estimate of orientation from magnetometer. Orthogonal to
% accelerometer estimate
Vm = cross(acc, mag);
Vm = Vm / norm(Vm);

mx = Vm(1);my = Vm(2); mz = Vm(3);

% dqMag =  [2*q1^3 + 2*q1*q2^2 + 2*q1*q3^2 + 2*q1*q4^2 - 2*my*q1 + 2*mz*q2 - 2*mx*q4
%     2*q1^2*q2 + 2*mz*q1 + 2*q2^3 + 2*q2*q3^2 + 2*q2*q4^2 + 2*my*q2 - 2*mx*q3
%     2*q1^2*q3 + 2*q2^2*q3 - 2*mx*q2 + 2*q3^3 + 2*q3*q4^2 - 2*my*q3 - 2*mz*q4
%     2*q1^2*q4 - 2*mx*q1 + 2*q2^2*q4 + 2*q3^2*q4 - 2*mz*q3 + 2*q4^3 + 2*my*q4];

dqMag = -[2*mx*q4 - 2*mz*q2 + q1*(2*my - 2)
2*mx*q3 - 2*mz*q1 - q2*(2*my + 2)
2*mx*q2 + 2*mz*q4 + q3*(2*my - 2)
2*mx*q1 + 2*mz*q3 - q4*(2*my + 2)];

% R = blkdiag(Sigma_a,Sigma_m);
% Jbig = blkdiag(Ja,Jm);
% R = Jbig * R * Jbig.';
% R = blkdiag(Ja * Sigma_a * Ja.',Jm * Sigma_m * Jm.');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dqAcc = dqAcc ./ norm(dqAcc);
% dqMag = dqMag ./ norm(dqMag);

E = -[dqAcc;dqMag];
% E = E + [q.';q.'];
% E = E / 4;
H = [eye(4);eye(4)];


% E = -[dqAcc;dqMag];


[qPredicted,A] = stateFcn(q.',gyro,dt);


% E = E - H*qPredicted;

Q = stateCovariance(q,dt,Sigma_g);

% RA = diag([Sigma_a(1,1) Sigma_a(1,1) Sigma_a(1,1) Sigma_a(1,1)]);
% RM = diag([Sigma_m(1,1) Sigma_m(1,1) Sigma_m(1,1) Sigma_m(1,1)]);
% R = blkdiag(RA,RM);
% RD = R;
% % 
% R(4,4) = 0.1;
% R(2,2) = 0.1;

% R(5,5) = 0.01;



% J = [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        -2*q3,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           2*q2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          2*q1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             0
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         2*q4,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           2*q1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         -2*q2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             0
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         -2*q1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           2*q4,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         -2*q3,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             0
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          2*q2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           2*q3,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          2*q4,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             0
% (q2*(2*my*(ax*my - ay*mx) + 2*mz*(ax*mz - az*mx))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (2*my*q2)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - q1*((2*mz)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - ((2*my*(ax*my - ay*mx) + 2*mz*(ax*mz - az*mx))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)) - (q4*(2*my*(ax*my - ay*mx) + 2*mz*(ax*mz - az*mx))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), (2*mx*q2)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) + (2*mz*q4)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - (q2*(2*mx*(ax*my - ay*mx) - 2*mz*(ay*mz - az*my))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (q1*(2*mx*(ax*my - ay*mx) - 2*mz*(ay*mz - az*my))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) + (q4*(2*mx*(ax*my - ay*mx) - 2*mz*(ay*mz - az*my))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), q1*((2*mx)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - ((2*mx*(ax*mz - az*mx) + 2*my*(ay*mz - az*my))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)) - (2*my*q4)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - (q2*(2*mx*(ax*mz - az*mx) + 2*my*(ay*mz - az*my))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) + (q4*(2*mx*(ax*mz - az*mx) + 2*my*(ay*mz - az*my))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), q1*((2*az)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - ((2*ay*(ax*my - ay*mx) + 2*az*(ax*mz - az*mx))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)) + (2*ay*q2)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - (q2*(2*ay*(ax*my - ay*mx) + 2*az*(ax*mz - az*mx))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) + (q4*(2*ay*(ax*my - ay*mx) + 2*az*(ax*mz - az*mx))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), (q2*(2*ax*(ax*my - ay*mx) - 2*az*(ay*mz - az*my))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (2*az*q4)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - (2*ax*q2)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) + (q1*(2*ax*(ax*my - ay*mx) - 2*az*(ay*mz - az*my))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (q4*(2*ax*(ax*my - ay*mx) - 2*az*(ay*mz - az*my))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), (2*ay*q4)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - q1*((2*ax)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - ((2*ax*(ax*mz - az*mx) + 2*ay*(ay*mz - az*my))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)) + (q2*(2*ax*(ax*mz - az*mx) + 2*ay*(ay*mz - az*my))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (q4*(2*ax*(ax*mz - az*mx) + 2*ay*(ay*mz - az*my))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)
% q2*((2*mz)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - ((2*my*(ax*my - ay*mx) + 2*mz*(ax*mz - az*mx))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)) - (2*my*q1)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) + (q1*(2*my*(ax*my - ay*mx) + 2*mz*(ax*mz - az*mx))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (q3*(2*my*(ax*my - ay*mx) + 2*mz*(ax*mz - az*mx))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), (2*mx*q1)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) + (2*mz*q3)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - (q1*(2*mx*(ax*my - ay*mx) - 2*mz*(ay*mz - az*my))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) + (q2*(2*mx*(ax*my - ay*mx) - 2*mz*(ay*mz - az*my))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) + (q3*(2*mx*(ax*my - ay*mx) - 2*mz*(ay*mz - az*my))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), (q3*(2*mx*(ax*mz - az*mx) + 2*my*(ay*mz - az*my))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (2*my*q3)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - (q1*(2*mx*(ax*mz - az*mx) + 2*my*(ay*mz - az*my))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - q2*((2*mx)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - ((2*mx*(ax*mz - az*mx) + 2*my*(ay*mz - az*my))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)), (2*ay*q1)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - q2*((2*az)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - ((2*ay*(ax*my - ay*mx) + 2*az*(ax*mz - az*mx))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)) - (q1*(2*ay*(ax*my - ay*mx) + 2*az*(ax*mz - az*mx))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) + (q3*(2*ay*(ax*my - ay*mx) + 2*az*(ax*mz - az*mx))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), (q1*(2*ax*(ax*my - ay*mx) - 2*az*(ay*mz - az*my))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (2*az*q3)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - (2*ax*q1)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - (q2*(2*ax*(ax*my - ay*mx) - 2*az*(ay*mz - az*my))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (q3*(2*ax*(ax*my - ay*mx) - 2*az*(ay*mz - az*my))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), q2*((2*ax)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - ((2*ax*(ax*mz - az*mx) + 2*ay*(ay*mz - az*my))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)) + (2*ay*q3)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) + (q1*(2*ax*(ax*mz - az*mx) + 2*ay*(ay*mz - az*my))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (q3*(2*ax*(ax*mz - az*mx) + 2*ay*(ay*mz - az*my))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)
% (2*my*q4)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - q3*((2*mz)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - ((2*my*(ax*my - ay*mx) + 2*mz*(ax*mz - az*mx))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)) - (q4*(2*my*(ax*my - ay*mx) + 2*mz*(ax*mz - az*mx))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (q2*(2*my*(ax*my - ay*mx) + 2*mz*(ax*mz - az*mx))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), (2*mz*q2)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - (2*mx*q4)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) + (q4*(2*mx*(ax*my - ay*mx) - 2*mz*(ay*mz - az*my))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (q3*(2*mx*(ax*my - ay*mx) - 2*mz*(ay*mz - az*my))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) + (q2*(2*mx*(ax*my - ay*mx) - 2*mz*(ay*mz - az*my))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), q3*((2*mx)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - ((2*mx*(ax*mz - az*mx) + 2*my*(ay*mz - az*my))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)) - (2*my*q2)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) + (q4*(2*mx*(ax*mz - az*mx) + 2*my*(ay*mz - az*my))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) + (q2*(2*mx*(ax*mz - az*mx) + 2*my*(ay*mz - az*my))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), q3*((2*az)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - ((2*ay*(ax*my - ay*mx) + 2*az*(ax*mz - az*mx))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)) - (2*ay*q4)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) + (q4*(2*ay*(ax*my - ay*mx) + 2*az*(ax*mz - az*mx))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) + (q2*(2*ay*(ax*my - ay*mx) + 2*az*(ax*mz - az*mx))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), (2*ax*q4)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - (2*az*q2)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - (q4*(2*ax*(ax*my - ay*mx) - 2*az*(ay*mz - az*my))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) + (q3*(2*ax*(ax*my - ay*mx) - 2*az*(ay*mz - az*my))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (q2*(2*ax*(ax*my - ay*mx) - 2*az*(ay*mz - az*my))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), (2*ay*q2)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - q3*((2*ax)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - ((2*ax*(ax*mz - az*mx) + 2*ay*(ay*mz - az*my))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)) - (q4*(2*ax*(ax*mz - az*mx) + 2*ay*(ay*mz - az*my))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (q2*(2*ax*(ax*mz - az*mx) + 2*ay*(ay*mz - az*my))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)
% q4*((2*mz)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - ((2*my*(ax*my - ay*mx) + 2*mz*(ax*mz - az*mx))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)) + (2*my*q3)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - (q3*(2*my*(ax*my - ay*mx) + 2*mz*(ax*mz - az*mx))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (q1*(2*my*(ax*my - ay*mx) + 2*mz*(ax*mz - az*mx))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), (2*mz*q1)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - (2*mx*q3)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) + (q3*(2*mx*(ax*my - ay*mx) - 2*mz*(ay*mz - az*my))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) + (q4*(2*mx*(ax*my - ay*mx) - 2*mz*(ay*mz - az*my))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) + (q1*(2*mx*(ax*my - ay*mx) - 2*mz*(ay*mz - az*my))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), (q3*(2*mx*(ax*mz - az*mx) + 2*my*(ay*mz - az*my))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (2*my*q1)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - q4*((2*mx)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - ((2*mx*(ax*mz - az*mx) + 2*my*(ay*mz - az*my))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)) + (q1*(2*mx*(ax*mz - az*mx) + 2*my*(ay*mz - az*my))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), (q3*(2*ay*(ax*my - ay*mx) + 2*az*(ax*mz - az*mx))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (2*ay*q3)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - q4*((2*az)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - ((2*ay*(ax*my - ay*mx) + 2*az*(ax*mz - az*mx))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)) + (q1*(2*ay*(ax*my - ay*mx) + 2*az*(ax*mz - az*mx))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), (2*ax*q3)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - (2*az*q1)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - (q3*(2*ax*(ax*my - ay*mx) - 2*az*(ay*mz - az*my))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (q4*(2*ax*(ax*my - ay*mx) - 2*az*(ay*mz - az*my))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (q1*(2*ax*(ax*my - ay*mx) - 2*az*(ay*mz - az*my))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2), q4*((2*ax)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - ((2*ax*(ax*mz - az*mx) + 2*ay*(ay*mz - az*my))*(ax*mz - az*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)) + (2*ay*q1)/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(1/2) - (q3*(2*ax*(ax*mz - az*mx) + 2*ay*(ay*mz - az*my))*(ax*my - ay*mx))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2) - (q1*(2*ax*(ax*mz - az*mx) + 2*ay*(ay*mz - az*my))*(ay*mz - az*my))/((ax*my - ay*mx)^2 + (ax*mz - az*mx)^2 + (ay*mz - az*my)^2)^(3/2)];





J = [-2*q3, 2*q2,  2*q1,     0,     0,     0
 2*q4, 2*q1, -2*q2,     0,     0,     0
-2*q1, 2*q4, -2*q3,     0,     0,     0
 2*q2, 2*q3,  2*q4,     0,     0,     0
    0,    0,     0, -2*q4, -2*q1,  2*q2
    0,    0,     0, -2*q3,  2*q2,  2*q1
    0,    0,     0, -2*q2, -2*q3, -2*q4
    0,    0,     0, -2*q1,  2*q4, -2*q3];
% % J(4,4) = -J(4,4);
% J = J / 2;
% 
RA = diag([Sigma_a(1,1) Sigma_a(1,1) Sigma_a(1,1)]);
RM = diag([Sigma_m(1,1) Sigma_m(1,1) Sigma_m(1,1)]);
R = J * blkdiag(RA,RM) * J.';

Ra = Sigma_a(1,1)^2 ./ R(1:4,1:4);
Rm = Sigma_m(1,1)^2 ./ R(5:8,5:8);
R = blkdiag(Ra,Rm);


R(1:4,5:8) = 100000;
R(5:8,1:4) = 100000;

% Rd = diag(R);
% R = 100000*(ones(8)-eye(8));
% R = R + Rd;

% aFac = Sigma_a(1,1)/Sigma_g(1,1);
% mFac=  Sigma_m(1,1)/Sigma_g(1,1);
% R = R + blkdiag(aFac*Q,mFac*Q)/dt/dt*4/2;
% dgR = diag(diag(R));
% R = R - dgR + blkdiag(diag([Sigma_a(1,1) Sigma_a(1,1) Sigma_a(1,1) Sigma_a(1,1)]),diag([Sigma_m(1,1) Sigma_m(1,1) Sigma_m(1,1) Sigma_m(1,1)]));
% R = R + eye(8)*0.02;

% R = 1/4 * J * blkdiag(RA,RM) * J.';
% R = diag(diag(R));

% R = RD;
% R = blkdiag(aFac*Q,mFac*Q)/dt/dt*4/2;

% ra = Sigma_a(1,1);
% rm = Sigma_m(1,1);
% NN = 1e6;
% % R = diag([ra ra ra NN rm NN rm rm]);
% % R = diag([ra ra ra rm rm rm NN rm]);
% % R = diag([ra ra ra NN rm rm NN rm]);
% R = diag([ra ra ra NN rm NN NN rm]);

% H([4 6],:) = [];
% R = diag([ra ra ra rm rm rm]);
% E([4 6]) = [];
% R([4 6],[4 6]) = [];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Treat gyro as a measurement
% [qG] = stateFcn(q.',gyro,dt);
% % Error quaternions
% E = [-dqAcc;-dqMag;qG - q.'];
% % Measuremnt Model
% H = [eye(4);eye(4);eye(4)];
% 
% % Process model
% A = eye(4);
% Q = stateCovariance(q,dt,Sigma_g);
% % Q = zeros(4);
% % P = diag([0.001 0.001 0.001 0.001]);
% qPredicted = q.';
% 
% % [qPredicted,A] = stateFcn(q.',gyro,dt);
% % Q = stateCovariance(q,dt,Sigma_g);
% 
% % Measurement Model
% RA = diag([Sigma_a(1,1) Sigma_a(1,1) Sigma_a(1,1) Sigma_a(1,1)]);
% RM = diag([Sigma_m(1,1) Sigma_m(1,1) Sigma_m(1,1) Sigma_m(1,1)]);
% RG = stateCovariance(q,dt,Sigma_g);
% R = blkdiag(RA,RM,RG);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

P_ = A*P*A.' + Q;
S = H*P_*H.' + R;

% P_ = Q;
% S = H*P_*H.' + R;

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
