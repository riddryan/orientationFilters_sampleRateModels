Ra = diag(rand(1)*ones(1,4));
% Rm = diag(rand(1)*ones(1,4));
Rm = Ra;

P = diag(0.001*ones(1,4));

% Separated
Sa = P + Ra;
Sm = P + Rm;

La = P / Sa
Lm = P / Sm

% Combined
H = [eye(4,4);eye(4,4)];
Rbig = blkdiag(Ra,Rm);
S = H*P*H.' + Rbig;
L = P*H.' / S

% Combined - assuming P (state error covariance) is small
S2 = Rbig;
% L2 = P*H.' / S2