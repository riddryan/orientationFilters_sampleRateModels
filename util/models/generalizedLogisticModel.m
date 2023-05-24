function E = generalizedLogisticModel(phi,fs)
A = phi(1); %lower asymptote
K = phi(2); %upper asymptote
C = 1;
Q = phi(3); % y-intercept
lambda = phi(4); % Growth rate
v = 1;


E = A + (K-A)./ ((C + Q*exp(-lambda*fs)).^v);

E(isnan(E)) = 0;
E(isinf(E)) = 100000;