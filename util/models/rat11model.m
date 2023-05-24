function E = rat11model(phi,fs)
%% K/theta

p1 = phi(1);
p0 = phi(2);

q0 = phi(3);

E = (p1*fs + p0) ./ (fs + q0);
E(isnan(E)) = 0;
% E(isinf(E)) = 100000;