function E = rat01model(phi,fs)
%% K/theta

p0 = phi(1);

q0 = phi(2);

E = (p0) ./ (fs + q0);
E(isnan(E)) = 0;
% E(isinf(E)) = 100000;