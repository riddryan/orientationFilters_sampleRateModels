function E = rat12model(phi,fs)
%% K/theta

p1 = phi(1);
p0 = phi(2);

q1 = phi(3);
q0 = phi(4);

E = (p1*fs + p0) ./ (fs.^2 + q1*fs + q0);
E(isnan(E)) = 0;
% E(isinf(E)) = 100000;