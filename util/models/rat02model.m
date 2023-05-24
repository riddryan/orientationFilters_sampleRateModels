function E = rat02model(phi,fs)
%% K/theta

p0 = phi(1);

q1 = phi(2);
q0 = phi(3);

E = (p0) ./ (fs.^2 + q1*fs + q0);
E(isnan(E)) = 0;
% E(isinf(E)) = 100000;