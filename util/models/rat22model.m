function E = rat22model(phi,fs)
%% K/theta

p2 = phi(1);
p1 = phi(2);
p0 = phi(3);

q1 = phi(4);
q0 = phi(5);

E = (p2*fs.^2 + p1*fs + p0) ./ (fs.^2 + q1*fs + q0);
E(isnan(E)) = 0;
E(isinf(E)) = 100000;