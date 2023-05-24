function E = rat22modelc(phi,fs)
%% K/theta

% p2 = phi(1);
% p1 = phi(2);
% p0 = phi(3);
% 
% q2 = phi(4);
% q1 = phi(5);
% q0 = phi(6);

p2 = phi(1);
p0 = phi(2);

q2 = phi(3);
q0 = phi(4); 

dt = 1./fs;

% E = (p2*dt.^2 + p1*dt + p0) ./ (q2*dt.^2 + q1*dt + q0);
E = (p2*dt.^2 + p0) ./ (q2*dt.^2 + q0);

E = E ./ dt;
E(isnan(E)) = 0;
E(isinf(E)) = 100000;