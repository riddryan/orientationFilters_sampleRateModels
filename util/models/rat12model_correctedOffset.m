function E = rat12model_correctedOffset(phi,fs)
% Also have an extra term here (since in the GDF filters the gain is
% originally already multiplied by the sampling period). This extra term
% here allows the estimation of a gain that is independent of sampling
% frequency (i.e. an offset).

p2 = phi(1);
% p1 = phi(2);
p0 = phi(2);

q2 = phi(3);
% q1 = phi(5);
q0 = phi(4);

% c = phi(5);

% E = (p1*fs + p0) ./ (fs.^2 + q1*fs + q0) + c.*fs;

dt = 1./fs;
% E = (p1*dt + p0) ./ (dt.^2 + q1*dt + q0) + c./dt;
% E = (p1*dt + p0) ./ (dt.^2 + q1*dt + q0) + c;

% E = (p2*dt + p1 + p0./dt) ./ (q2*dt.^2 + q1*dt + q0);
E = (p2*dt + p0./dt) ./ (q2*dt.^2 + q0);

E(isnan(E)) = 0;
% E(isinf(E)) = 100000;