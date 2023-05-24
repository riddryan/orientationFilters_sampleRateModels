function E = optimalKalman2model(phi,fs)
% Also have an extra term here (since in the GDF filters the gain is
% originally already multiplied by the sampling period). This extra term
% here allows the estimation of a gain that is independent of sampling
% frequency (i.e. an offset).

n2 = phi(1);
d0 = phi(2);

% n2 = phi(1);
% n0 = phi(2);
% d0 = phi(3);

% E = (p1*fs + p0) ./ (fs.^2 + q1*fs + q0) + c.*fs;

dt = 1./fs;
E = (n2*dt.^2) ./ (n2*dt.^2 + d0);
% E = (n2*dt + n0./dt) ./ (n2*dt.^2 + d0);
% E = (n2*dt.^2 + n0) ./ (n2*dt.^2 + d0);

% E = (p1*dt + p0) ./ (dt.^2 + q1*dt + q0) + c./dt;
E(isnan(E)) = 0;
E(isinf(E)) = 100000;