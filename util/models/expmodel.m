function E = expmodel(phi,fs)
%% K/theta
E = phi(1).*exp(phi(2).*fs) + phi(3);
E(isnan(E)) = 0;
E(isinf(E)) = 1000000;