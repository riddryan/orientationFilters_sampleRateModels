function E = gammaModel(phi,fs)
% %% Alpha/beta
% phi = real(phi);
% alpha = phi(1);
% beta = phi(2);
% 
% try E = (fs.^(alpha-1).*exp(-beta*fs).*beta^(alpha))./gamma(alpha);
% catch
%     keyboard
% end
% 
% E(isnan(E)) = 0;
%% K/theta
phi = real(phi);
k = phi(1);
theta = phi(2);
E = (fs.^(k-1)) .* exp(-fs/theta) ./ (theta^k * gamma(k));

if length(phi) > 2
    offset = phi(3);
    E = E + offset;
end

E(isnan(E)) = 0;