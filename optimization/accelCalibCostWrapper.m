function cost = accelCalibCostWrapper(accel,angles,params,gravMag,verbose)
% Estimate error between expected gravity and measured gravity. Wrapper
% function to accept optimization input from matlab functions.
%
% accel: Nx3 vector of acceleration data to be used for calibration. SHOULD
% IDEALLY BE DATA IN WHICH THERE IS NO MOVEMENT.
%
% angles: a stacked columnvector (length of 2N) of the pitch and roll at
% each time point.
% 1) 1:N - pitch
% 2) N+1:2N - roll

if nargin < 3
    params = [1;1;1;0;0;0;0;0;0];
end
if nargin < 4
    gravMag = 1;
end
if nargin < 5
    verbose = false;
end

N = length(accel);

pitch = angles(1:N,:);
roll = angles(N+1:2*N,:);

g = expectedGravity_withCalibration(pitch,roll,params,gravMag);

cost = accel-g;

if verbose
figure;
subplot(311)
plot(accel(:,1));
hold on
plot(g(:,1))
subplot(312)
plot(accel(:,2));
hold on
plot(g(:,2))
subplot(313)
plot(accel(:,3));
hold on
plot(g(:,3))
% x=1;
end
% params = z(2*N+1:2*N+9);

