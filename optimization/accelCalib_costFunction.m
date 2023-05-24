function E = accelCalib_costFunction(accelData,pitch,roll,params,target)
% Estimate error between expected magnitude of acceleration (target) and
% actual magnitude of acceleration, based on several parameters for
% acceleromter calibration. Based on Panahandeh 2010
%
% accelData: Nx3 array of accelerometer data when there IS NOT MUCH
% MOVEMENT
%
% Additional required data:
% pitch: pitch angle in radians at each time point.
% roll: roll angle in radians at each time point.
%
% params: 9x1 vector array specifying parameters for the calibration. In
% order:
% 1) kx: scale x axis
% 2) ky: scale y axis
% 3) kz: scale z axis
% 4) ax_y: amount of corruption of x-axis not being orthogonal to y axis
% 5) ax_z: amount of corruption of x-axis not being orthgonal to z axis
% 6) ay_z: amount of corruption of y-axis not being orthogonal to z axis
% 7) bx: bias x axis
% 8) by: bias y axis
% 9) bz: bias z axis

if nargin < 4
    params = [1;1;1;0;0;0;0;0;0];
end
if nargin < 5
    target = [0 0 1];
end

% Rotate gravity into frame of sensor
ex = cos(pitch)*sin(roll);
gravRot = [-sin(pitch);ex;ex];

% Apply calibration parameters
kx = params(1);
ky = params(2);
kz = params(3);
ax_y = params(4);
ax_z = params(5);
ay_z = params(6);
bx = params(7);
by = params(8);
bz = params(9);

T = [1 -ax_y ax_z
     0  1    -ay_z
     0  0        1];
K = diag(kx,ky,kz);
b = [bx;by;bz];

u = K/T*gravRot + b;

