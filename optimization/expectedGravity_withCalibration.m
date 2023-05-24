function [g,K,T,b,gravRot] = expectedGravity_withCalibration(pitch,roll,params,worldGrav)
% Estimate gravity vector using calibration parameters applied to various
% orientations. Based on Panahandeh 2010
%
% Required data of the system:
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

if nargin < 3
    params = [1;1;1;0;0;0;0;0;0];
end
if nargin < 4
    worldGrav = 1;
%     worldGrav = 9.80665;
end

% Rotate gravity into frame of sensor
ex1 = cos(pitch.').*sin(roll.');
ex2 = cos(pitch.').*cos(roll.');
gravRot = [-sin(pitch.');ex1;ex2] * worldGrav;

% Apply calibration parameters
if length(params)>3
kx = params(1);
ky = params(2);
kz = params(3);
ax_y = params(4);
ax_z = params(5);
ay_z = params(6);

% Matrix to scale each axis 
K = diag([kx ky kz]);
else
ax_y = params(1);
ax_z = params(2);
ay_z = params(3);
K = eye(3);

end
if length(params) > 6
    bx = params(7);
    by = params(8);
    bz = params(9);
    % Offset for each axis
    b = [bx;by;bz];
else
    b = [0;0;0];
end

% Matrix to enforce orthogonality of axis
T = [1 -ax_y ax_z
     0  1    -ay_z
     0  0        1];




% Combine the three upon the expected gravity vector
g = K/T*gravRot + b;
g = g.';

gravRot = gravRot.';

