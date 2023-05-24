
%% Segment from global rotation

% Define global (G) and segment (A) frames
RG = eye(3);
RA = rotz(30);
% RA = rotx(50)*rotz(30);

% Convert to quaternions
qg = rotm2quat(RG);
qa = rotm2quat(RA);

% Convert to euler angles
eaG = eulerd(quaternion(qg),'ZYX','point');
eaA = eulerd(quaternion(qa),'ZYX','point');

% Compare to inverse rotation
RAT = RA.';
qat = rotm2quat(RAT);
eaAT = eulerd(quaternion(qat),'ZYX','frame');