function [q_mocap] = alignMocapToSensors(sensorMarkers,alignmentinfo,sensorname)
% Estimate an inertial frame for the cluster
clusterOrientation = frameFrom4markers(sensorMarkers);

% R = clusterOrientation;
% RF = [-R(:,3,:) -R(:,2,:) -R(:,1,:)];
% q_mocap = quaternion(RF,'rotmat','point');


% Quaternion form
q_mocap = quaternion(clusterOrientation ,'rotmat','point');
% % Adjust to match sensor axes
q_mocap = q_mocap*alignmentinfo.(sensorname).q_adjust;

