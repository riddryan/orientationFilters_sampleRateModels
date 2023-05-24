function mocap = alignMocapToWorld(vicondata,IDX,alignmentinfo)
% Align motion capture vicon data to the gravity and magnetic fields.
%
% vicondata is a structure with fields x, y, and z representing cartesian
% coordinates of the markers' position. Each field is a NxM matrix where N
% is the number of time points, and M is the number of markets.
%
% IDX is the range of data to perform the alignment on. If IDX is provided
% as a scalar, assume that it is the starting index of the data, and that
% the end of the range should be the final time point (N).
%
% alignmentinfo is a structure with fields
% magfield_heading: scalar specifying heading of earth's magnetic field
% q_gravity: quaternion specifying the correction to the direction of
% gravity

if isscalar(IDX)
    IDX = IDX:size(vicondata.x,1);
end

% Select desired data range
x=vicondata.x(IDX,:);
y=vicondata.y(IDX,:);
z=vicondata.z(IDX,:);

% correct volume for gravity aligment and average heading angle
% derived from mapping magnetic field. Note that heading was
% assessed after z- or gravity vector correction.
q_heading = quaternion([alignmentinfo.magfield_heading,0,0],'eulerd','ZXY','point');
% This can be summarized as follows: -- check -- ok (compounding rotations)
Qcorrection = conj(alignmentinfo.q_gravity) * q_heading;

% Apply rotation to mocap data to align with gravity and magnetic field
mocap = rotateframe(Qcorrection,[ x(:) y(:) z(:)]);
[N,M] = size(x);


% Output data as Nx3xM
x = reshape(mocap(:,1),N,M);
y = reshape(mocap(:,2),N,M);
z = reshape(mocap(:,3),N,M);
mocap = cat(3,x,y,z);
mocap = permute(mocap,[1 3 2]);
