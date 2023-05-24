function R = frameFrom4markers(markers)
% Estimate an inertial frame based on a cluster of 4 markers.
%
% MARKERS is a Nx3xM representing a time series (length N) of the position
% of M markers. The order of the markers along the third dimension (1
% through M) is assumed to be top left, top right, bottom right, bottom
% left. Although this procedure should still work with any 4 markers that
% are not in a line.
%
% R is a 3x3xN matrix representing the orientation matrix at each time
% point

%%%%%%%%%%
% Extract orientation of marker-cluster on sensor
%%%%%%%%%%
topleft = markers(:,:,1);
topright = markers(:,:,2);
bottomright = markers(:,:,3);
bottomleft = markers(:,:,4);

za = (topleft + topright) - (bottomleft + bottomright);
xa = cross(bottomright - topleft, bottomleft - topright);
ya = cross(za,xa);

% Normalize direction vectors
xa = xa ./ vecnorm(xa,2,2);
ya = ya ./ vecnorm(ya,2,2);
za = za ./ vecnorm(za,2,2);



% Orientation matrix of cluster (3x3xN)
R=[reshape(xa',3,1,size(xa,1)) reshape(ya',3,1,size(xa,1)) reshape(za',3,1,size(xa,1))];