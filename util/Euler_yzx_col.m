function [y,z,x]=Euler_yzx_col(gRfirst, gRsecond)
% % function: [y,z,x]=Euler_yzx_col(gRfirst, gRsecond)
% ------------------------------------------------------------------------- 
% This function calculates Euler angles in the decomposition order depicted
% by the function name. 
% If only one matrix is inputted, the angles of this matrix will be calculated
% with respect to the global global reference frame ([1 0 0   0 1 0   0 0 1]).
% If two matrixes are inputted, the angles of the second matix (gRsecond) with 
% repect to the first matrix will be calculated:
% firstRg=transpose_col(gRfirst);
% firstRsecond=prod_col(firstRg, gRsecond);
% ------------------------------------------------------------------------- 
% % INPUT
% gRfirst : reference orientation matrix in reference frame g (time x 9)
% gRsecond: second orientation matrix of which the Euler angle is calculated 
% with respect to gRfirst (n x 9)
% % OUTPUT
% x: Euler angle about the x-axis (n x 1)
% y: Euler angle about the y-axis (n x 1)
% z: Euler angle about the z-axis (n x 1)
% (n= number of samples) 

% If only one matix is inputted calculate the angles with respect to global
if nargin==1
    gRsecond = gRfirst;
    gRfirst  = [1 0 0   0 1 0   0 0 1];
end

%% find nans
gRfirst(isnan(sum(gRfirst,2)),:)=nan;
gRsecond(isnan(sum(gRsecond,2)),:)=nan;


firstRg=transpose_col(gRfirst);
firstRsecond=prod_col(firstRg, gRsecond);

% matrix=[ 1 4 7
%         2 5 8
%          3 6 9];

% first method
z1 = asin(firstRsecond(:,2));
sx =-firstRsecond(:,8)./cos(z1);
cx = firstRsecond(:,5)./cos(z1);
x1 = atan2(sx,cx);
sy =-firstRsecond(:,3)./cos(z1);
cy = firstRsecond(:,1)./cos(z1);
y1 = atan2(sy,cy);

% second method
z2 =-pi - z1;
i = find(z1)>=0;
z2(i) = pi - z1(i);
sx =-firstRsecond(:,8)./cos(z2);
cx = firstRsecond(:,5)./cos(z2);
x2 = atan2(sx,cx);
sy =-firstRsecond(:,3)./cos(z2);
cy = firstRsecond(:,1)./cos(z2);
y2 = atan2(sy,cy);

% chose which one to use
y = y2;
z = z2;
x = x2;
i2 = find((abs(y1)+abs(z1)+abs(x1)) <= (abs(y2)+abs(z2)+abs(x2)));
y(i2) = y1(i2);
z(i2) = z1(i2);
x(i2) = x1(i2);