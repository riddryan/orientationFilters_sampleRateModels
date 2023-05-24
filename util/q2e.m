function e = q2e(q,rotSeq,rotType,wrapFlag)
% convert quaternion to euler angles. q is a quaternion. i,j, and k are
% the euler seq. EG a ZYX sequence would be i = 3, j = 2, k = 1.
%
% Taken from: 
% 
% Quaternion to Euler angles conversion: A direct, general and
% computationally efficient method EvandroBernardes ID , Ste´phane Viollet
% (2022)

if nargin < 4
    wrapFlag = true;
end

switch rotType
    case 'frame'
        q(:,2) = -q(:,2);
        q(:,3) = -q(:,3);
        q(:,4) = -q(:,4);
end

if ~ischar(rotSeq) || length(rotSeq) ~= 3
    error('rotSeq must be a length 3 char representing desired euler sequence')
end

for jj = 1 : 3
switch rotSeq(jj)
    case 'X'
        R(jj) = 1;
    case 'Y'
        R(jj) = 2;
    case 'Z'
        R(jj) = 3;
    otherwise
        error('Only Z, Y, X are valid');
end
end
i = R(1);
j = R(2);
k = R(3);

if i == k
    not_proper = false;
    k = 6-i-j;
else
    not_proper = true;
end

eps = (i-j) * (j - k) * (k-i)/2;

if not_proper
    a = q(:,1) - q(:,j+1);
    b = q(:,i+1) + q(:,k+1)*eps;
    c = q(:,j+1) + q(:,1);
    d = q(:,k+1)*eps - q(:,i+1);
else
    a = q(:,1);
    b = q(:,i+1);
    c = q(:,j+1);
    d = q(:,k+1);
end

num = a.^2 + b.^2;
den = a.^2 + b.^2 + c.^2 + d.^2;
theta2 = acos(2*(num./den) - 1);

thetap = atan2(b,a);
thetam = atan2(d,c);


theta1 = thetap - thetam;
theta3 = thetap + thetam;

dex1 = theta2 == 0;
theta1(dex1) = 0;
theta3(dex1) = 2*thetap(dex1) - theta1(dex1);

dex2 = theta2 == pi/2;
theta1(dex2) = 0;
theta3(dex2) = 2*thetam(dex2) + theta1(dex2);



if not_proper
    theta3 = eps*theta3;
    theta2 = theta2 - pi/2;
end

e = [theta1 theta2 theta3];

switch rotType
    case 'frame'
    e = -e;
end

if wrapFlag
e = wrapToPi(e);
end
