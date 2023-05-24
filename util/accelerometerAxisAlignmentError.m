function [E,R] = accelerometerAxisAlignmentError(x,alpha,target)

if nargin < 2
    alpha = [0 0 0];
end
if nargin < 3
    target = 1;
end

R = [1 -alpha(1) alpha(2)
     0 1          -alpha(3)
     0 0             1];


rotX = (R*x.').';

E = mean(vecnorm(rotX,2,2) - target);