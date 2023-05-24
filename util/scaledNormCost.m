function cost = scaledNormCost(x,w,targetNorm)
% Weights (1x3 or 3x1) vector: defining weights for each dimension
% x: Nx3 vector of data points
% cost: mean error between expected vec magnitude of x and target value
% targetNorm
% targetNorm: the value for which x should be equal to magnitude 
if nargin < 2
    w = [1 1 1];
end
if nargin < 3
    targetNorm = 1;
end

if size(w,2)==1
    w = w.';
end

scaledMag = sqrt(sum(w.*(x.^2),2));
error = scaledMag - targetNorm;
cost = mean(error);
end